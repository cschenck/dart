#include <iostream>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <GL/glew.h>
#include <GL/glut.h>

#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_gl_interop.h>

#include <ros/ros.h>

#include "joint_state_listener.h"
#include "marker_publisher.h"
#include "pointcloud_processor.h"
#include "ROSDepthSource.h"
#include "tracker.h"
#include "tclap/CmdLine.h"
#include "util.h"
#include "util/dart_io.h"

#define EIGEN_DONT_ALIGN

#define LEFT_HAND_FRAME 16
#define RIGHT_HAND_FRAME 7

using namespace std;

dart::PoseReduction* default_pose_reduction(const dart::Model& model)
{
    int nDimensions = model.getPoseDimensionality();
    std::vector<float> jointMins, jointMaxs;
    std::vector<std::string> jointNames;
    for (int j=0; j<model.getNumJoints(); ++j) {
        jointMins.push_back(model.getJointMin(j));
        jointMaxs.push_back(model.getJointMax(j));
        jointNames.push_back(model.getJointName(j));
    }
    dart::PoseReduction* poseReduction = new dart::NullReduction(nDimensions - 6,
        jointMins.data(), jointMaxs.data(), jointNames.data());
    return poseReduction;
}

void match_pose_to_frame(const dart::Model& base, const dart::Pose& pose, int frame, dart::Pose& out)
{
    dart::SE3 full_se3 = base.getTransformModelToCamera();
    dart::SE3 frame_se3 = base.getTransformFrameToModel(frame);
    out.setTransformModelToCamera(full_se3*frame_se3);
    for(int i = 0; i < out.getArticulatedDimensions(); ++i)
    {
        for(int j = 0; j < pose.getArticulatedDimensions(); ++j)
        {
            string n1 = out.getReducedName(i);
            string n2 = pose.getReducedName(j);
            //cout << "'" << n1 << "', '" << n2 << "'" << endl;
            // Check to see if the end of the joint name in full pose corresponds to
            // a joint name in the hand pose.
            if(n2.length() >= n1.length() && n2.compare(n2.length() - n1.length(), n1.length(), n1) == 0)
                out.getArticulation()[i] = pose.getArticulation()[j];
        }
    }
}

int main(int argc, char** argv)
{
    string depth_topic;
    string rgb_topic;
    string url;
    string web_dir;
    string marker_topic;
    string model_fp;
    string bowl_fp;
    string cup_fp;
    string joint_topic;
    string topic_prefix;
    string pc_topic;
    string hand_model;
    string sync_topic;
    bool use_sync;
    
    try
    {
        TCLAP::CmdLine cmd("DART with ROS", ' ', "0.1");
        TCLAP::ValueArg<std::string> dt("d","depth_topic","The ROS topic to listen to for depth data.",false,"/camera/depth_registered/image_raw","string");
        TCLAP::ValueArg<std::string> rt("r","rgb_topic","The ROS topic to listen to for rgb data.",false,"/camera/rgb/image_color","string");
        TCLAP::ValueArg<std::string> va_url("u","url","The url that rviz can use to access the webserver on this machine. This is used to pass the model files to rviz.",false,"","string");
        TCLAP::ValueArg<std::string> va_wd("w","web_dir","The folder on this machine corresponding to the --url flag. The model files will be written here.",false,"","string");
        TCLAP::ValueArg<std::string> mt("m","marker_topic","The topic the marker publisher will publish the model markers to.", false, "dart/markers", "string");
        TCLAP::ValueArg<std::string> jt("j","joint_state_topic","The topic to listen to for the state of the robot joints.", false, "/robot/joint_states", "string");
        TCLAP::ValueArg<std::string> pc("","pointcloud_topic","The topic to publish the pointcloud data to.", false, "/dart/pointcloud", "string");
        TCLAP::ValueArg<std::string> model("o","robot_model","The xml file for the robot model.", true, "", "string");
        TCLAP::ValueArg<std::string> bowl("","bowl_model","The xml file for the bowl model (i.e., object on table).", true, "", "string");
        TCLAP::ValueArg<std::string> cup("","cup_model","The xml file for the cup model (i.e., object in robot's gripper).", true, "", "string");
        TCLAP::ValueArg<std::string> hand("","hand_model","The xml file for the robot's hand.", true, "", "string");
        TCLAP::ValueArg<std::string> prefix("","topic_prefix","Prefix to append to all INPUT ros topics, e.g., PREFIX/TOPIC. Useful when replaying rosbags.", false, "", "string");
        TCLAP::ValueArg<std::string> st("","sync_topic","Topic to send all rosbag sync commands on.", false, "/dart/sync", "string");
        TCLAP::SwitchArg sync("","sync","Send sync commands to rosbag player.", false);
        cmd.add(dt);
        cmd.add(rt);
        cmd.add(va_url);
        cmd.add(va_wd);
        cmd.add(mt);
        cmd.add(model);
        cmd.add(jt);
        cmd.add(prefix);
        cmd.add(pc);
        cmd.add(bowl);
        cmd.add(cup);
        cmd.add(hand);
        cmd.add(st);
        cmd.add(sync);
        cmd.parse(argc, argv);
        depth_topic = dt.getValue();
        rgb_topic = rt.getValue();
        url = va_url.getValue();
        web_dir = va_wd.getValue();
        marker_topic = mt.getValue();
        model_fp = model.getValue();
        joint_topic = jt.getValue();
        topic_prefix = prefix.getValue();
        pc_topic = pc.getValue();
        bowl_fp = bowl.getValue();
        cup_fp = cup.getValue();
        hand_model = hand.getValue();
        sync_topic = st.getValue();
        use_sync = sync.getValue();
    }
    catch(TCLAP::ArgException &e)
    {
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
        return 1;
    }
    
    depth_topic = combine_paths(topic_prefix, depth_topic);
    rgb_topic = combine_paths(topic_prefix, rgb_topic);
    joint_topic = combine_paths(topic_prefix, joint_topic);
    
    cout << "Listening for depth frames on topic: " << depth_topic << endl;
    cout << "Listening for rgb frames on topic: " << rgb_topic << endl;
    cout << "Publishing markers to topic: " << marker_topic << endl;
    cout << "Publishing pointlcoud to topic: " << pc_topic << endl;


    if(web_dir.size() == 0 || url.size() == 0)
    {
        printf("web_dir and url not fully specified, attempting to set defaults...\n");
        if(!set_default_url(&url, &web_dir))
        {
            printf("Unable to set defaults for web_dir and url. Please specify as command line args.\n");
            return 1;
        }
    }
    cout << "Publishing model files at url: " << url << endl;
    cout << "Placing models files in folder: " << web_dir << endl;

    
    ROS_INFO("Starting dart node.");
    ros::init(argc, argv, "dart");
    
    cudaGLSetGLDevice(0);
    cudaDeviceReset();
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowPosition(-10000, -10000);
    glutInitWindowSize(1, 1);
    glutCreateWindow("ExtensionTest");
    glutHideWindow();
    
    
    if(glewInit() != GLEW_OK)
    {
        printf("Glew did not initialize properly.\n");
    }
    
    dart::Tracker tracker;
    auto depthSource = shared_ptr<ROSDepthSource>(new ROSDepthSource());
    depthSource->initialize(depth_topic, rgb_topic, use_sync, sync_topic);
    depthSource->startRosSpinner();
    tracker.addDepthSource(depthSource.get());
    JointStateListener* jsl = new JointStateListener(joint_topic);
    
    MarkerPublisher* mp = new MarkerPublisher(url, web_dir, marker_topic, pc_topic);  
    //mp->showSdfsInstead(true); 
    
    // Find the table.
    PointcloudProcessor pcp;
    if(pcp.findTable(depthSource.get()))
    {
        printf("Found table.\n");
        mp->addUntrackedObject(pcp.getTablePose(), pcp.getTableSize());
        //mp->addUntrackedObject(pcp.table_points, -1);
        //mp->addUntrackedObject(pcp.table_points, 0);
        //mp->addUntrackedObject(pcp.table_points, 1);
    }
    else
    {
        printf("Unable to find table.\n");
        return 1;
    }
    
    if(pcp.findObjectsOnTable(depthSource.get()))
    {
        printf("Found %d objects on the table.\n", pcp.numObjectsOnTable());
        //for(int i = 0; i < pcp.numObjectsOnTable(); ++i)
            //mp->addUntrackedObject(pcp.getObjectPose(i), pcp.getObjectSize(i));
    }
    else
    {
        printf("Unable to find objects on table.\n");
        return 1;
    }
                     
    // Add the robot model
    dart::HostOnlyModel robot_model;
    dart::readModelXML(model_fp.c_str(), robot_model);
    robot_model.computeStructure();
    dart::Pose robot_pose(default_pose_reduction(robot_model));
    // Baxter specific pose.
    robot_pose.setTransformModelToCamera(dart::SE3FromTranslation(-0.181 + 0.2, -0.016 + 0.5, -0.491 + 0.55)*dart::SE3FromQuaternion(-0.630, 0.619, -0.336, -0.326));
    jsl->setModelJoints(&robot_pose);
    robot_model.setPose(robot_pose);
    
    // Add the robot's hand.
    int hand_frame;
    {        
        // For the baxter model, left hand is frame 18 and right is frame 9.
        // See if the left gripper is above the table.
        if((dart::SE3Invert(pcp.getTablePose())*SE3ToTranslation(robot_model.getTransformCameraToModel()*robot_model.getTransformFrameToModel(18))).z > 0)
            hand_frame = LEFT_HAND_FRAME;
        else
            hand_frame = RIGHT_HAND_FRAME;
        tracker.addModel(hand_model);
        dart::Pose& pose = tracker.getPose(0);
        match_pose_to_frame(robot_model, robot_pose, hand_frame, pose);
        tracker.getModel(0).setPose(pose);
    }
    
    // Next add the bowl.
    {
        tracker.addModel(bowl_fp);
        dart::Pose& pose = tracker.getPose(1);
        pose.setTransformModelToCamera(pcp.projectOntoTable(pcp.getObjectPose(0)));
        tracker.getModel(1).setPose(pose);
    }
    
    // Finally add the cup.
    
    dart::SE3 cup_offset;
    {
    // For the baxter model, attach it to frame 18 for the left arm or 9 for the right.
        tracker.addModel(cup_fp);
        dart::Pose& pose = tracker.getPose(2);
        // See if the left gripper is above the table.
        if(hand_frame == LEFT_HAND_FRAME)
            cup_offset = dart::SE3FromTranslation(0,0,0.1)*dart::SE3FromRotationY(-M_PI/2);
        else
            cup_offset = dart::SE3FromTranslation(0,0,0.1)*dart::SE3FromRotationY(-M_PI/2)*dart::SE3FromRotationZ(M_PI);
        pose.setTransformModelToCamera(robot_model.getTransformCameraToModel()*robot_model.getTransformFrameToModel(hand_frame+2)*cup_offset);
        tracker.getModel(2).setPose(pose);
    }
    
    
    
    // Set up the optimization options.
    dart::Optimizer* optimizer = tracker.getOptimizer();
    dart::OptimizationOptions & opts = tracker.getOptions();    
    opts.lambdaObsToMod = 1;
    opts.lambdaModToObs = 1;
    //initialize per model parameters
    opts.distThreshold.resize(tracker.getNumModels());
    opts.regularization.resize(tracker.getNumModels());
    opts.regularizationScaled.resize(tracker.getNumModels());
    opts.planeOffset.resize(tracker.getNumModels());
    opts.planeNormal.resize(tracker.getNumModels());
    opts.lambdaIntersection.resize(tracker.getNumModels()*tracker.getNumModels());
    for(int i = 0; i < tracker.getNumModels(); i += 1)
    {
        opts.distThreshold[i] = 0.03;
        opts.regularization[i] = 0.01;
        opts.regularizationScaled[i] = 1.0;
        opts.planeOffset[i] = -1;
        opts.planeNormal[i] = make_float3(0,0,0);
        for(int j = 0; j < tracker.getNumModels(); j += 1)
        {
            opts.lambdaIntersection[i*tracker.getNumModels() + j] = 1.0;
        } 
    }
    // Set the clipping plane for the bowl to be its far edge.
    //opts.planeNormal[1] = make_float3(0,-1,0);
    //opts.planeOffset[1] = pcp.getObjectSize(0).y/2.0;

    float rate = 30.0;
    ros::Rate r(rate);
    
    bool is_tracking = false;
    int frame = 0;
    
    printf("Running...\n");
    while(ros::ok())
    {
        pcp.computeCloudMask(depthSource.get());
        mp->setCloudMask(pcp.getHostMask());
        
        // Set the robot pose based on the joint angles since it's not tracked.
        robot_pose.setTransformModelToCamera(robot_model.getTransformModelToCamera());
        jsl->setModelJoints(&robot_pose);
        robot_model.setPose(robot_pose);
        
        if(is_tracking)
        {
            // Update pose using DART.
            tracker.maskPointCloud(pcp.getDeviceMask());
            tracker.optimizePoses();
            for (int m=0; m<tracker.getNumModels(); ++m) 
                tracker.updatePose(m);
        }
        else
        {
            dart::Pose& cup_pose = tracker.getPose(2);
            cup_pose.setTransformModelToCamera(robot_model.getTransformModelToCamera()*robot_model.getTransformFrameToModel(hand_frame+2)*cup_offset);
            tracker.getModel(2).setPose(cup_pose);
            dart::Pose& hand_pose = tracker.getPose(0);
            match_pose_to_frame(robot_model, robot_pose, hand_frame, hand_pose);
            tracker.getModel(0).setPose(hand_pose);
            if(frame > rate*1)
            {
                is_tracking = true;
                printf("Starting DART tracking...\n");
            }
        }
        
        auto header = depthSource->getHeader();
        mp->update(tracker, {&robot_model}, header);
        mp->publishPointcloud(depthSource.get(), header);
        tracker.stepForward();
        //printf("%13.2f\n", header.stamp.toSec());
        r.sleep();
        ++frame;
    }
    
    depthSource->stopRosSpinner();
    delete jsl;
    ros::shutdown();
    
    delete mp;
        
    return 0;
}
    
    






























