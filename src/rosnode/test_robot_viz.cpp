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

using namespace std;



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
    string model_inter_fp;
    try
    {
        TCLAP::CmdLine cmd("DART with ROS", ' ', "0.1");
        TCLAP::ValueArg<std::string> dt("d","depth_topic","The ROS topic to listen to for depth data.",false,"/camera/depth_registered/image_raw","string");
        TCLAP::ValueArg<std::string> rt("r","rgb_topic","The ROS topic to listen to for rgb data.",false,"/camera/rgb/image_color","string");
        TCLAP::ValueArg<std::string> va_url("u","url","The url that rviz can use to access the webserver on this machine. This is used to pass the model files to rviz.",false,"","string");
        TCLAP::ValueArg<std::string> va_wd("w","web_dir","The folder on this machine corresponding to the --url flag. The model files will be written here.",false,"","string");
        TCLAP::ValueArg<std::string> mt("m","marker_topic","The topic the marker publisher will publish the model markers to.", false, "dart_markers", "string");
        TCLAP::ValueArg<std::string> jt("j","joint_state_topic","The topic to listen to for the state of the robot joints.", false, "/robot/joint_states", "string");
        TCLAP::ValueArg<std::string> pc("","pointcloud_topic","The topic to publish the pointcloud data to.", false, "/dart/pointcloud", "string");
        TCLAP::ValueArg<std::string> model("o","robot_model","The xml file for the robot model.", true, "", "string");
        TCLAP::ValueArg<std::string> bowl("","bowl_model","The xml file for the bowl model (i.e., object on table).", true, "", "string");
        TCLAP::ValueArg<std::string> cup("","cup_model","The xml file for the cup model (i.e., object in robot's gripper).", true, "", "string");
        TCLAP::ValueArg<std::string> model_inter("","robot_intersection","The text file detailing which frames of the robot model may intersect.", true, "", "string");
        TCLAP::ValueArg<std::string> prefix("","topic_prefix","Prefix to append to all INPUT ros topics, e.g., PREFIX/TOPIC. Useful when replaying rosbags.", false, "", "string");
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
        cmd.add(model_inter);
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
        model_inter_fp = model_inter.getValue();
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
    depthSource->initialize(depth_topic, rgb_topic);
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
        //mp->addUntrackedObject(pcp.getTablePose(), pcp.getTableSize());
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
    {
        tracker.addModel(model_fp);
        dart::Pose& pose = tracker.getPose(0);
        for (int i=0; i<pose.getArticulatedDimensions(); ++i) 
            pose.getArticulation()[i] = (tracker.getModel(0).getJointMin(i) + tracker.getModel(0).getJointMax(i))/2.0;
        // This pose is specific to a Baxter robot with a chest mounted RGBD camera.
        pose.setTransformModelToCamera(dart::SE3FromTranslation(-0.181 + 0.2, -0.016 + 0.5, -0.491 + 0.55)*dart::SE3FromQuaternion(-0.630, 0.619, -0.336, -0.326));
        jsl->setModelJoints(&pose);
        tracker.getModel(0).setPose(pose);
        int * selfIntersectionMatrix = dart::loadSelfIntersectionMatrix(model_inter_fp,tracker.getModel(0).getNumSdfs());
        for(int i = 0; i < tracker.getModel(0).getNumSdfs()*tracker.getModel(0).getNumSdfs(); ++i)
            selfIntersectionMatrix[i] = 0;
        tracker.setIntersectionPotentialMatrix(0,selfIntersectionMatrix);
        delete [] selfIntersectionMatrix;
    }
    
    // Next add the bowl.
    {
        tracker.addModel(bowl_fp);
        dart::Pose& pose = tracker.getPose(1);
        pose.setTransformModelToCamera(pcp.projectOntoTable(pcp.getObjectPose(0)));
        tracker.getModel(1).setPose(pose);
    }
    
    // Finally add the cup.
    int cup_frame;
    dart::SE3 cup_offset;
    {
    // For the baxter model, attach it to frame 18 for the left arm or 9 for the right.
        tracker.addModel(cup_fp);
        dart::Pose& pose = tracker.getPose(2);
        // See if the left gripper is above the table.
        if((dart::SE3Invert(pcp.getTablePose())*SE3ToTranslation(tracker.getModel(0).getTransformCameraToModel()*tracker.getModel(0).getTransformFrameToModel(18))).z > 0)
        {
            cup_frame = 18;
            cup_offset = dart::SE3FromTranslation(0,0,0.1)*dart::SE3FromRotationY(-M_PI/2);
        }
        else
        {
            cup_frame = 9;
            cup_offset = dart::SE3FromTranslation(0,0,0.1)*dart::SE3FromRotationY(-M_PI/2)*dart::SE3FromRotationZ(M_PI);
        }
        pose.setTransformModelToCamera(tracker.getModel(0).getTransformCameraToModel()*tracker.getModel(0).getTransformFrameToModel(cup_frame)*cup_offset);
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
            // Update pose based on joint positions.
            dart::Pose& pose = tracker.getPose(0);
            // Since we're not calling optimzePoses this needs to be set manually.
            pose.setTransformModelToCamera(tracker.getModel(0).getTransformModelToCamera());
            jsl->setModelJoints(&pose);
            tracker.getModel(0).setPose(pose);
            dart::Pose& pose2 = tracker.getPose(2);
            pose2.setTransformModelToCamera(tracker.getModel(0).getTransformModelToCamera()*tracker.getModel(0).getTransformFrameToModel(cup_frame)*cup_offset);
            tracker.getModel(2).setPose(pose2);
            if(frame > rate*3)
            {
                is_tracking = true;
                printf("Starting DART tracking...\n");
            }
        }
        
        auto header = depthSource->getHeader();
        mp->update(tracker, header);
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
    
    






























