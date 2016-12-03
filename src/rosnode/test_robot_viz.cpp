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
    string joint_topic;
    string topic_prefix;
    string pc_topic;
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
    tracker.addModel(model_fp);
                     
    
    dart::Pose pose = tracker.getPose(0);
    for (int i=0; i<pose.getArticulatedDimensions(); ++i) 
        pose.getArticulation()[i] = (tracker.getModel(0).getJointMin(i) + tracker.getModel(0).getJointMax(i))/2.0;

    // This pose is specific to a Baxter robot with a chest mounted RGBD camera.
    pose.setTransformCameraToModel(dart::SE3FromTranslation(-0.181, -0.016, -0.491)*dart::SE3FromQuaternion(-0.630, 0.619, -0.336, -0.326));
    tracker.getModel(0).setPose(pose);

    ros::Rate r(30);
    
    printf("Running...\n");
    while(ros::ok())
    {
        pose = tracker.getPose(0);
        // Since we're not calling optimzePoses this needs to be set manually.
        pose.setTransformCameraToModel(tracker.getModel(0).getTransformCameraToModel());
        jsl->setModelJoints(&pose);
        tracker.getModel(0).setPose(pose);
        auto header = depthSource->getHeader();
        mp->update(tracker, header);
        mp->publishPointcloud(depthSource.get(), header);
        //printf("%13.2f\n", header.stamp.toSec());
        depthSource->advance();
        r.sleep();
    }
    
    depthSource->stopRosSpinner();
    delete jsl;
    ros::shutdown();
    
    delete mp;
        
    return 0;
}
    
    






























