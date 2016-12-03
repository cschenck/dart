


#include <iostream>
#include <map>
#include <memory>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <GL/glew.h>
#include <GL/glut.h>

#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_gl_interop.h>



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
    // First parse the command line args.
    string depth_topic;
    string rgb_topic;
    string cmd_topic;
    string out_topic;
    string url;
    string web_dir;
    string marker_topic;
    string topic_prefix;
    string joint_topic;
    bool use_viz;
    try
    {
        TCLAP::CmdLine cmd("DART with ROS", ' ', "0.1");
        TCLAP::ValueArg<std::string> dt("","depth_topic","The ROS topic to listen to for depth data.",false,"/camera/depth_registered/image_raw","string");
        TCLAP::ValueArg<std::string> rt("","rgb_topic","The ROS topic to listen to for rgb data.",false,"/camera/rgb/image_color","string");
        TCLAP::ValueArg<std::string> ct("","cmd_topic","The ROS topic to listen to for tracking commands.",false,"/dart/cmd","string");
        TCLAP::ValueArg<std::string> pt("","pose_topic","The ROS topic to publish object poses to.",false,"/dart/pose","string");
        TCLAP::ValueArg<std::string> va_url("","url","The url that rviz can use to access the webserver on this machine. This is used to pass the model files to rviz.",false,"","string");
        TCLAP::ValueArg<std::string> va_wd("","web_dir","The folder on this machine corresponding to the --url flag. The model files will be written here.",false,"","string");
        TCLAP::SwitchArg viz("","viz","Run the marker publisher to visualize the dart state in rviz.", false);
        TCLAP::ValueArg<std::string> mt("","marker_topic","The topic the marker publisher will publish the model markers to.", false, "dart_markers", "string");
        TCLAP::ValueArg<std::string> prefix("","topic_prefix","Prefix to append to all INPUT ros topics, e.g., PREFIX/TOPIC. Useful when replaying rosbags.", false, "", "string");
        TCLAP::ValueArg<std::string> jt("j","joint_state_topic","The topic to listen to for the state of the robot joints.", false, "/robot/joint_states", "string");
        cmd.add(dt);
        cmd.add(rt);
        cmd.add(ct);
        cmd.add(pt);
        cmd.add(va_url);
        cmd.add(va_wd);
        cmd.add(viz);
        cmd.add(mt);
        cmd.add(prefix);
        cmd.add(jt);
        cmd.parse(argc, argv);
        depth_topic = dt.getValue();
        rgb_topic = rt.getValue();
        cmd_topic = ct.getValue();
        out_topic = pt.getValue();
        url = va_url.getValue();
        web_dir = va_wd.getValue();
        use_viz = viz.getValue();
        marker_topic = mt.getValue();
        topic_prefix = prefix.getValue();
        joint_topic = jt.getValue();
    }
    catch(TCLAP::ArgException &e)
    {
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
        return 1;
    }
    
    depth_topic = combine_paths(topic_prefix, depth_topic);
    rgb_topic = combine_paths(topic_prefix, rgb_topic);
    cmd_topic = combine_paths(topic_prefix, cmd_topic);
    joint_topic = combine_paths(topic_prefix, joint_topic);
    
    cout << "Listening for depth frames on topic: " << depth_topic << endl;
    cout << "Listening for rgb frames on topic: " << rgb_topic << endl;
    cout << "Listening for commands on topic: " << cmd_topic << endl;
    cout << "Publishing poses to topic: " << out_topic << endl;
    if(use_viz)
    {
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
    }
    
    // Set up ROS.
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
    
    MarkerPublisher* mp = NULL;
    if(use_viz)
        mp = new MarkerPublisher(url, web_dir, marker_topic);
    
    
    printf("Running...\n");
    while(ros::ok())
    {
        
        if(use_viz)
            mp->update(tracker, depthSource->getHeader());
    }
    
    depthSource->stopRosSpinner();
    ros::shutdown();
    
    if(mp != NULL)
        delete mp;
        
    
        
    return 0;
}
    
    






























