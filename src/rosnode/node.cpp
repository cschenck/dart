#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <GL/glew.h>
#include <GL/glut.h>

#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_gl_interop.h>

#include "ros/ros.h"

#include "geometry/SE3.h"
#include "optimization/optimizer.h"
#include "mesh/mesh.h"
#include "tclap/CmdLine.h"

using namespace std;


int main(int argc, char** argv)
{
    string depth_topic;
    string rgb_topic;
    string cmd_topic;
    string out_topic;
    try
    {
        TCLAP::CmdLine cmd("DART with ROS", ' ', "0.1");
        TCLAP::ValueArg<std::string> dt("d","depth_topic","The ROS topic to listen to for depth data.",false,"/camera/depth_registered/image_raw","string");
        TCLAP::ValueArg<std::string> rt("r","rgb_topic","The ROS topic to listen to for rgb data.",false,"/camera/rgb/image_color","string");
        TCLAP::ValueArg<std::string> ct("c","cmd_topic","The ROS topic to listen to for tracking commands.",false,"/dart/cmd","string");
        TCLAP::ValueArg<std::string> pt("p","pose_topic","The ROS topic to publish object poses to.",false,"/dart/pose","string");
        cmd.add(dt);
        cmd.add(rt);
        cmd.add(ct);
        cmd.add(pt);
        cmd.parse(argc, argv);
        depth_topic = dt.getValue();
        rgb_topic = rt.getValue();
        cmd_topic = ct.getValue();
        out_topic = pt.getValue();
    }
    catch(TCLAP::ArgException &e)
    {
        cerr << "error: " << e.error() << " for arg " << e.argId() << endl;
        return 1;
    }
    
    cout << "Listening for depth frames on topic: " << depth_topic << endl;
    cout << "Listening for rgb frames on topic: " << rgb_topic << endl;
    cout << "Listening for commands on topic: " << cmd_topic << endl;
    cout << "Publishing poses to topic: " << out_topic << endl;
    
    return 0;
}
