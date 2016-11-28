#include <arpa/inet.h>
#include <dirent.h>
#include <ifaddrs.h>
#include <iostream>
#include <map>
#include <memory>
#include <netdb.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

#include <GL/glew.h>
#include <GL/glut.h>

#include <cuda_runtime.h>
#include <cuda.h>
#include <cuda_gl_interop.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "marker_publisher.h"
#include "ROSDepthSource.h"
#include "tracker.h"
#include "tclap/CmdLine.h"
#include "util.h"
#include "util/dart_io.h"

#define EIGEN_DONT_ALIGN

using namespace std;

bool set_default_url(string* url, string* web_dir)
{
    // First get the local ip address.
    struct ifaddrs *ifaddr, *ifa;
    int family, s;
    char host[NI_MAXHOST];
    if(getifaddrs(&ifaddr) == -1) 
    {
        printf("getifaddrs() failed.\n");
        return false;
    }
    for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) 
    {
        if (ifa->ifa_addr == NULL)
            continue;
         s=getnameinfo(ifa->ifa_addr,sizeof(struct sockaddr_in),host, NI_MAXHOST, NULL, 0, NI_NUMERICHOST);

        if((strcmp(host,"127.0.0.1")!=0)&&(ifa->ifa_addr->sa_family==AF_INET))
        {
            if (s != 0)
            {
                printf("getnameinfo() failed: %s\n", gai_strerror(s));
                return false;
            }
            //printf("\tInterface : <%s>\n",ifa->ifa_name );
            //printf("\t  Address : <%s>\n", host); 
            break;
        }
    }
    freeifaddrs(ifaddr);
    
    char username[64];
    getlogin_r(username, 64);
    const char* homedir;
    if((homedir = getenv("HOME")) == NULL)
    {
        printf("HOME environment variable not defined.\n");
        return false;
    }
    
    *web_dir = combine_paths(homedir, "public_html");
    struct stat st = {0};
    if(stat(web_dir->c_str(), &st) == -1)
    {
        printf("Unable to open the public_html folder in this user's home directory. Please ensure you are running a web server on this machine.\n");
        return false;
    }
    
    *web_dir = combine_paths(*web_dir, "meshes");
    if(stat(web_dir->c_str(), &st) == -1)
    {
        mkdir(web_dir->c_str(), 0777);
    }
    
    *url = string("http://") + host + "/~" + username + "/meshes";
    
    return true;
}

map<string, float> global_joint_state;
pthread_mutex_t joint_state_lock;
void setJointState(const sensor_msgs::JointState& msg)
{
    pthread_mutex_lock(&joint_state_lock);
    for(int i = 0; i < msg.name.size(); ++i)
    {
        global_joint_state[msg.name[i]] = msg.position[i];
    }    
    pthread_mutex_unlock(&joint_state_lock);
}

void setModelJoints(dart::Pose* pose)
{    
    pthread_mutex_lock(&joint_state_lock);
    map<string, float> js = global_joint_state;
    pthread_mutex_unlock(&joint_state_lock);
    for(int i = 0; i < pose->getArticulatedDimensions(); ++i)
    {
        string name = pose->getReducedName(i);
        if(js.find(name) != js.end())
        {
            pose->getArticulation()[i] = js[name];
            cout << name << " = " << js[name] << endl;
        }
        else
        {
            printf("Error: Joint name %s not found in ros joint_states message.\n", name.c_str());   
        }
    }
    
}

int main(int argc, char** argv)
{
    string depth_topic;
    string rgb_topic;
    string cmd_topic;
    string out_topic;
    string url;
    string web_dir;
    string marker_topic;
    bool use_viz;
    try
    {
        TCLAP::CmdLine cmd("DART with ROS", ' ', "0.1");
        TCLAP::ValueArg<std::string> dt("d","depth_topic","The ROS topic to listen to for depth data.",false,"/camera/depth_registered/image_raw","string");
        TCLAP::ValueArg<std::string> rt("r","rgb_topic","The ROS topic to listen to for rgb data.",false,"/camera/rgb/image_color","string");
        TCLAP::ValueArg<std::string> ct("c","cmd_topic","The ROS topic to listen to for tracking commands.",false,"/dart/cmd","string");
        TCLAP::ValueArg<std::string> pt("p","pose_topic","The ROS topic to publish object poses to.",false,"/dart/pose","string");
        TCLAP::ValueArg<std::string> va_url("u","url","The url that rviz can use to access the webserver on this machine. This is used to pass the model files to rviz.",false,"","string");
        TCLAP::ValueArg<std::string> va_wd("w","web_dir","The folder on this machine corresponding to the --url flag. The model files will be written here.",false,"","string");
        TCLAP::SwitchArg viz("v","viz","Run the marker publisher to visualize the dart state in rviz.", false);
        TCLAP::ValueArg<std::string> mt("m","marker_topic","The topic the marker publisher will publish the model markers to.", false, "dart_markers", "string");
        cmd.add(dt);
        cmd.add(rt);
        cmd.add(ct);
        cmd.add(pt);
        cmd.add(va_url);
        cmd.add(va_wd);
        cmd.add(viz);
        cmd.add(mt);
        cmd.parse(argc, argv);
        depth_topic = dt.getValue();
        rgb_topic = rt.getValue();
        cmd_topic = ct.getValue();
        out_topic = pt.getValue();
        url = va_url.getValue();
        web_dir = va_wd.getValue();
        use_viz = viz.getValue();
        marker_topic = mt.getValue();
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
    
    ROS_INFO("Starting dart node.");
    ros::init(argc, argv, "dart");
    
    ros::NodeHandle* rosNode = new ros::NodeHandle();
    ros::Subscriber* sub = new ros::Subscriber(rosNode->subscribe("/robot/joint_states", 1, &setJointState));
    pthread_mutex_init(&joint_state_lock, NULL);
    
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
    
    //tracker.addModel("models/spaceJustin/spaceJustinHandRight.xml");
    //tracker.addModel("models/ikeaMug/ikeaMug.xml");
    tracker.addModel("models/baxter/baxter_rosmesh_closedgripper.xml");    
                     
    
    dart::Pose pose = tracker.getPose(0);
    for (int i=0; i<pose.getArticulatedDimensions(); ++i) 
        pose.getArticulation()[i] = (tracker.getModel(0).getJointMin(i) + tracker.getModel(0).getJointMax(i))/2.0;

    pose.setTransformCameraToModel(dart::SE3FromTranslation(0,0,0));
    tracker.getModel(0).setPose(pose);

    ros::Rate r(30);
    
    printf("Running...\n");
    while(ros::ok())
    {
        pose = tracker.getPose(0);
        setModelJoints(&pose);
        tracker.getModel(0).setPose(pose);
        if(use_viz)
            mp->update(tracker, depthSource->getHeader());
        r.sleep();
    }
    
    depthSource->stopRosSpinner();
    ros::shutdown();
    
    if(mp != NULL)
        delete mp;
        
    sub->shutdown();
    delete rosNode;
    delete sub;
    pthread_mutex_destroy(&joint_state_lock);
        
    return 0;
}
    
    






























