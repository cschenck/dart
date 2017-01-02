
#include <stdio.h>
#include <pthread.h>
#include <string.h>
#include <math.h>

#include <cuda_runtime.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

#include "depth_sources/depth_source.h"

#include "ROSDepthSource.h"



ROSDepthSource::ROSDepthSource() : DepthSource<ushort,uchar3>()
{

    rosNode = new ros::NodeHandle();
    sub = NULL;
    colorSub = NULL;
    
    
    depth_data_volatile = NULL;
    depth_data_stable = NULL;
    depth_data_device = NULL;
    
    color_data_volatile = NULL;
    color_data_stable = NULL;
    
    
    depth_width = -1;
    depth_height = -1;
    
    color_width = -1;
    color_height = -1;
    
    runningRosSpinner = false;
    pthread_mutex_init(&depth_data_lock, NULL);
    pthread_mutex_init(&color_data_lock, NULL);
    
    _frame = 0;
    _hasColor = true;

}

ROSDepthSource::~ROSDepthSource()
{
    

    ROS_INFO("Shutting down node listener");
    if(sub != NULL)
        unsubscribeListener();    
    
    
    delete rosNode;

    if(depth_data_volatile != NULL)
        delete depth_data_volatile;
    if(depth_data_stable != NULL)
        delete depth_data_stable;
    if(depth_data_device != NULL)
        cudaFree(depth_data_device);
    
    if(color_data_volatile != NULL)
        delete color_data_volatile;
    if(color_data_stable != NULL)
        delete color_data_stable;

    runningRosSpinner = false;
    pthread_mutex_destroy(&depth_data_lock);
    pthread_mutex_destroy(&color_data_lock);

}

bool ROSDepthSource::initialize(const string& depth_topic, const string& rgb_topic, bool sync, const string& sync_topic) 
{
    sub = new ros::Subscriber(rosNode->subscribe(depth_topic, 1, &ROSDepthSource::setDepthData, this));
    colorSub = new ros::Subscriber(rosNode->subscribe(rgb_topic, 1, &ROSDepthSource::setColorData, this));
    _depth_topic = depth_topic;
    _new_data = false;
    
    _sync = sync;
    if(_sync)
    {
        _sync_topic = sync_topic;
        _sync_pub = new ros::Publisher(rosNode->advertise<std_msgs::String>(_sync_topic, 1));
    }

    //get the first depth image before adding this to the tracker
    ros::Rate rate(100);
    printf("Waiting on first depth and color frames...\n");
    while(depth_data_volatile == NULL || color_data_volatile == NULL)
    {
        if(_sync)
            sendSyncCommand();
        ros::spinOnce();
        rate.sleep();
    }
    printf("Received first frames.\n");

    advance(); //make sure that the pointers for the tracker have data
}

void ROSDepthSource::sendSyncCommand()
{
    std_msgs::String msg;
    msg.data = _depth_topic + " " + to_string(_frame);
    _sync_pub->publish(msg);
}


void ROSDepthSource::unsubscribeListener()
{
    sub->shutdown();
    delete sub;
    sub = NULL;
    
    colorSub->shutdown();
    delete colorSub;
    colorSub = NULL;
    
    if(_sync)
    {
        _sync_pub->shutdown();
        delete _sync_pub;
        _sync_pub = NULL;
    }
}



void ROSDepthSource::setDepthData(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("I heard: [%d:%dx%d:%s]", msg->header.seq, msg->height, msg->width, msg->encoding.c_str());
    
    pthread_mutex_lock(&depth_data_lock);
    if(depth_data_volatile == NULL || depth_width != msg->width || depth_height != msg->height)
    {
        if(depth_data_volatile != NULL)
            delete depth_data_volatile;
        depth_height = _depthHeight = msg->height;
        depth_width = _depthWidth = msg->width;
        depth_data_volatile = new ushort[depth_height*depth_width];
        
        //TODO ask Tanner how to set these correctly
        _focalLength = make_float2(535.*_depthWidth/640,535.*_depthWidth/640);
        _principalPoint = make_float2(_depthWidth/2,_depthHeight/2);
        
    }
    
    _header_volatile = msg->header;
    
    memcpy(depth_data_volatile, msg->data.data(), msg->height*msg->width*sizeof(ushort));
    _new_data = true;
    pthread_mutex_unlock(&depth_data_lock);
    
}

void ROSDepthSource::setColorData(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("I heard: [%d:%dx%d:%s]", msg->header.seq, msg->height, msg->width, msg->encoding.c_str());
    
    pthread_mutex_lock(&color_data_lock);
    if(color_data_volatile == NULL || color_width != msg->width || color_height != msg->height)
    {
        if(color_data_volatile != NULL)
            delete color_data_volatile;
        color_height = _colorHeight = msg->height;
        color_width = _colorWidth = msg->width;
        color_data_volatile = new uchar3[color_height*color_width];
    }
    
    memcpy(color_data_volatile, msg->data.data(), msg->height*msg->width*sizeof(uchar3));
    pthread_mutex_unlock(&color_data_lock);
    
}

void* rosSpinner(void* ptr)
{
    ROSDepthSource* depthSource = (ROSDepthSource*)ptr;
    ros::Rate rate(100);
    while(depthSource->isRosSpinning())
    {
        ros::spinOnce();
        rate.sleep();
    }
    pthread_exit(NULL);
}

void ROSDepthSource::startRosSpinner()
{
    runningRosSpinner = true;
    _isLive = true;
    pthread_create(&rosSpinner_t, NULL, rosSpinner, (void*)this);
}

void ROSDepthSource::stopRosSpinner()
{
    runningRosSpinner = false;
    _isLive = false;
    pthread_join(rosSpinner_t, NULL);
}

void ROSDepthSource::swapDepthPointers()
{
    pthread_mutex_lock(&depth_data_lock);
    ushort* tmp = depth_data_stable;
    depth_data_stable = depth_data_volatile;
    depth_data_volatile = tmp;
    _header_stable = _header_volatile;
    _new_data = false;
    pthread_mutex_unlock(&depth_data_lock);
}

void ROSDepthSource::swapColorPointers()
{
    pthread_mutex_lock(&color_data_lock);
    uchar3* tmp = color_data_stable;
    color_data_stable = color_data_volatile;
    color_data_volatile = tmp;
    pthread_mutex_unlock(&color_data_lock);
}

const ushort *ROSDepthSource::getDepth() const 
{
    //ROS_INFO("ROSDepthSource::getDepth: called");
    return depth_data_stable;
}

const ushort *ROSDepthSource::getDeviceDepth() const 
{
    //ROS_INFO("ROSDepthSource::getDeviceDepth: called");
    return depth_data_device;
}

const uchar3 *ROSDepthSource::getColor() const 
{
    return color_data_stable;
}

void ROSDepthSource::setFrame(const uint frame) {

    fprintf(stderr, "ROSDepthSource:setFrame: this method is not implemented.\n");
    throw -1;

}

void ROSDepthSource::advance() 
{
    if(_sync)
    {
        _new_data = false;
        sendSyncCommand();
        ros::spinOnce();
        ros::Rate rate(100);
        while(!_new_data)
        {
            ros::spinOnce();
            rate.sleep();
            sendSyncCommand();
        }
    }
    
    //make sure we have the most up to date data
    swapColorPointers();
    swapDepthPointers(); 
    
    if(depth_data_device == NULL)
        cudaMalloc(&depth_data_device,depth_width*depth_height*sizeof(ushort));
    cudaMemcpy(depth_data_device, depth_data_stable, depth_width*depth_height*sizeof(ushort), cudaMemcpyHostToDevice);
    
    ++_frame;
}














