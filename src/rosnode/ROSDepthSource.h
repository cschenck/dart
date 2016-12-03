#ifndef ROS_DEPTH_SOURCE_H
#define ROS_DEPTH_SOURCE_H

#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include "depth_sources/depth_source.h"

#define MAX_DIST_VALUE 65535 //2^16 - 1

using namespace std;

class ROSDepthSource : public dart::DepthSource<ushort,uchar3> {
public:

    ROSDepthSource();

    ~ROSDepthSource();

    bool initialize(const string& depth_topic, const string& rgb_topic);

    const ushort *getDepth() const;

    const ushort *getDeviceDepth() const;

    const uchar3 *getColor() const;
    
    std_msgs::Header getHeader() const { return _header_stable; }

    dart::ColorLayout getColorLayout() const { return dart::LAYOUT_BGR; }
    
    void setFrame(const uint frame);

    void advance();

    bool hasRadialDistortionParams() const { return false; }
    
    void setDepthData(const sensor_msgs::ImageConstPtr& msg);
    
    void setColorData(const sensor_msgs::ImageConstPtr& msg);

    inline float getScaleToMeters() const { return 1/(1000.0f); }
    
    void startRosSpinner();
    
    void stopRosSpinner();
    
    bool isRosSpinning() const { return runningRosSpinner; }
    
    void unsubscribeListener();
    
   
    

private:

    ROSDepthSource(const ROSDepthSource& that) { throw -1; }

    ROSDepthSource& operator=(const ROSDepthSource& that){ throw -1; }
    
    void swapDepthPointers();
    
    void swapColorPointers();
    

    ros::NodeHandle* rosNode;
    ros::Subscriber* sub;
    ros::Subscriber* colorSub;
    
    ushort* depth_data_volatile;
    ushort* depth_data_stable;
    ushort* depth_data_device;
    std_msgs::Header _header_stable;
    std_msgs::Header _header_volatile;
    
    int depth_width;
    int depth_height;
    
    int color_width;
    int color_height;
    
    bool runningRosSpinner;
    pthread_t rosSpinner_t;
    pthread_mutex_t depth_data_lock;
    
    uchar3* color_data_volatile;
    uchar3* color_data_stable;
    pthread_mutex_t color_data_lock;
    
    
};

#endif














