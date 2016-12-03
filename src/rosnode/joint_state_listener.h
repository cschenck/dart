#ifndef __JOINT_STATE_LISTENER_H__
#define __JOINT_STATE_LISTENER_H__

#include <map>
#include <pthread.h>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include "pose/pose.h"

using namespace std;

class JointStateListener
{
public:
    JointStateListener(string joint_state_topic);
    ~JointStateListener();
    void setJointState(const sensor_msgs::JointState& msg);
    void setModelJoints(dart::Pose* pose);

private:
    ros::NodeHandle* _ros_node;
    ros::Subscriber* _sub;
    map<string, float> _joint_state;
    pthread_mutex_t _joint_state_lock;
};




#endif
