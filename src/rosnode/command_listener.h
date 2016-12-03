#ifndef __COMMAND_LISTENER_H__
#define __COMMAND_LISTENER_H__

#include <pthread.h>
#include <queue>
#include <string>

#include <ros/ros.h>
#include <std_msgs/ByteMultiArray.h>

using namespace std;

class CommandListener
{
public:
    CommandListener(string cmd_topic);
    ~CommandListener();
    void addCommand(const std_msgs::ByteMultiArrayConstPtr& msg);
    void processCommands();

private:
    ros::NodeHandle* _ros_node;
    ros::Subscriber* _sub;
    pthread_mutex_t _cmd_lock;
    queue<std_msgs::ByteMultiArrayConstPtr> _cmds;
};





#endif
