


#include "command_listener.h"




CommandListener::CommandListener(string cmd_topic)
{
    _ros_node = new ros::NodeHandle();
    _sub = new ros::Subscriber(_ros_node->subscribe(cmd_topic, 1, &CommandListener::addCommand, this));
}

CommandListener::~CommandListener()
{
    _sub->shutdown();
    delete _ros_node;
    delete _sub;
}

void CommandListener::addCommand(const std_msgs::ByteMultiArrayConstPtr& msg)
{
    pthread_mutex_lock(&_cmd_lock);
    _cmds.push(msg);
    pthread_mutex_unlock(&_cmd_lock);
}

void CommandListener::processCommands()
{
    while(!_cmds.empty()) // This is okay because if something is missed it'll get processed next time.
    {
        pthread_mutex_lock(&_cmd_lock);
        const std_msgs::ByteMultiArrayConstPtr cmd = _cmds.front();
        _cmds.pop();
        pthread_mutex_unlock(&_cmd_lock);
        
        // Find Table
        
        // Add object
        
        // Add object on table
    }
}
