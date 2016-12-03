


#include "joint_state_listener.h"



JointStateListener::JointStateListener(string joint_state_topic)
{
    _ros_node = new ros::NodeHandle();
    pthread_mutex_init(&_joint_state_lock, NULL);
    _sub = new ros::Subscriber(_ros_node->subscribe(joint_state_topic, 1, &JointStateListener::setJointState, this));
}

JointStateListener::~JointStateListener()
{
    _sub->shutdown();
    delete _ros_node;
    delete _sub;
    pthread_mutex_destroy(&_joint_state_lock);
}



void JointStateListener::setJointState(const sensor_msgs::JointState& msg)
{
    pthread_mutex_lock(&_joint_state_lock);
    for(int i = 0; i < msg.name.size(); ++i)
    {
        _joint_state[msg.name[i]] = msg.position[i];
    }    
    pthread_mutex_unlock(&_joint_state_lock);
}

void JointStateListener::setModelJoints(dart::Pose* pose)
{    
    pthread_mutex_lock(&_joint_state_lock);
    map<string, float> js = _joint_state;
    pthread_mutex_unlock(&_joint_state_lock);
    for(int i = 0; i < pose->getArticulatedDimensions(); ++i)
    {
        string name = pose->getReducedName(i);
        if(js.find(name) != js.end())
        {
            pose->getArticulation()[i] = js[name];
        }
    }
    
}
