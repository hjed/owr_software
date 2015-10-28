/*
 * program rather than as an rviz plugin.
 * File Created by: Harry J.E Day
 * Date Created: 28/10/15
 * ROS Node Name: arm_gui (note: this is not a main class)
 * ROS Package: oculus_arm_gui
 * Purpose: This class manages the interaction with ros
 */
#ifndef JOINT_MANAGER_H
#define JOINT_MANAGER_H
#define ROS_SPINNER_THREADS 2
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string.h>
#include <OGRE/OgreVector3.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>

//used to store the joint position
typedef struct _jointPosition {
    //the urdf joint name
    std::string jointName;
    //the position of the joint relative to base link
    geometry_msgs::Vector3 pos;
    //the rotation of the joint relative to its origin
    geometry_msgs::Vector3 rot;
    
} jointPosition;

typedef jointPosition* JointPosition;

class JointManager {
    
    public:
        JointManager(ros::NodeHandle nh);
        ~JointManager();
        
        JointPosition getJoint(std::string jointName);
        bool putJoint(JointPosition joint);
    
        void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

        void logicLoop();
    protected:
        ros::Subscriber jointSub;
        ros::Publisher jointPub;
    private:
        //stores the position of all the joints
        std::map <std::string, JointPosition> jointMap;
        //listens for the oculus transform
        tf::TransformListener  listenToOculus;


};


#endif
