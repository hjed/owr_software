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

#include <ros/ros.h>
#include <string.h>
#include <OGRE/OgreVector3.h>

//used to store the joint position
typedef struct _jointPosition {
    //the urdf joint name
    std::string jointName;
    //the position of the joint relative to base link
    Ogre::Vector3 pos;
    //the rotation of the joint relative to its origin
    Ogre::Vector3 rot;
    
} *JointPositon;

class JointManager {
    
    public:
        JointManager(ros::NodeHandle nh);
        ~JointManager();
        
        JointPositon getJoint(std::string jointName);
        bool putJoint(JointPositon joint);
        
    private:
        //stores the position of all the joints
        std::map <std::string, JointPositon> jointMap;
    
}


#endif