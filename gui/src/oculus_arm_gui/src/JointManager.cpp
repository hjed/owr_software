/*
 * program rather than as an rviz plugin.
 * File Created by: Harry J.E Day
 * Date Created: 28/10/15
 * ROS Node Name: arm_gui (note: this is not a main class)
 * ROS Package: oculus_arm_gui
 * Purpose: This class manages the interaction with ros
 */
#include <oculus_arm_gui/JointManager.hpp>
#define JOINTS_IN_TOPIC "/joint_states"
#define JOINTS_OUT_TOPIC "/joint_states"

JointManager::JointManager(ros::NodeHandle nh) {
    jointSub = nh.subscribe(JOINTS_IN_TOPIC, 2,  &JointManager::jointCallback, this);
    jointPub = nh.advertise<sensor_msgs::JointState>(JOINTS_OUT_TOPIC, 2, true);
    // spin async
    ros::AsyncSpinner spinner(ROS_SPINNER_THREADS);
    spinner.start();
}

JointManager::~JointManager() {
  
}

JointPositon JointManager::getJoint(std::string jointName) {
    if (jointMap.count(jointName)) {
        return jointMap[jointName];
    }
    return NULL;
}
bool JointManager::putJoint(JointPositon joint) {
    // TODO: sends a ros message to move the joint
}

void JointManager::jointCallback(const sensor_msgs::JointState::ConstPtr& msg){
    for (int i=0; i < msg->name.size(); i++) {
        JointPositon tempJoint = malloc(sizeof(struct _tempJoint));
        tempJoint->pos = msg->position;
        tempJoint->rot = msg->velocity;
        jointMap[msg->name[i]] = tempJoint;
    }
}

