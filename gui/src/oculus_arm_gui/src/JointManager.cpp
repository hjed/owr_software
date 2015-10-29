/*
 * program rather than as an rviz plugin.
 * File Created by: Harry J.E Day
 * Date Created: 28/10/15
 * ROS Node Name: arm_tf (note: this is not a main class)
 * ROS Package: oculus_arm_gui
 * Purpose: This class manages the interaction with ros
 */
#include <oculus_arm_gui/JointManager.hpp>
#define JOINTS_IN_TOPIC "/joint_states"
#define JOINTS_OUT_TOPIC "/joint_control"



JointManager::JointManager(ros::NodeHandle nh) {
    jointSub = nh.subscribe(JOINTS_IN_TOPIC, 2,  &JointManager::jointCallback, this);
    jointPub = nh.advertise<sensor_msgs::JointState>(JOINTS_OUT_TOPIC, 2, true);
    // spin async
    ros::AsyncSpinner *spinner = new ros::AsyncSpinner(ROS_SPINNER_THREADS);
    spinner->start();
}

JointManager::~JointManager() {
  
}

JointPosition JointManager::getJoint(std::string jointName) {
    if (jointMap.count(jointName)) {
        return jointMap[jointName];
    }
    return NULL;
}

bool JointManager::putJoint(JointPosition joint) {
    // TODO: sends a ros message to move the joint

}

void JointManager::jointCallback(const sensor_msgs::JointState::ConstPtr& msg){

    for (int i=0; i < msg->name.size(); i++) {
        /*JointPosition tempJoint = (JointPosition) malloc(sizeof(jointPosition));
        tempJoint->pos.x = msg->position[i][0];
        tempJoint->pos.y = msg->position[i][1];
        tempJoint->pos.z = msg->position[i][2];
        tempJoint->rot.x = msg->velocity[i][0];
        tempJoint->rot.y = msg->velocity[i][1];
        tempJoint->rot.z = msg->velocity[i][2];

        jointMap[msg->name[i]] = tempJoint;
        ROS_INFO("joint %f,%f,%f %f,%f,%f",tempJoint->pos.x,tempJoint->pos.y,tempJoint->pos.z,tempJoint->rot.x,tempJoint->rot.y,tempJoint->rot.z);*/
    }
}


void JointManager::logicLoop() {
	tf::Vector3 lastOrig;
	while(ros::ok()) {
		tf::StampedTransform transform;
		tf::StampedTransform cameraArmSTransform;

		try {
			//we want to rotate around camera_link and then match that
			listenToOculus.lookupTransform("/base_link", "/oculus",  ros::Time(0), transform);
			tf::Vector3 orig =  transform.getOrigin();
			tf::Quaternion rotQ =  transform.getRotation();
			//printf("%f, %f, %f\n", orig.x(), orig.y(), orig.z());
			//for oculus: z is down, y is forward back, x is left right
			//rotation: x is up/down, y is rotation about the horizontal axis, z is about the vertical axis
			//dif them
			printf("%f, %f, %f\n", rotQ.getX(), rotQ.getY(), rotQ.getZ());
			sensor_msgs::JointState move;

			//we don't need to change anything here, because there rotation is the same
			move.name.push_back("neck_pan");
			move.position.push_back(rotQ.getZ()*1);

			//listenToOculus.lookupTransform("/arm_shoulder", "/camera",  ros::Time(0), cameraArmSTransform);
			move.name.push_back("neck_tilt");
			move.position.push_back(rotQ.getX()*-1);

			move.name.push_back("arm_elbow");
			move.position.push_back(rotQ.getX()*-1);
			jointPub.publish(move);
		} catch (tf::TransformException ex){
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}
		//ros::spinonce();
	}
}

