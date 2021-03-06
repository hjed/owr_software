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

//we assume that our focal point is 1m away from the camera
#define FOCAL_DISTANCE_FROM_CAMERA 1.0

JointManager::JointManager(ros::NodeHandle nh) {
    this->nh = nh;
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
    std::vector<std::string> zAxisRotate;
    zAxisRotate.push_back("neck_pan");
    std::vector<std::string> yAxisRotate;
    yAxisRotate.push_back("neck_tilt");
    yAxisRotate.push_back("arm_elbow");
    //the vertical axis about which the camera should rotate
    std::string rotationAxis;
    while(ros::ok()) {
        tf::StampedTransform transform;
        tf::StampedTransform cameraArmSTransform;
        
        try {
            //load the arm parameters
            nh.param<std::vector<std::string> >("zAxisRotate", zAxisRotate, zAxisRotate);
            nh.param<std::vector<std::string> >("yAxisRotate", yAxisRotate, yAxisRotate);
            nh.param<std::string>("verticalRotation", rotationAxis, "/arm_base");
            //we want to rotate around camera_link and then match that
            listenToOculus.lookupTransform(rotationAxis.c_str(), "/oculus",  ros::Time(0), transform);
            tf::Vector3 orig =  transform.getOrigin();
            tf::Quaternion rotQ =  transform.getRotation();
            //printf("%f, %f, %f\n", orig.x(), orig.y(), orig.z());
            //for oculus: z is down, y is forward back, x is left right
            //rotation: x is up/down, y is rotation about the horizontal axis, z is about the vertical axis
            //dif them
            printf("%f, %f, %f\n", rotQ.getX(), rotQ.getY(), rotQ.getZ());
            sensor_msgs::JointState move;

            //we don't need to change anything here, because there rotation is the same
//             adjustJoint(&move, "neck_pan", orig.z(), 0.0, rotQ.getZ(),0);
//             adjustJoint(&move, "neck_tilt", orig.x(), 0.0, rotQ.getX()*-1,0);
//             adjustJoint(&move, "arm_elbow", orig.x(), 0.0, rotQ.getX()*-1,0);
            for(int i = 0; i < zAxisRotate.size(); i++) {
                move.name.push_back(zAxisRotate[i].c_str());
                move.position.push_back(rotQ.getZ()*1);
            }
            

//             listenToOculus.lookupTransform("/arm_shoulder", "/camera",  ros::Time(0), cameraArmSTransform);
            for(int i = 0; i < yAxisRotate.size(); i++) {
                move.name.push_back(yAxisRotate[i].c_str());
                move.position.push_back(rotQ.getY()*1);
            }
            jointPub.publish(move);
        } catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
        //ros::spinonce();
    }
}

void JointManager::adjustJoint ( sensor_msgs::JointState  *msg, std::string jointName, float axisCordOculus, float axisCordCamera, float oculusAngle, float cameraAngle ) {
    float distance1D = fabs(axisCordOculus - axisCordCamera);
    /* we calculate the extirior angle of the triangle between the camera, the oculus view point, and  a point 1m away from the camera to find the correct angle
     * we have two sides (distances) and one useful angle opposite side d
     * By the sin rule we can calculate the angle sum of the triangle
     * Then the extirior angle
     */
    float angleGama = asin(distance1D*sin(oculusAngle));
    //float angle = angleGama + 
    //msg->name.push_back(jointName);
    //msg->position.push_back(oculusAngle);
}


