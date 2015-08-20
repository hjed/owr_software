/*
 * Main class for pbuff relays
 * Author: Harry J.E Day for Bluesat OWR
 * Date: 14/12/14
 */
 

//#include "bluesat_owr_protobuf/Message1Relay.h"
#include "MagnetConverter.h" 
#include <iostream>
#include <list>
#include <cmath>
#include <stdio.h>

#define TOPIC "/owr/position"
//minum number of lat/long inputs to calculate the heading
#define MIN_H_CALC_BUFFER_SIZE 2 
 
int main(int argc, char ** argv) {
    
    
    //init ros
    ros::init(argc, argv, "owr_position_node");
    
    MagnetConverter p(TOPIC);
    p.spin();
    
    return EXIT_SUCCESS;   
}

MagnetConverter::MagnetConverter(const std::string topic) {
    subscriber = node.subscrib("/owr/sensors/mag", 5, &MagnetConverter::receiveMsg, this); // mangnet stuff
    publisher =  nh.advertise<sensor_msgs::Imu>("/owr/nav/mag_imu", 10);
}

void MagnetConverter::receiveMsg(const boost::shared_ptr<geometry_msgs::Vector3 const> & msg) {
    //Normalized vector
    float norm = sqrt(pow(msg->x,2) + pow(msg->y,2) + pow(msg->z,2))
    float norm_x = msg->x/norm;
    float norm_y = msg->y/norm;
    float norm_z = msg->z/norm;

    sensor_msgs::Imu imu;
    //We need orientation set
    //TODO: set valuews
    publisher.publish(imu);
}



//main loop
void MagnetConverter::spin() {
    while(ros::ok()) {
        ros::spinOnce();
    }
}