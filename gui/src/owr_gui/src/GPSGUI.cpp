/*
 * GPSLogger Node
 * Logs GPS input to KML
 * By Harry J.E Day for Bluesat OWR
 * Date: 31/05/2014
 */
#include "GpsGUI.h"
#include <fstream>

GPSGUI::GPSGUI(OwrGui * ogui) {
    ROS_INFO("initlising GPSLogger");
    gui = ogui;
    //a nodehandler is used to communiate with the rest of ros
    ros::NodeHandle n("~");
    
    battery = 5;
    signal = 5;
    tiltX = 30;
    tiltY = 30;
    ultrasonic = 0.0;
    list = NULL;
    end = NULL;
    
    //pass the function that is called when a message is recived
    gpsSub = n.subscribe("/gps/fix", 1000, &GPSGUI::reciveGpsMsg, this);
    //same for battery
    batterySub = n.subscribe("/status/battery", 1000, &GPSGUI::reciveBatteryMsg,this);
}

void GPSGUI::spin() {
  
  ros::spin();

}



void GPSGUI::reciveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    assert(msg);
    
    ROS_INFO("recived a message");
    ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
        
    //create a new node
    ListNode l = (ListNode) malloc(sizeof(vector2D));
    l->y = msg->latitude; 
    l->x = msg->longitude;
    l->next = NULL;
    
    if(end == NULL) {
        list = l;
        end = l;
    } else {
        end->next = l;
        end = l;
    }
    gui->updateConstants(battery,signal,ultrasonic,list,target);
    
}


void GPSGUI::reciveBatteryMsg(const bluesat_owr_protobuf::battery_ros::ConstPtr& msg) {
    assert(msg);
    
    ROS_INFO("recived a message");
    ROS_INFO("voltage %f", msg->voltage);
        
    
    gui->updateConstants(msg->voltage,signal,ultrasonic,list,target);
    
}

