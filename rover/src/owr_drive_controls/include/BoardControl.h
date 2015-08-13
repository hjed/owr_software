/*
 * Converts joystick comands to velocity, etc
 * Author: Harry J.E Day for BlueSat OWR
 * Start: 8/07/15
 */
#ifndef BOARD_CONTROL_H
#define BOARD_CONTROL_H
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <stdio.h>
#include "buttons.h"
#include "Bluetongue.h"

using namespace std; 


//serial IO
#define TTY "/dev/ttyACM0"

class BoardControl {
    public:
        BoardControl();
        void run();
        
    private:
        void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void armCallback(const sensor_msgs::Joy::ConstPtr& joy);
        void switchFeed(int * storedState, int joyState, int feedNum);
        void publishGPS(GPSData gps);
        //void sendMessage(float lf, float lm, float lb, float rf, float rm, float rb);
        ros::NodeHandle nh;
        ros::Publisher  velPublisher;
        ros::Publisher gpsPublisher;
        ros::Subscriber joySubscriber;
        ros::Subscriber armSubscriber;
        float leftDrive, rightDrive;
        //arm top, bottom
        int armTop, armBottom;
        float armRotate;
        int armIncRate;
        //gps sequence number
        int gpsSequenceNum;
        
        //to keep track of button states. It is possible press could change it
        int cam0Button, cam1Button, cam2Button, cam3Button;
        
        //serial i0
        FILE * fd;
};

#endif
