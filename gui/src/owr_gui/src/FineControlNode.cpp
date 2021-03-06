/*
	FineControl Node
	Handles updates to the FineControl GUI
	By Harry J.E Day for Bluesat OWR
	Date: 31/05/2014
	
	
	Updated 30/5/15 by Simon Ireland to detect and pass to gui the active/inactive/offline cameras
*/

#include "FineControlNode.h"
#include "FineControlGUI.h"
#include "ListNode.h"
#include <fstream>
// Include for the image_trasport pkg which will allow us to use compressed
// images through magic ros stuff :)
#include <image_transport/image_transport.h>

FineControlNode::FineControlNode(FineControlGUI *newgui) {
	ROS_INFO("Starting FineControl Node");
	gui = newgui;
	//a nodehandler is used to communiate with the rest of ros
	ros::NodeHandle n("~");
    image_transport::ImageTransport imgTrans(n);
	
	//Initialise the feeds array
	for(int i = 0; i < TOTAL_FEEDS; i++)
		feeds[i] = FEED_OFFLINE;
	
	voltage = 0;
	memset(&armState, 0, sizeof(armState));
	pH = humidity = 0;
	memset(&currentPos, 0, sizeof(currentPos));
	heading = 0;
	tiltX = 0;
	tiltY = 0;
	ultrasonic = 0;
	
	// 
	// Subscribe to all relevant topics for information used by the gui
	// pass the function that is called when a message is received into the subscribe function
	// 
	
	gpsSub = n.subscribe("/gps/fix", 1000, &FineControlNode::receiveGpsMsg, this); // GPS related data
	feedsSub = n.subscribe("/owr/control/availableFeeds", 1000, &FineControlNode::receiveFeedsStatus, this);
	
	// Subscribe to all topics that will be published to by cameras, if the topic hasnt been
	// created yet, will wait til it has w/o doing anything
	videoSub[0] = imgTrans.subscribe("/cam0", 1, &FineControlNode::receiveVideoMsg0, this, image_transport::TransportHints("compressed"));
	videoSub[1] = imgTrans.subscribe("/cam1", 1, &FineControlNode::receiveVideoMsg1, this, image_transport::TransportHints("compressed"));
	videoSub[2] = imgTrans.subscribe("/cam2", 1, &FineControlNode::receiveVideoMsg2, this, image_transport::TransportHints("compressed"));
	videoSub[3] = imgTrans.subscribe("/cam3", 1, &FineControlNode::receiveVideoMsg3, this, image_transport::TransportHints("compressed"));
	//videoSub[2] = n.subscribe("/cam2", 1000, &FineControlNode::receiveVideoMsg2, this);
	//videoSub[3] = n.subscribe("/cam3", 1000, &FineControlNode::receiveVideoMsg3, this);
}

// Spin to wait until a message is received
void FineControlNode::spin() {
	ros::spin();
}

// Called when a message from availableFeeds topic appears, updates with a list of connected cameras:
// FEED_OFFLINE means camera not conencted
// FEED_ACTIVE means its currently streaming
// FEED_INACTIVE means connected but not streaming.
// See gui/src/owr_messages/msg/activeCameras.msg and stream.msg to understand the input message
//
// Simon Ireland: 30/5/15

void FineControlNode::receiveFeedsStatus(const owr_messages::activeCameras::ConstPtr &msg) {
	assert(msg);
	
	//ROS_INFO("finding active feeds");
	
	// Reset feeds array
	for(int i = 0; i < TOTAL_FEEDS; i++)
		feeds[i] = FEED_OFFLINE;
	
	// There isn't guaranteed to be 4 streams in msg, so only update the ones that do appear
	for(int i = 0; i < msg->num; i++) {
		// Get the actual camera number from msg
		int feed = msg->cameras[i].stream;
		
		// If on, then it is streaming, otherwise its only connected 
		if(msg->cameras[i].on)
			feeds[feed] = FEED_ACTIVE;
		else
			feeds[feed] = FEED_INACTIVE;
	}
	
	// Update the gui
	gui->updateFeedsStatus(feeds, msg->num);
}

void FineControlNode::receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received a message");
	//ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
	
	vector3D l;
	l.lat = msg->latitude;
	l.lon = msg->longitude;
	l.alt = msg->altitude;
	gui->updateInfo(voltage, ultrasonic, pH, humidity, NULL, heading, tiltX, tiltY, &l);
}

void FineControlNode::receiveVideoMsg0(const sensor_msgs::Image::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received video frame");
	
	gui->updateVideo((unsigned char *)msg->data.data(), msg->width, msg->height, 0);
}

void FineControlNode::receiveVideoMsg1(const sensor_msgs::Image::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received video frame");
	
	gui->updateVideo((unsigned char *)msg->data.data(), msg->width, msg->height, 1);
}

void FineControlNode::receiveVideoMsg2(const sensor_msgs::Image::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received video frame");
	
	gui->updateVideo((unsigned char *)msg->data.data(), msg->width, msg->height, 2);
}

void FineControlNode::receiveVideoMsg3(const sensor_msgs::Image::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received video frame");
	
	gui->updateVideo((unsigned char *)msg->data.data(), msg->width, msg->height, 3);
}
