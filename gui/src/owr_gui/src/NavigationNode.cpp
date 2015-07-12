/*
	Navigation Node
	Handles updates to the Navigation GUI
	By Harry J.E Day for Bluesat OWR
	Date: 31/05/2014
	
	
	Updated 30/5/15 by Simon Ireland to detect and pass to gui the active/inactive/offline cameras
*/

#include "NavigationNode.h"
#include <fstream>

NavigationNode::NavigationNode(NavigationGUI *newgui) {
	ROS_INFO("Starting Navigation Node");
	gui = newgui;
	//a nodehandler is used to communiate with the rest of ros
	ros::NodeHandle n("~");
	
	//Initialise all the information to be used in by the gui
	battery = 5;
	signal = 5;
	tiltX = 30;
	tiltY = 30;
	ultrasonic = 0;
	
	//Initialise the feeds array
	for(int i = 0; i < TOTAL_FEEDS; i++)
		feeds[i] = FEED_OFFLINE;
	
	// 
	// Subscribe to all relevant topics for information used by the gui
	// pass the function that is called when a message is received into the subscribe function
	// 
	
	gpsSub = n.subscribe("/gps/fix", 1000, &NavigationNode::receiveGpsMsg, this); // GPS related data
	batterySub = n.subscribe("/status/battery", 1000, &NavigationNode::receiveBatteryMsg, this); // Power left on the battery
	feedsSub = n.subscribe("/owr/control/availableFeeds", 1000, &NavigationNode::receiveFeedsStatus, this);
	
	// Subscribe to all topics that will be published to by cameras, if the topic hasnt been
	// createed yet, will wait til it has w/o doing anything
	
	videoSub[0] = n.subscribe("/cam0", 1000, &NavigationNode::receiveVideoMsg, this);
	videoSub[1] = n.subscribe("/cam1", 1000, &NavigationNode::receiveVideoMsg, this);
	videoSub[2] = n.subscribe("/cam2", 1000, &NavigationNode::receiveVideoMsg, this);
	videoSub[3] = n.subscribe("/cam3", 1000, &NavigationNode::receiveVideoMsg, this); // Frames of video from camera
	
}

// Spin to wait until a message is received
void NavigationNode::spin() {
	ros::spin();
}

// Called when a message from availableFeeds topic appears, updates with a list of connected cameras:
// FEED_OFFLINE means camera not conencted
// FEED_ACTIVE means its currently streaming
// FEED_INACTIVE means connected but not streaming.
// See gui/src/owr_messages/msg/activeCameras.msg and stream.msg to understand the input message
//
// Simon Ireland: 30/5/15

void NavigationNode::receiveFeedsStatus(const owr_messages::activeCameras::ConstPtr &msg) {
	assert(msg);
	
	//ROS_INFO("finding active feeds");
	
	// Reset feeds array
	for(int i = 0; i < TOTAL_FEEDS; i++)
		feeds[i] = FEED_OFFLINE;
	
	// There isnt gaurenteed to be 4 streams in msg, so only update the ones that do appear
	for(int i = 0; i < msg->num; i++) {
		// Get the actual camera number from msg
		int feed = msg->cameras[i].stream;
		
		// If on, then it is streaming, oterwise its only connected 
		if(msg->cameras[i].on)
			feeds[feed] = FEED_ACTIVE;
		else
			feeds[feed] = FEED_INACTIVE;
	}
	
	// Update the gui
	gui->updateFeedsStatus(feeds, msg->num);
}

void NavigationNode::receiveGpsMsg(const sensor_msgs::NavSatFix::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received a message");
	//ROS_INFO("long %lf, lat %lf, alt %lf", msg->longitude, msg->latitude, msg->altitude);
		
	//create a new node
	ListNode l = (ListNode)malloc(sizeof(vector3D));
	l->lat = msg->latitude;
	l->lon = msg->longitude;
	l->alt = msg->altitude;
	gui->updateInfo(battery, signal, ultrasonic, l, target);
}


void NavigationNode::receiveBatteryMsg(const bluesat_owr_protobuf::battery_ros::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received a message");
	//ROS_INFO("voltage %f", msg->voltage);
	battery = msg->voltage;
	
	gui->updateInfo(battery, signal, ultrasonic, NULL, target);
}

void NavigationNode::receiveVideoMsg(const sensor_msgs::Image::ConstPtr& msg) {
	assert(msg);
	
	//ROS_INFO("received video frame");
	
	gui->updateVideo((unsigned char *)msg->data.data(), msg->width, msg->height);
}
