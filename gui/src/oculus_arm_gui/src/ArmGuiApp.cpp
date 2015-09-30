/* 
 * Date Started: 24/09/15
 * Original Author: Harry J.E Day
 * ROS Node Name: arm_gui
 * ROS Package: oculus_arm_gui
 * Purpose: This is the main class file for oculus arm gui developed for Harry's RSA
 *  project. This module handles the visual interface part. This class handles the
 *  main logic loop.
 */
 
#include "oculus_arm_gui/ArmGuiApp.hpp" 
#include <../../opt/ros/indigo/include/ros/init.h>


int main(int argc, char ** argv) {
    ArmGuiApp * armGui = new ArmGuiApp(argc, argv);
    armGui->run();
    delete armGui;
}


/**
 * Constructor for ArmGuiApp
 * Initialises ROS, and the OVR SDK
 */
ArmGuiApp::ArmGuiApp(int argc, char ** argv) {
    //initialise ros
    ros::init(argc, argv, "arm_gui");
    //initialise the api
    ovr_Initialize();
    //Create the HMD object
    hmd = ovrHmd_Create(0);
    
    //If we can't connect, create a fake one
    if(!hmd) {
        ROS_ERROR("Failed to Create HMD\n Creating a fake one");
        hmd = ovrHmd_CreateDebug(ovrHmdType(ovrHmd_DK2));
    }
    
    //enable head tracking
    //second parameter is capabilities we want, third is those we must have
    ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);
    

}

/**
 * Deconstructor for ArmGuiApp
 * Frees ROS, and the OVR SDK
 */
ArmGuiApp::~ArmGuiApp() {
    ovrHmd_Destroy(hmd);
    ovr_Shutdown();
    ROS_INFO("shutdown complete");
}

/**
 * Main Logic Loop
 */
void ArmGuiApp::run() {
    //start getting callbacks
    ros::AsyncSpinner spinner(ROS_SPINNER_THREADS);
    spinner.start();
    while(ros::ok()) {
        
    }
    
}


