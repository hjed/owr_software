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


int main(int argc, char ** argv) {
    ros::init(argc, argv, "arm_gui");
    /*ROS_INFO("object avoidance starting");
    ObjectAvoidance ObjectAvoidance;
    ROS_INFO("initialising...run");
    ObjectAvoidance.run();*/
}


/**
 * Constructor for ArmGuiApp
 * Initialises ROS, and the OVR SDK
 */
ArmGuiApp::ArmGuiApp() {
    //initialise the api
    ovr_Initialize();
    //Create the HMD object
    hmd = ovrHmd_Create(0);
    
    //If we can't connect, create a fake one
    if(!hmd) {
        ROS_ERROR("Failed to Create HMD\n Creating a fake one");
        hmd = ovrHmd_CreateDebug(ovrHmdType(ovrHmd_DK2));
    }

}

/**
 * Deconstructor for ArmGuiApp
 * Frees ROS, and the OVR SDK
 */
ArmGuiApp::~ArmGuiApp() {
    ovrHmd_Destroy(hmd);
    ovr_Shutdown();

}
