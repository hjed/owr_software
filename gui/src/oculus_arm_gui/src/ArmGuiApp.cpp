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
#include <../../opt/ros/indigo/include/rviz/visualization_manager.h>
#include <QApplication>
#include <QVBoxLayout>

int main(int argc, char ** argv) {
    ros::init(argc, argv, "arm_gui");
    QApplication app(argc, argv);
    ArmGuiApp * armGui = new ArmGuiApp();
    armGui->show();
    app.exec();

    delete armGui;
}


/**
 * Constructor for ArmGuiApp
 * Initialises ROS, and the OVR SDK
 */
ArmGuiApp::ArmGuiApp(QWidget* parent)  : QWidget( parent ) {  
    //enable head tracking
    //second parameter is capabilities we want, third is those we must have
    //ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);
    
    // Construct and lay out render panel.
    renderPanel = new rviz::RenderPanel();
    QVBoxLayout* mainLayout = new QVBoxLayout();
    mainLayout->addWidget(renderPanel);
    
    setLayout(mainLayout);
    //renderPanel->setParent(  parent);
    //renderPanel->setWindowFlags( Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMaximizeButtonHint );
    
    //this is the main rviz class
    //this code attaches it to the renderPanel we just setup
    manager = new rviz::VisualizationManager( renderPanel);
    renderPanel->initialize( manager->getSceneManager(), manager);
    manager->initialize();
    manager->startUpdate();

    // Create an rviz Grid display.
    grid = manager->createDisplay( "rviz/Grid", "adjustable grid", true );
    
    ROS_ASSERT( grid != NULL );

    // Configure the GridDisplay the way we like it.
    grid->subProp( "Line Style" )->setValue( "Billboards" );
    grid->subProp( "Color" )->setValue( Qt::yellow );
    
    display = new OculusDisplay(renderPanel,manager, this);
    manager->addDisplay(display, true);

    
    

}

/**
 * Deconstructor for ArmGuiApp
 * Frees ROS, and the OVR SDK
 */
ArmGuiApp::~ArmGuiApp() {
    ovrHmd_Destroy(hmd);
    ovr_Shutdown();
    delete manager;
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
        //ros::spinonce();
    }
    
}


