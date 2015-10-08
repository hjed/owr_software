/* 
 * Date Started: 24/09/15
 * Original Author: Harry J.E Day
 * ROS Node Name: arm_gui
 * ROS Package: oculus_arm_gui
 * Purpose: This is the main class file for oculus arm gui developed for Harry's RSA
 *  project. This module handles the visual interface part. This class handles the
 *  main logic loop.
 */
#include <QApplication>
#include <QGridLayout>
#include <QVBoxLayout>
#include "oculus_arm_gui/ArmGuiApp.hpp" 


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
ArmGuiApp::ArmGuiApp(QWidget* parent)  : QWidget( parent ){
    //initialise the api
    ovr_Initialize();
    //Create the HMD object
    hmd = ovrHmd_Create(0);
    
    //If we can't connect, create a fake one
    if(!hmd) {
        ROS_ERROR("Failed to Create HMD\n Creating a fake one");
        hmd = ovrHmd_CreateDebug(ovrHmdType(ovrHmd_DK2));
    }
    
    
    // Construct and lay out render panel.
    renderPanel = new rviz::RenderPanel();
    QVBoxLayout* mainLayout = new QVBoxLayout();
    mainLayout->addWidget( renderPanel);
    
    setLayout(mainLayout);
    
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


}

/**
 * Deconstructor for ArmGuiApp
 * Frees ROS, and the OVR SDK
 */
ArmGuiApp::~ArmGuiApp() {
    ovrHmd_Destroy(hmd);
    ovr_Shutdown();
    delete manager;
}
