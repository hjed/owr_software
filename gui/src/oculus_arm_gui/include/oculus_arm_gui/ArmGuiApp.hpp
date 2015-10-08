/* 
 * Date Started: 24/09/15
 * Original Author: Harry J.E Day
 * ROS Node Name: arm_gui
 * ROS Package: oculus_arm_gui
 * Purpose: This is the header file for oculus arm gui developed for Harry's RSA
 *  project. This module handles the visual interface part. This class handles the
 *  main logic loop.
 */
 
#ifndef ARM_GUI_APP_H
#define ARM_GUI_APP_H
#include <OVR_CAPI.h>
#include <ros/ros.h>

//QT Dependencies

#include <QWidget>  

//rviz dependencies
#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
//#include <OVR_Kernel.h>

 
//using namespace OVR;
//using namespace OVR::OvrPlatform;
//using namespace OVR::Render;
//using namespace ros;
 
class ArmGuiApp : public QWidget {
    Q_OBJECT //required for rviz
    public:
        //Constructor and Deconstructor
        ArmGuiApp(QWidget* parent = 0);
        ~ArmGuiApp();

    
    
    //private Q_SLOTS:
      //void setThickness( int thickness_percent );
      //void setCellSize( int cell_size_percent );
    
    private:
        ovrHmd hmd;
        rviz::VisualizationManager* manager;
        rviz::RenderPanel* renderPanel;
        rviz::Display* grid;
    
 
};
 
 #endif //ARM_GUI_APP_H
