/*
 * Important Note: This is heavily influenced by the oculus_rviz_plugin which can be found at
 * https://github.com/OSUrobotics/oculus_rviz_plugins/blob/groovy-devel/src/oculus_display.cpp
 * However this is designed specifically for this project, and to work as an independent 
 * program rather than as an rviz plugin.
 * File Created by: Harry J.E Day
 * ROS Node Name: arm_gui (note: this is not a main class)
 * ROS Package: oculus_arm_gui
 * Purpose: This class represents a rviz display object, that is used to hijack rviz's 
 *          rendering engine to display it in the right format for the occulus
 */
 
#include "oculus_arm_gui/OculusDisplay.hpp" 

#include <QApplication>
#include <OGRE/OgreRenderWindow.h>
 
OculusDisplay::OculusDisplay(rviz::RenderPanel *renderPanel, QWidget* parent):  sceneNode(0) {
    renderWidget=renderPanel;
    /*std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);*/

    //this detects when the number of screen changes (i.e when an occulus is connected)
    connect( QApplication::desktop(), SIGNAL( screenCountChanged ( int ) ), this, SLOT( onScreenCountChanged(int)) );
}

void OculusDisplay::onInitialize() {
    
    //setup the window so that it works correctly with oculus
    renderWidget->setWindowFlags( Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMaximizeButtonHint );
    Ogre::RenderWindow *window = renderWidget->getRenderWindow();
    window->setVisible(false);
    window->setAutoUpdated(false);
    
    //attach ourselves to the ORGE system so we can get the low level stuff
    window->addListener(this);
    
    sceneNode = sceneManager->getRootSceneNode()->createChildSceneNode();
}


//called by QT when the number of screens changed
void OculusDisplay::onScreenCountChanged( int newCount)  {
  if ( newCount == 1 ) {
    fullscreenProperty->setBool(false);
    fullscreen_property->setHidden(true);
    setStatus( rviz::StatusProperty::Error, "Screen", "No secondary screen detected. Cannot render to Oculus device.");
  }  else  {
    fullscreen_property_->setHidden(false);
    setStatus( rviz::StatusProperty::Ok, "Screen", "Using screen #2.");
  }
}


void update( float wall_dt, float ros_dt ) {};
void reset() {};

void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt ) {};
void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt ) {};

void onEnable() {};
void onDisable() {};

void updateCamera() {};

