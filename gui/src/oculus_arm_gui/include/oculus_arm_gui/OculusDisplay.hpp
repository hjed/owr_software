/*
 * Important Note: This is heavily influenced by the oculus_rviz_plugin which can be found at
 * https://github.com/OSUrobotics/oculus_rviz_plugins/blob/groovy-devel/include/oculus_rviz_plugins/oculus_display.h
 * However this is designed specifically for this project, and to work as an independent 
 * program rather than as an rviz plugin.
 * File Created by: Harry J.E Day
 * ROS Node Name: arm_gui (note: this is not a main class)
 * ROS Package: oculus_arm_gui
 * Purpose: This class represents a rviz display object, that is used to hijack rviz's 
 *          rendering engine to display it in the right format for the occulus
 */
#ifndef OCULUS_DISPLAY_H
#define OCULUS_DISPLAY_H

//Qt dependencies
#include <QObject>
#include <QWidget>

#include <boost/bind.hpp>

//OGRE (framework rviz is based on) dependencies
#include <OGRE/OgreRenderTargetListener.h>

//rviz depenencies
#include "rviz/display.h"
#include "rviz/render_panel.h"

//our depenecies
#include "oculus_arm_gui/ArmGuiApp.hpp" 

//This class extends rviz::Display to overide rviz functionality, and renderTargetListener
//to get the information it needs
class OculusDisplay : public rviz::Display, public Ogre::RenderTargetListener {
    Q_OBJECT
    public:
        OculusDisplay(rviz::RenderPanel *renderPanel,QWidget* parent = 0);
        virtual ~OculusDisplay();

        // Overrides from Display
        virtual void onInitialize();
        virtual void update( float wall_dt, float ros_dt );
        virtual void reset();

        // Overrides from Ogre::RenderTargetListener
        virtual void preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
        virtual void postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt );
        
    protected:

        virtual void onEnable();
        virtual void onDisable();

        void updateCamera();

    protected Q_SLOTS:

        /*void onFullScreenChanged();
        void onPredictionDtChanged();
        void onPubTfChanged();
        void onFollowCamChanged();
        */
        void onScreenCountChanged( int newCount );

    private:

        rviz::RenderPanel *renderWidget;
        Ogre::SceneNode *sceneNode;
        
        rviz::BoolProperty *fullscreenProperty;



};
#endif
