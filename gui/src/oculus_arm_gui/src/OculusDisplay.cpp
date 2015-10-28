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
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreCompositorManager.h>

#include <ros/package.h>

#include <rviz/properties/bool_property.h>
#include <rviz/properties/status_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/vector_property.h>

#include <QDesktopWidget>

#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>

#define DEFAULT_NEAR_CLIP 0.01f
#define DEFAULT_FAR_CLIP 10000.0f
#define DK2_ASPECT_RATIO 960.0/1080.0
const float g_defaultIPD = 0.064f;
const Ogre::ColourValue g_defaultViewportColour(97 / 255.0f, 97 / 255.0f, 200 / 255.0f);
#define DEFAULT_PROJECTION_CENTRE_OFFSET  0.14529906f
const float g_defaultDistortion[4] = {1.0f, 0.22f, 0.24f, 0.0f};
const float g_defaultChromAb[4] = {0.996, -0.004, 1.014, 0.0f};
 
OculusDisplay::OculusDisplay(rviz::RenderPanel *renderPanel,rviz::VisualizationManager* manager, QWidget* parent):  
    sceneNode(0), oculusReady(false),
    centreOffset(DEFAULT_PROJECTION_CENTRE_OFFSET){
    renderWidget = renderPanel;
    this->manager = manager;
    
    //initalise the cameras
    for(int i =0; i < NUM_EYES; i++) {
        oculusCameras[i] = NULL;
        viewport[i] = NULL;
        compositors[i] = NULL;
    }
    
    /*std::string rviz_path = ros::package::getPath(ROS_PACKAGE_NAME);
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation( rviz_path + "/ogre_media", "FileSystem", ROS_PACKAGE_NAME );
    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(ROS_PACKAGE_NAME);*/

    //this detects when the number of screen changes (i.e when an occulus is connected)
    //connect( QApplication::desktop(), SIGNAL( screenCountChanged ( int ) ), this, SLOT( onScreenCountChanged(int)) );
    //HACK: dosen't work
}

void OculusDisplay::onInitialize() {
    //create our render widget
    //renderWidget = new rviz::RenderWidget(rviz::RenderSystem::get());
    renderWidget->setWindowTitle("Oculus Arm GUI");
    //setup the window so that it works correctly with oculus
    renderWidget->setWindowFlags( Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMaximizeButtonHint );
    Ogre::RenderWindow *window = renderWidget->getRenderWindow();
    window->setVisible(false);
    window->setHidden(false);
    window->setAutoUpdated(false);
    
    fullscreenProperty = new rviz::BoolProperty( "Render to Oculus", true,
        "If checked, will render fullscreen on your secondary screen. Otherwise, shows a window.",
        this, SLOT(onFullScreenChanged()));
    
    //attach ourselves to the ORGE system so we can get the low level stuff
    window->addListener(this);
    
    sceneNode = manager->getSceneManager()->getRootSceneNode()->createChildSceneNode();
}


//called by QT when the number of screens changed
void OculusDisplay::onScreenCountChanged( int newCount)  {
  /*if ( newCount == 1 ) {
    fullscreenProperty->setBool(false);
    fullscreenProperty->setHidden(true);
    setStatus( rviz::StatusProperty::Error, "Screen", "No secondary screen detected. Cannot render to Oculus device.");
  }  else  {
    fullscreenProperty->setHidden(false);
    setStatus( rviz::StatusProperty::Ok, "Screen", "Using screen #2.");
  }*/
  fullscreenProperty->setHidden(false);
  fullscreenProperty->setBool(true);
}


void OculusDisplay::update( float wall_dt, float ros_dt ) {
    updateCamera();
    if(renderWidget->getRenderWindow()) {
        renderWidget->getRenderWindow()->update(false);
    }
}
void OculusDisplay::reset() {
    rviz::Display::reset();
    //TODO: oculus stuff
}

void OculusDisplay::preRenderTargetUpdate( const Ogre::RenderTargetEvent& evt ) {
    updateCamera();
}
void OculusDisplay::postRenderTargetUpdate( const Ogre::RenderTargetEvent& evt ) {
    renderWidget->getRenderWindow()->swapBuffers();
}

void OculusDisplay::onEnable() {
    
    //setup the oculus if its not ready
    if(!oculusReady || !hmd) {
        //intialise the ovr
       // ovrInitParams params = {0,0,NULL,0};
        ovr_Initialize();
        //create the hmd, on the 0th oculus
        hmd = ovrHmd_Create(0);
        
        //If we can't connect, create a fake one
        if(!hmd) {
            ROS_ERROR("Failed to Create HMD\n Creating a fake one");
            hmd = ovrHmd_CreateDebug(ovrHmdType(ovrHmd_DK2));
            if(!hmd) {
                ROS_ERROR("Failed to create a fake one");
                abort();
            }
        }
        
        ovrHmd_RecenterPose(hmd);
        
        //enable tracking
        ovrHmd_ConfigureTracking(hmd, ovrTrackingCap_Orientation | ovrTrackingCap_MagYawCorrection | ovrTrackingCap_Position, 0);
        
        //configure rendering
        if(ovrHmd_ConfigureRendering(hmd, 0, 0, 0, eyeRenderDesc)) {
            ROS_ERROR("Failed to configure rendering");
            //abort();
        }
        
        oculusReady = true;
        
        
        
    } else {
        ROS_ERROR("oculus not ready");
    }
    //load the oculus compositors from the oculus_rviz_plugins package
   /* Ogre::ResourceGroupManager::getSingleton().addResourceLocation();
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
    Ogre::ResourceGroupManager::getSingleton().loadResourceGroup();*/
    Ogre::ResourceGroupManager::getSingletonPtr()->createResourceGroup("OculusResources");
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(OCULUS_OGRE_COMPOS, "FileSystem", "OculusResources", true);
    Ogre::ResourceGroupManager::getSingletonPtr()->initialiseResourceGroup("OculusResources");
    Ogre::ResourceGroupManager::getSingletonPtr()->loadResourceGroup("OculusResources");
    //rviz::RenderSystem::get()->root()->addResourceLocation(OCULUS_OGRE_COMPOS, "FileSystem");
    if(Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(OCULUS_OGRE_COMPOS)) {
        ROS_ERROR("location dosen't exist");
    }
    if(!Ogre::MaterialManager::getSingleton().resourceExists("Ogre/Compositor/Oculus")) {
        ROS_ERROR("can't load compositor");
    }
    //this loads the compistor script that splits the camearas into two perscpetvies
    Ogre::MaterialPtr matLeft = Ogre::MaterialManager::getSingleton().getByName("Ogre/Compositor/Oculus");
    Ogre::MaterialPtr matRight = matLeft->clone("Ogre/Compositor/Oculus/Right");
    Ogre::GpuProgramParametersSharedPtr pParamsLeft =
        matLeft->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
    Ogre::GpuProgramParametersSharedPtr pParamsRight =
        matRight->getTechnique(0)->getPass(0)->getFragmentProgramParameters();
    
    
    //setup the cameras
    cameraNode = sceneNode->createChildSceneNode("StereoCameraNode");\
    oculusCameras[0] = manager->getSceneManager()->createCamera("left");
    oculusCameras[1] = manager->getSceneManager()->createCamera("right");
    
    //attach the eye cameras in the right place
    for(int i = 0; i < NUM_EYES; ++i) {
        cameraNode->attachObject(oculusCameras[i]);
        //setup camera focus, etc
        oculusCameras[i]->setFarClipDistance(DEFAULT_NEAR_CLIP);
        oculusCameras[i]->setNearClipDistance(DEFAULT_NEAR_CLIP);
        oculusCameras[i]->setPosition((i * 2 - 1) * OVR_DEFAULT_IPD * 0.5f, 0, 0);
        oculusCameras[i]->setFOVy(Ogre::Radian(hmd->CameraFrustumVFovInRadians*2.));
        oculusCameras[i]->setAspectRatio(DK2_ASPECT_RATIO); 
        
        renderWidget->getRenderWindow()->removeViewport(i);
        viewport[i] = renderWidget->getRenderWindow()->addViewport(oculusCameras[i],i, 0.5f * i, 0, 0.5f, 1.0f);
        compositors[i] = Ogre::CompositorManager::getSingleton().addCompositor(viewport[i],
                                                                             i == 0 ? "OculusLeft" : "OculusRight");
        compositors[i]->setEnabled(true);
    }
    
    updateProjection();
    
    renderWidget->setVisible(true);
    //check if the oculus is connected
    onScreenCountChanged( QApplication::desktop()->numScreens() );

}

//handles the camera updating
void OculusDisplay::updateProjection() {
    //starts the frame update
    ovrHmd_BeginFrameTiming(hmd, 0);
    
    //update the projection for each eye
    for(int i = 0; i < NUM_EYES; i++) {
        Ogre::Matrix4 proj = Ogre::Matrix4::IDENTITY;
        oculusCameras[i]->setCustomProjectionMatrix(false);
        
        ovrEyeType eye;
        //TODO: #define this
        if(i == 0) {
            eye = ovrEye_Left;
        } else {
            eye = ovrEye_Right;
        }
        
        ovrEyeRenderDesc renderDesc = ovrHmd_GetRenderDesc(hmd, eye, hmd->DefaultEyeFov[i]);
        
        //do hte projection
        proj.setTrans(Ogre::Vector3(renderDesc.HmdToEyeViewOffset.x, renderDesc.HmdToEyeViewOffset.y, renderDesc.HmdToEyeViewOffset.z));
        
        oculusCameras[i]->setCustomProjectionMatrix(true, proj * oculusCameras[i]->getProjectionMatrix());
        
    }
    
    //ends the frame update
    ovrHmd_EndFrameTiming(hmd);
}


void OculusDisplay::onDisable() {
    clearStatuses();
    renderWidget->setVisible(false);
    //TODO: oculus stuff
}

//Core rendering function
void OculusDisplay::updateCamera() {
    //TODO: check oculus stuff
    
    //stores the camera position
    Ogre::Vector3 pos;
    Ogre::Quaternion ori;
    
    //have the camera follow a transform frame
    context_->getFrameManager()->getTransform(CAMERA_TF, ros::Time(), pos, ori);
    Ogre::Quaternion r;
    r.FromAngleAxis( Ogre::Radian(M_PI*0.5), Ogre::Vector3::UNIT_X );
    ori = ori * r;
    r.FromAngleAxis( Ogre::Radian(-M_PI*0.5), Ogre::Vector3::UNIT_Y );
    ori = ori * r;
    
    sceneNode->setPosition(pos);
    sceneNode->setOrientation(ori);
    
    updateProjection();
    
    //setup the current 
    Ogre::Quaternion orient = calcOrientation();
    ovrEyeType eye; 
    ovrPosef currentPose = ovrHmd_GetHmdPosePerEye(hmd, eye);
    cameraNode->setPosition(currentPose.Position.x, currentPose.Position.y, currentPose.Position.z); //
    cameraNode->setOrientation(calcOrientation());
    
}


Ogre::Quaternion OculusDisplay::calcOrientation() {
    ovrTrackingState state = ovrHmd_GetTrackingState(hmd, 0.0);
    //this find the post of the head
    ovrQuatf q = state.HeadPose.ThePose.Orientation;
    return Ogre::Quaternion(q.w, q.x, q.y, q.z);
}


OculusDisplay::~OculusDisplay() {
    delete renderWidget;
    
    //if the oculus exists, shut it down
    if(hmd) {
        ovrHmd_Destroy(hmd);
        ovr_Shutdown();
        oculusReady = false;
    }
    
    //TODO: shutdown orge
}
