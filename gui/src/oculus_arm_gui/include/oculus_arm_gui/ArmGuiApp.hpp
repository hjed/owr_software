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
 
using namespace OVR;
using namespace OVR::OvrPlatform;
using namespace OVR::Render;
using namespace ROS;
 
class ArmGuiApp : public Application {
    
    public:
        //Constructor and Deconstructor
        ArmGuiApp();
        ~ArmGuiApp();
    
    protected:
    
    private:
    
 
}
 
 #endif //ARM_GUI_APP_H
