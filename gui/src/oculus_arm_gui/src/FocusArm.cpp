/**
 * Uses mouse input to center the oculus on the correct point.
 * Started by: Harry J.E Day
 * Date: 5/11/2015
 */
 
#include "oculus_arm_gui/FocusArm.hpp"

 
 FocusArm::FocusArm() {
    //key to activate this mode
    shortcut_key_ = 'o';
    
 }
 
 FocusArm::~FocusArm() {
    //does nothing
 }
 
 void FocusArm::onInitalize() {
    
 }
 
 void FocusArm::activate() {
 }
 
 void FocusArm::deactivate() {
 
 }
 
 int FocusArm::processMouseEvent( rviz::ViewportMouseEvent& event) {
    if (event.leftDown() ) {
        
    }
    return Render;
 }
