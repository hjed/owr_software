
/* 
 * Function Implementations for Video_Feed_Frame
 */

#include <Video_Feed_Frame.hpp>
#include <GL/freeglut.h>

// Not providing Zero argument constructor, therefore instances must be initialised in a 
//    constructor initialisor list in any classes that use this
Video_Feed_Frame::Video_Feed_Frame(int _centreX, int _centreY, int width, int height){
   centreX = _centreX;
   centreY = _centreY;
   halfWidth = width/2;
   halfHeight = height/2;

}
void Video_Feed_Frame::displayFrame(){
   glPushMatrix();
   glEnable(GL_TEXTURE_2D);
   glColor3f(1, 1, 1);
   glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
   glBindTexture(GL_TEXTURE_2D, videoTexture);
   glBegin(GL_QUADS);
          glTexCoord2f(0, 1); glVertex2i(0, -halfWidth); // Bottom Left
          glTexCoord2f(1, 1); glVertex2i(-halfHeight, halfWidth); // Bottom Right
          glTexCoord2f(1, 0); glVertex2i(halfHeight, 0); // Top Right
          glTexCoord2f(0, 0); glVertex2i(0, 0); // Top Left
   glEnd();
   glDisable(GL_TEXTURE_2D);
   glPopMatrix();
}	

#endif
