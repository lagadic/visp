#include <visp/vpConfig.h>

#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImageIo.h>
// For 2D image 
#include <visp/vpImage.h>
// Video device interface
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGTK.h>

// For frame transformation and projection
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpCameraParameters.h>

// Needed geometric features
#include <visp/vpPoint.h>
#include <visp/vpLine.h>
#include <visp/vpCylinder.h>
#include <visp/vpCircle.h>
#include <visp/vpSphere.h>

int main()
{
  #ifdef VISP_HAVE_GTK
  
  std::cout << "ViSP geometric features display example" <<std::endl;
  unsigned int height = 288;
  unsigned int width = 384;
  vpImage<unsigned char> I(height,width);
  I = 255; // I is a white image

  // create a display window
  vpDisplayGTK display;
  // initialize a display attached to image I
  display.init(I,100,100,"ViSP geometric features display");
  // camera parameters to digitalize the image plane 
  vpCameraParameters cam(600,600,width/2,height/2); // px,py,u0,v0

  // pose of the camera with reference to the scene
  vpTranslationVector t(0,0,1);
  vpRxyzVector rxyz(-M_PI/4,0,0);
  vpRotationMatrix R(rxyz);
  vpHomogeneousMatrix cMo(R,t);

  // scene building, geometric features definition
  vpPoint point;
  point.setWorldCoordinates(0,0,0);// (X0=0,Y0=0,Z0=0)
  vpLine line;
  line.setWorldCoordinates(1,1,0,0,0,0,1,0); // planes:(X+Y=0)&(Z=0)
  vpCylinder cylinder;
  cylinder.setWorldCoordinates(1,-1,0,0,0,0,0.1); // alpha=1,beta=-1,gamma=0,
                                                  // X0=0,Y0=0,Z0=0,R=0.1
  vpCircle circle;
  circle.setWorldCoordinates(0,0,1,0,0,0,0.1); // plane:(Z=0),X0=0,Y0=0,Z=0,R=0.1
  vpSphere sphere;
  sphere.setWorldCoordinates(0,0,0,0.1); // X0=0,Y0=0,Z0=0,R=0.1

  // change frame to be the camera frame and project features in the image plane
  point.project(cMo);
  line.project(cMo);
  cylinder.project(cMo);
  circle.project(cMo);  
  sphere.project(cMo);  

  // display the scene
  vpDisplay::display(I); // display I
  // draw the projections of the 3D geometric features in the image plane.
  point.display(I,cam,vpColor::black);   // draw a black cross over I 
  line.display(I,cam,vpColor::blue);     // draw a blue line over I 
  cylinder.display(I,cam,vpColor::red);  // draw two red lines over I 
  circle.display(I,cam,vpColor::orange); // draw an orange ellipse over I 
  sphere.display(I,cam,vpColor::black);  // draw a black ellipse over I 

  vpDisplay::flush(I);    // flush the display buffer
  std::cout << "A click in the display to exit" << std::endl;  
  vpDisplay::getClick(I); // wait for a click in the display to exit

  // save the drawing
  vpImage<vpRGBa> Ic;
  vpDisplay::getImage(I,Ic);
  std::cout << "ViSP creates \"./geometricFeatures.ppm\" B&W image "<< std::endl;
  vpImageIo::writePPM(Ic, "./geometricFeatures.ppm");

  #endif
  return 0;
}
