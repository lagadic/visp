// This example shows how to put a permanent background image on your
// viewer canvas, below the 3D graphics, plus overlay foreground
// geometry.  Written by mortene.  Copyright Systems in Motion 2002.

// *************************************************************************


#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpSimulator.h>


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpDebug.h>
#include <visp/vpFeatureBuilder.h>

#define SAVE 1

static
void *mainLoop (void *_simu)
{
  vpSimulator *simu = (vpSimulator *)_simu ;
  simu->initMainApplication() ;


  vpServo task ;
  vpRobotCamera robot ;

  // you initialization

  while(1)
  {
    robot.getPosition(cMo) ;
    // you visual servoing loop

    v = task.computeControlLaw() ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    simu->setCameraPosition(cMo) ;
  }
  simu->closeMainApplication() ;

  return (void *) ;
}


int
main(int argc, char ** argv)
{
  vpCameraParameters cam ;

  vpSimulator simu ;
  simu.initInternalViewer(200,200) ;
  simu.initExternalViewer(200,200) ;

  simu.load("iv/4points.iv") ;
  simu.setCameraParameters(cam) ;
  simu.initApplication(&mainLoop) ;

  simu.mainLoop() ;
}
