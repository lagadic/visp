/*!
  \example testCircle.cpp
  \brief Visual servoing experiment on a circle with a visualization
  from the camera and from an external view
*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>


#ifdef HAVE_LIBSOQT

#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpSimulator.h>



#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpCircle.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>



#define SAVE 0

static
void *mainLoop (void *_simu)
{
  vpSimulator *simu = (vpSimulator *)_simu ;
  simu->initMainApplication() ;

  vpPoseVector vcMo ;

  vcMo[0] = 0.3 ;
  vcMo[1] = 0.2 ;
  vcMo[2] = 3 ;
  vcMo[3] = 0 ;
  vcMo[4] = vpMath::rad(45)  ;
  vcMo[5] = vpMath::rad(40) ;
  vpHomogeneousMatrix cMo(vcMo) ; ;

  vpHomogeneousMatrix cMod ;
  cMod[0][3] = 0 ;
  cMod[1][3] = 0 ;
  cMod[2][3] = 1 ;

  int it =0 ;
  int pos = 2 ;
  while (pos!=0)
  {


  vpServo task ;
  vpRobotCamera robot ;

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Test program for vpServo "  <<endl ;
  cout << " Simulation " << endl ;
  cout << " task : servo a circle " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;


  TRACE("sets the initial camera location " ) ;


  robot.setPosition(cMo) ;
  simu->setCameraPosition(cMo) ;


  if (pos==1)  cMod[2][3] = 0.32 ;

  cout << " Hit a key "<< endl ;
  {int a ; cin >>a ; } ;

  TRACE("sets the circle coordinates in the world frame "  ) ;
  vpCircle circle ;
  circle.setWorldCoordinates(0,0,1,0,0,0,0.1) ;

  TRACE("sets the desired position of the visual feature ") ;
  vpFeatureEllipse pd ;
  circle.track(cMod) ;
  vpFeatureBuilder::create(pd,circle)  ;

  TRACE("project : computes  the circle coordinates in the camera frame and its 2D coordinates"  ) ;

  TRACE("sets the current position of the visual feature ") ;
  vpFeatureEllipse p ;
  circle.track(cMo) ;
  vpFeatureBuilder::create(p,circle)  ;

  TRACE("define the task") ;
  TRACE("\t we want an eye-in-hand control law") ;
  TRACE("\t robot is controlled in the camera frame") ;
  task.setServo(vpServo::EYEINHAND_CAMERA) ;
  task.setInteractionMatrixType(vpServo::CURRENT) ;

  TRACE("\t we want to see a circle on a circle..") ;
  cout << endl ;
  task.addFeature(p,pd) ;

  TRACE("\t set the gain") ;
  if (pos==2)  task.setLambda(0.0251) ;
  else task.setLambda(0.0251) ;


  TRACE("Display task information " ) ;
  task.print() ;

  int iter=0 ;
  TRACE("\t loop") ;
  int itermax ;
  if (pos==2) itermax = 75 ; else itermax = 100 ;
    char name[FILENAME_MAX] ;
  while(iter++<itermax)
  {
    cout << "---------------------------------------------" << iter <<endl ;
    vpColVector v ;

    if (iter==1) TRACE("\t\t get the robot position ") ;
    robot.getPosition(cMo) ;
    if (iter==1) TRACE("\t\t new circle position ") ;
    //retrieve x,y and Z of the vpCircle structure

    circle.track(cMo) ;
    vpFeatureBuilder::create(p,circle);

    if (iter==1) TRACE("\t\t compute the control law ") ;
    v = task.computeControlLaw() ;
    //  TRACE("computeControlLaw" ) ;
    cout << task.rankJ1 <<endl ;
    if (iter==1) TRACE("\t\t send the camera velocity to the controller ") ;
    robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

    simu->setCameraPosition(cMo) ;
    vpTime::wait(40) ;

    sprintf(name,"/tmp/marchand/image.%04d.external.png",it) ;
    cout << name << endl ;
    if(SAVE==1)
    {
    	simu->write(vpSimulator::EXTERNAL,name) ;
	sprintf(name,"/tmp/marchand/image.%04d.internal.png",iter) ;
	simu->write(vpSimulator::INTERNAL,name) ;
      it++ ;
    }
    //  TRACE("\t\t || s - s* || ") ;
    //  cout << task.error.sumSquare() <<endl ; ;
  }
  pos-- ;
  }


  simu->closeMainApplication() ;

  void *a=NULL ;
  return a ;
  // return (void *);
}


int
main(int argc, char ** argv)
{
  vpCameraParameters cam ;
  vpHomogeneousMatrix fMo ; fMo[2][3] = 0 ;

  vpSimulator simu ;
  simu.initInternalViewer(10,10) ;
  simu.initExternalViewer(10,10) ;

  vpTime::wait(1000) ;
  simu.setZoomFactor(0.2) ;
  simu.addAbsoluteFrame() ;

  simu.load("iv/circle.iv",fMo) ;

  simu.setCameraParameters(cam) ;
  simu.initApplication(&mainLoop) ;

  simu.mainLoop() ;
}


#else
int
main()
{  TRACE("You should install Coin3D and SoQT") ;

}
#endif
