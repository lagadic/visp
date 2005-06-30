// This example shows how to put a permanent background image on your
// viewer canvas, below the 3D graphics, plus overlay foreground
// geometry.  Written by mortene.  Copyright Systems in Motion 2002.

// *************************************************************************


#include <visp/vpConfig.h>
#include <visp/vpDebug.h>


#ifdef HAVE_LIBSOQT

#include <visp/vpImage.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpTime.h>
#include <visp/vpSimulator.h>


#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpServo.h>
#include <visp/vpRobotCamera.h>
#include <visp/vpFeatureBuilder.h>

#define SAVE 0

static
void *mainLoop (void *_simu)
{
  vpSimulator *simu = (vpSimulator *)_simu ;
  simu->initMainApplication() ;

  while (1)
  {int i ;

    vpServo task ;
    vpRobotCamera robot ;


    cout << endl ;
    cout << "-------------------------------------------------------" << endl ;
    cout << " Test program for vpServo "  <<endl ;
    cout << " Eye-in-hand task control,  articular velocity are computed" << endl ;
    cout << " Simulation " << endl ;
    cout << " task : servo 4 points " << endl ;
    cout << "-------------------------------------------------------" << endl ;
    cout << endl ;


    TRACE("sets the initial camera location " ) ;
    vpPoseVector vcMo ;

    vcMo[0] = 0.3 ;
    vcMo[1] = 0.2 ;
    vcMo[2] = 3 ;
    vcMo[3] = 0 ;
    vcMo[4] = +vpMath::rad(0)  ;
    vcMo[5] = vpMath::rad(40) ;

    vpHomogeneousMatrix cMo(vcMo) ;
    robot.setPosition(cMo) ;
    simu->setCameraPosition(cMo) ;

    vpCameraParameters cam ;

    TRACE("sets the point coordinates in the world frame "  ) ;
    vpPoint point[4] ;
    point[0].setWorldCoordinates(-0.1,-0.1,0) ;
    point[1].setWorldCoordinates(0.1,-0.1,0) ;
    point[2].setWorldCoordinates(0.1,0.1,0) ;
    point[3].setWorldCoordinates(-0.1,0.1,0) ;

    TRACE("project : computes  the point coordinates in the camera frame and its 2D coordinates"  ) ;
    for (i = 0 ; i < 4 ; i++)
      point[i].track(cMo) ;

    TRACE("sets the desired position of the point ") ;
    vpFeaturePoint p[4] ;
    for (i = 0 ; i < 4 ; i++)
      vpFeatureBuilder::create(p[i], point[i])  ;  //retrieve x,y and Z of the vpPoint structure


    TRACE("sets the desired position of the point ") ;
    vpFeaturePoint pd[4] ;

    pd[0].buildFrom(-0.05,-0.05,1) ;
    pd[1].buildFrom(0.05,-0.05,1) ;
    pd[2].buildFrom(0.05,0.05,1) ;
    pd[3].buildFrom(-0.05,0.05,1) ;

    TRACE("define the task") ;
    TRACE("\t we want an eye-in-hand control law") ;
    TRACE("\t articular velocity are computed") ;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe) ;
    task.setInteractionMatrixType(vpServo::CURRENT) ;


    TRACE("Set the position of the camera in the end-effector frame ") ;
    vpHomogeneousMatrix cMe ;
    vpTwistMatrix cVe(cMe) ;
    task.set_cVe(cVe) ;

    TRACE("Set the Jacobian (expressed in the end-effector frame)") ;
    vpMatrix eJe ;
    robot.get_eJe(eJe) ;
    task.set_eJe(eJe) ;

    TRACE("\t we want to see a point on a point..") ;
    for (i = 0 ; i < 4 ; i++)
      task.addFeature(p[i],pd[i]) ;

    TRACE("\t set the gain") ;
    task.setLambda(0.051) ;

    cout << "\nEnter a character to continue..." <<endl ;
    {    int a ; cin >> a ; }

    TRACE("Display task information " ) ;
    task.print() ;

    char name[FILENAME_MAX] ;
    int iter=0 ;
    TRACE("\t loop") ;
    while(iter++<150)
    {
      vpColVector v ;

      robot.get_eJe(eJe) ;
      task.set_eJe(eJe) ;

      robot.getPosition(cMo) ;
      for (i = 0 ; i < 4 ; i++)
      {
	point[i].track(cMo) ;
	vpFeatureBuilder::create(p[i],point[i])  ;
      }

      v = task.computeControlLaw() ;
      robot.setVelocity(vpRobot::CAMERA_FRAME, v) ;

      vpTime::wait(240) ;

      simu->setCameraPosition(cMo) ;

      if(SAVE==1)
      {
	sprintf(name,"/tmp/image.%04d.external.png",iter) ;
	cout << name << endl ;
	simu->write(vpSimulator::EXTERNAL,name) ;
	sprintf(name,"/tmp/image.%04d.internal.png",iter) ;
	simu->write(vpSimulator::INTERNAL,name) ;
      }
      {int a ;
      cin >>a ; }
    }

    TRACE("Display task information " ) ;
    task.print() ;

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
  // simu.addAbsoluteFrame() ;
  simu.load("iv/4points.iv") ;
  // simu.load("tie.iv",fMo) ;
  //  simu.load("/local/modeles-3D/starwars/jabbaTheHutt3DS/jabba.iv") ;
  simu.setCameraParameters(cam) ;
  simu.initApplication(&mainLoop) ;

  simu.mainLoop() ;
}


#else
int
main()
{
  TRACE("You should install Coin3D and SoQT") ;
}
#endif
