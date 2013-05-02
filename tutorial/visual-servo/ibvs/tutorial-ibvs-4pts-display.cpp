/*! \example tutorial-ibvs-4pts-display.cpp */
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpProjectionDisplay.h>
#include <visp/vpServo.h>
#include <visp/vpServoDisplay.h>
#include <visp/vpSimulatorCamera.h>

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point,
                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam)
{
  int thickness = 3;
  static std::vector<vpImagePoint> traj[4];
  vpImagePoint cog;
  for (unsigned int i=0; i<4; i++) {
    // Project the point at the given camera position
    point[i].project(cMo);
    vpMeterPixelConversion::convertPoint(cam, point[i].get_x(), point[i].get_y(), cog);
    traj[i].push_back(cog);
  }
  for (unsigned int i=0; i<4; i++) {
    for (unsigned int j=1; j<traj[i].size(); j++) {
      vpDisplay::displayLine(I, traj[i][j-1], traj[i][j], vpColor::green, thickness);
    }
  }
}

int main()
{
  vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
  vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

  std::vector<vpPoint> point(4) ;
  point[0].setWorldCoordinates(-0.1,-0.1, 0);
  point[1].setWorldCoordinates( 0.1,-0.1, 0);
  point[2].setWorldCoordinates( 0.1, 0.1, 0);
  point[3].setWorldCoordinates(-0.1, 0.1, 0);

  vpServo task ;
  task.setServo(vpServo::EYEINHAND_CAMERA);
  task.setInteractionMatrixType(vpServo::CURRENT);
  task.setLambda(0.5);

  vpFeaturePoint p[4], pd[4] ;
  for (int i = 0 ; i < 4 ; i++) {
    point[i].track(cdMo);
    vpFeatureBuilder::create(pd[i], point[i]);
    point[i].track(cMo);
    vpFeatureBuilder::create(p[i], point[i]);
    task.addFeature(p[i], pd[i]);
  }

  vpHomogeneousMatrix wMc, wMo;
  vpSimulatorCamera robot;
  robot.setSamplingTime(0.040);
  robot.getPosition(wMc);
  wMo = wMc * cMo;

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  // We open two displays, one for the internal camera view, the other one for
  // the external view, using either X11, or GDI.
  vpImage<unsigned char> Iint(480, 640, 255) ;
  vpImage<unsigned char> Iext(480, 640, 255) ;
#if defined VISP_HAVE_X11
  vpDisplayX displayInt(Iint, 0, 0, "Internal view");
  vpDisplayX displayExt(Iext, 670, 0, "External view");
#elif  defined VISP_HAVE_GDI
  vpDisplayGDI displayInt(Iint, 0, 0, "Internal view");
  vpDisplayGDI displayExt(Iext, 670, 0, "External view");
#endif
  vpProjectionDisplay externalview;
  // Parameters of our camera
  vpCameraParameters cam(840, 840, Iint.getWidth()/2, Iint.getHeight()/2);
  vpHomogeneousMatrix cextMo(0,0,3,
                             0,0,0) ;//vpMath::rad(40),  vpMath::rad(10),  vpMath::rad(60))   ;
  for (int i = 0 ; i < 4 ; i++)
    externalview.insert(point[i]) ;
#endif

  while(1) {
    robot.getPosition(wMc);
    cMo = wMc.inverse() * wMo;
    for (int i = 0 ; i < 4 ; i++) {
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
    }
    vpColVector v = task.computeControlLaw();
    robot.setVelocity(vpRobot::CAMERA_FRAME, v);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
    vpDisplay::display(Iint) ;
    vpDisplay::display(Iext) ;
    display_trajectory(Iint, point, cMo, cam);
    int thickness = 5;
    vpServoDisplay::display(task, cam, Iint, vpColor::green, vpColor::red, thickness) ;
    externalview.display(Iext, cextMo, cMo, cam, vpColor::red, true, 3) ;
    vpDisplay::flush(Iint);
    vpDisplay::flush(Iext);

    // A click to exit
    if (vpDisplay::getClick(Iint, false) || vpDisplay::getClick(Iext, false))
      break;
#endif
    vpTime::wait( robot.getSamplingTime() * 1000);
  }
  task.kill();
}

