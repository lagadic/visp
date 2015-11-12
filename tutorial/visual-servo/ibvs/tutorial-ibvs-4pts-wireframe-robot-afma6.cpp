/*! \example tutorial-ibvs-4pts-wireframe-robot-afma6.cpp */
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/robot/vpSimulatorAfma6.h>

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point,
                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam);

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point,
                        const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam)
{
  unsigned int thickness = 3;
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
#if defined(VISP_HAVE_PTHREAD)
  try {
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(-0.15, 0.1, 1., vpMath::rad(-10), vpMath::rad(10), vpMath::rad(50));

    /*
    Top view of the world frame, the camera frame and the object frame

    world, also robot base frame :
                     w_y
                  /|\
                   |
             w_x <--

    object :
                     o_y
                  /|\
                   |
             o_x <--


    camera :
                     c_y
                  /|\
                   |
             c_x <--

    */
    vpHomogeneousMatrix wMo(0, 0, 1., 0, 0, 0);

    std::vector<vpPoint> point;
    point.push_back( vpPoint(-0.1,-0.1, 0) );
    point.push_back( vpPoint( 0.1,-0.1, 0) );
    point.push_back( vpPoint( 0.1, 0.1, 0) );
    point.push_back( vpPoint(-0.1, 0.1, 0) );

    vpServo task ;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);

    vpFeaturePoint p[4], pd[4] ;
    for (unsigned int i = 0 ; i < 4 ; i++) {
      point[i].track(cdMo);
      vpFeatureBuilder::create(pd[i], point[i]);
      point[i].track(cMo);
      vpFeatureBuilder::create(p[i], point[i]);
      task.addFeature(p[i], pd[i]);
    }

    vpSimulatorAfma6 robot(true);
    robot.setVerbose(true);

    // Get the default joint limits
    vpColVector qmin = robot.getJointMin();
    vpColVector qmax = robot.getJointMax();

    std::cout << "Robot joint limits: " << std::endl;
    for (unsigned int i=0; i< 3; i ++)
      std::cout << "Joint " << i << ": min " << qmin[i] << " max " << qmax[i] << " (m)" << std::endl;
    for (unsigned int i=3; i< qmin.size(); i ++)
      std::cout << "Joint " << i << ": min " << vpMath::deg(qmin[i]) << " max " << vpMath::deg(qmax[i]) << " (deg)" << std::endl;

    robot.init(vpAfma6::TOOL_CCMOP, vpCameraParameters::perspectiveProjWithoutDistortion);
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    robot.initScene(vpWireFrameSimulator::PLATE, vpWireFrameSimulator::D_STANDARD);
    robot.set_fMo(wMo);
    bool ret = true;
#if VISP_VERSION_INT > VP_VERSION_INT(2,7,0)
    ret =
    #endif
        robot.initialiseCameraRelativeToObject(cMo);
    if (ret == false)
      return 0; // Not able to set the position
    robot.setDesiredCameraPosition(cdMo);

    vpImage<unsigned char> Iint(480, 640, 255);
#if defined(VISP_HAVE_X11)
    vpDisplayX displayInt(Iint, 700, 0, "Internal view");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI displayInt(Iint, 700, 0, "Internal view");
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV displayInt(Iint, 700, 0, "Internal view");
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpCameraParameters cam(840, 840, Iint.getWidth()/2, Iint.getHeight()/2);
    robot.setCameraParameters(cam);

    bool start = true;
    for ( ; ; )
    {
      cMo = robot.get_cMo();

      for (unsigned int i = 0 ; i < 4 ; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }

      vpDisplay::display(Iint);
      robot.getInternalView(Iint);
      if (!start) {
        display_trajectory(Iint, point, cMo, cam);
        vpDisplay::displayText(Iint, 40, 120, "Click to stop the servo...", vpColor::red);
      }
      vpDisplay::flush(Iint);

      vpColVector v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      // A click to exit
      if (vpDisplay::getClick(Iint, false))
        break;

      if (start) {
        start = false;
        v = 0;
        robot.setVelocity(vpRobot::CAMERA_FRAME, v);
        vpDisplay::displayText(Iint, 40, 120, "Click to start the servo...", vpColor::blue);
        vpDisplay::flush(Iint);
        vpDisplay::getClick(Iint);
      }

      vpTime::wait(1000*robot.getSamplingTime());
    }
    task.kill();
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
