/*! \example tutorial-ibvs-4pts-display.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/gui/vpProjectionDisplay.h>
#include <visp3/robot/vpSimulatorCamera.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

void display_trajectory(const vpImage<unsigned char> &I, std::vector<vpPoint> &point, const vpHomogeneousMatrix &cMo,
                        const vpCameraParameters &cam)
{
  VP_ATTRIBUTE_NO_DESTROY static std::vector<vpImagePoint> traj[4];
  vpImagePoint cog;
  for (unsigned int i = 0; i < 4; i++) {
    // Project the point at the given camera position
    point[i].project(cMo);
    vpMeterPixelConversion::convertPoint(cam, point[i].get_x(), point[i].get_y(), cog);
    traj[i].push_back(cog);
  }
  for (unsigned int i = 0; i < 4; i++) {
    for (unsigned int j = 1; j < traj[i].size(); j++) {
      vpDisplay::displayLine(I, traj[i][j - 1], traj[i][j], vpColor::green);
    }
  }
}

int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> displayInt;
  std::shared_ptr<vpDisplay> displayExt;
#else
  vpDisplay *displayInt = nullptr;
  vpDisplay *displayExt = nullptr;
#endif
  try {
    vpHomogeneousMatrix cdMo(0, 0, 0.75, 0, 0, 0);
    vpHomogeneousMatrix cMo(0.15, -0.1, 1., vpMath::rad(10), vpMath::rad(-10), vpMath::rad(50));

    std::vector<vpPoint> point;
    point.push_back(vpPoint(-0.1, -0.1, 0));
    point.push_back(vpPoint(0.1, -0.1, 0));
    point.push_back(vpPoint(0.1, 0.1, 0));
    point.push_back(vpPoint(-0.1, 0.1, 0));

    vpServo task;
    task.setServo(vpServo::EYEINHAND_CAMERA);
    task.setInteractionMatrixType(vpServo::CURRENT);
    task.setLambda(0.5);

    vpFeaturePoint p[4], pd[4];
    for (unsigned int i = 0; i < 4; i++) {
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

    vpImage<unsigned char> Iint(480, 640, 255);
    vpImage<unsigned char> Iext(480, 640, 255);
#if defined(VISP_HAVE_DISPLAY)
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    displayInt = vpDisplayFactory::createDisplay(Iint, 0, 0, "Internal view");
    displayExt = vpDisplayFactory::createDisplay(Iext, 670, 0, "External view");
#else
    displayInt = vpDisplayFactory::allocateDisplay(Iint, 0, 0, "Internal view");
    displayExt = vpDisplayFactory::allocateDisplay(Iext, 670, 0, "External view");
#endif
    vpProjectionDisplay externalview;
    for (unsigned int i = 0; i < 4; i++)
      externalview.insert(point[i]);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpCameraParameters cam(840, 840, Iint.getWidth() / 2, Iint.getHeight() / 2);
    vpHomogeneousMatrix cextMo(0, 0, 3, 0, 0, 0);

    while (1) {
      robot.getPosition(wMc);
      cMo = wMc.inverse() * wMo;
      for (unsigned int i = 0; i < 4; i++) {
        point[i].track(cMo);
        vpFeatureBuilder::create(p[i], point[i]);
      }
      vpColVector v = task.computeControlLaw();
      robot.setVelocity(vpRobot::CAMERA_FRAME, v);

      vpDisplay::display(Iint);
      vpDisplay::display(Iext);
      display_trajectory(Iint, point, cMo, cam);

      vpServoDisplay::display(task, cam, Iint, vpColor::green, vpColor::red);
#if defined(VISP_HAVE_DISPLAY)
      externalview.display(Iext, cextMo, cMo, cam, vpColor::red, true);
#endif
      vpDisplay::flush(Iint);
      vpDisplay::flush(Iext);

      // A click to exit
      if (vpDisplay::getClick(Iint, false) || vpDisplay::getClick(Iext, false))
        break;

      vpTime::wait(robot.getSamplingTime() * 1000);
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (displayInt != nullptr) {
    delete displayInt;
  }
  if (displayExt != nullptr) {
    delete displayExt;
  }
#endif
}
