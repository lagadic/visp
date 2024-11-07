/*! \example tutorial-pose-from-points-realsense-T265.cpp */
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpRealSense2.h>
#endif
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include "pose_helper.h"

int main(int argc, char **argv)
{
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)) &&                                 \
    defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    double opt_square_width = 0.12;
    int opt_camera_index = 1; // camera index: 1. Left, 2.Right

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--camera_index" && i + 1 < argc) {
        opt_camera_index = atoi(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--square_width" && i + 1 < argc) {
        opt_square_width = atoi(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0] << " [--camera_index <1.Left | 2.Right> (default: 1)]"
          << " [--square_width <square width in meter (default: 0.12)] [--help] [-h]\n"
          << "\nExample using right camera and square size 0.1:\n"
          << "  " << argv[0] << "--camera_index 2 --square_width 0.1\n"
          << std::endl;
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I;

    //! [Grabber]
    std::cout << "Use Realsense 2 grabber" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.enable_stream(RS2_STREAM_FISHEYE, 1);
    config.enable_stream(RS2_STREAM_FISHEYE, 2);

    g.open(config);
    if (opt_camera_index == 1) // Left camera
      g.acquire(&I, nullptr, nullptr);
    else
      g.acquire(nullptr, &I, nullptr);

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    // Parameters of our camera
    vpCameraParameters cam = g.getCameraParameters(
        RS2_STREAM_FISHEYE, vpCameraParameters::ProjWithKannalaBrandtDistortion, opt_camera_index);
    //! [Grabber]

    std::cout << "Square width  : " << opt_square_width << std::endl;
    std::cout << cam << std::endl;

    // The pose container
    vpHomogeneousMatrix cMo;

    std::vector<vpDot2> dot(4);
    std::vector<vpPoint> point;   // 3D coordinates of the points
    std::vector<vpImagePoint> ip; // 2D coordinates of the points in pixels
    double L = opt_square_width / 2.;
    point.push_back(vpPoint(-L, -L, 0));
    point.push_back(vpPoint(L, -L, 0));
    point.push_back(vpPoint(L, L, 0));
    point.push_back(vpPoint(-L, L, 0));

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV d(I);
#endif

    bool quit = false;
    bool apply_cv = false; // apply computer vision
    bool init_cv = true;   // initialize tracking and pose computation

    while (!quit) {
      double t_begin = vpTime::measureTimeMs();

      if (opt_camera_index == 1)
        g.acquire(&I, nullptr, nullptr);
      else
        g.acquire(nullptr, &I, nullptr);

      vpDisplay::display(I);
      if (apply_cv) {
        try {
          ip = track(I, dot, init_cv);
          computePose(point, ip, cam, init_cv, cMo);
          vpDisplay::displayFrame(I, cMo, cam, opt_square_width, vpColor::none, 3);
          if (init_cv)
            init_cv = false; // turn off the computer vision initialisation specific stuff

          { // Display estimated pose in [m] and [deg]
            vpPoseVector pose(cMo);
            std::stringstream ss;
            ss << "Translation: " << std::setprecision(5) << pose[0] << " " << pose[1] << " " << pose[2] << " [m]";
            vpDisplay::displayText(I, 60, 20, ss.str(), vpColor::red);
            ss.str(""); // erase ss
            ss << "Rotation tu: " << std::setprecision(4) << vpMath::deg(pose[3]) << " " << vpMath::deg(pose[4]) << " "
              << vpMath::deg(pose[5]) << " [deg]";
            vpDisplay::displayText(I, 80, 20, ss.str(), vpColor::red);
          }
        }
        catch (...) {
          std::cout << "Computer vision failure." << std::endl;
          apply_cv = false;
          init_cv = true;
        }
      }
      vpDisplay::displayText(I, 20, 20, "Right click: quit", vpColor::red);
      if (apply_cv) {
        vpDisplay::displayText(I, 40, 20, "Computer vision in progress...", vpColor::red);
      }
      else {
        vpDisplay::displayText(I, 40, 20, "Left click : start", vpColor::red);
      }
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button3) {
          quit = true;
        }
        else if (button == vpMouseButton::button1) {
          apply_cv = true;
        }
      }
      {
        std::stringstream ss;
        ss << "Time: " << vpTime::measureTimeMs() - t_begin << " ms";
        vpDisplay::displayText(I, 20, I.getWidth() - 100, ss.str(), vpColor::red);
      }
      vpDisplay::flush(I);
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
#elif !defined(VISP_HAVE_REALSENSE2)
  (void)argc;
  (void)argv;
  std::cout << "You do not realsense2 SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install librealsense2, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#elif !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  (void)argc;
  (void)argv;
  std::cout << "Install a 3rd party dedicated to image display (X11, GDI, OpenCV), configure and build ViSP again to "
    "use this example"
    << std::endl;
#elif !(RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
  (void)argc;
  (void)argv;
  std::cout << "Install librealsense version > 2.31.0" << std::endl;
#endif
}
