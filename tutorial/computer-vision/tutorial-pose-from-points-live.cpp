/*! \example tutorial-pose-from-points-live.cpp */
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#endif
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#include "pose_helper.h"

// Comment / uncomment following lines to use the specific 3rd party compatible with your camera
//! [Undef grabber]
//#undef VISP_HAVE_V4L2
//#undef VISP_HAVE_DC1394
//#undef VISP_HAVE_CMU1394
//#undef VISP_HAVE_OPENCV
//! [Undef grabber]

int main(int argc, char **argv)
{
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)) &&                                 \
    (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || (VISP_HAVE_OPENCV_VERSION >= 0x020100))
  try {
    std::string opt_intrinsic_file; // xml file obtained from camera calibration
    std::string opt_camera_name;    // corresponding camera name in the xml calibration file
    double opt_square_width = 0.12;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
        opt_intrinsic_file = std::string(argv[i + 1]);
      } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
        opt_camera_name = std::string(argv[i + 1]);
      } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
        opt_camera_name = std::string(argv[i + 1]);
      } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--intrinsic <xml calibration file>] [--camera_name <camera name in xml calibration file>]"
                     " [--square_width <square width in meter (default: 0.12)] [--help] [-h]\n"
                  << "\nExample using default camera parameters and square size:\n"
                  << "  " << argv[0] << "\n"
                  << "\nExample fully tuned for a 0.1m x 0.1m square:\n"
                  << "  " << argv[0] << " --intrinsic camera.xml --camera_name Camera --square_width 0.1\n"
                  << std::endl;
        return 0;
      }
    }

    vpImage<unsigned char> I;

    //! [Grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    g.setScale(1);
    g.open(I);
#elif defined(VISP_HAVE_DC1394)
    vp1394TwoGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_CMU1394)
    vp1394CMUGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_OPENCV)
    cv::VideoCapture g(0); // open the default camera
    if (!g.isOpened()) {   // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    //! [Grabber]

    // Parameters of our camera
    vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);
#ifdef VISP_HAVE_XML2
    vpXmlParserCamera parser;
    if (!opt_intrinsic_file.empty() && !opt_camera_name.empty()) {
      std::cout << "Intrinsic file: " << opt_intrinsic_file << std::endl;
      std::cout << "Camera name   : " << opt_camera_name << std::endl;
      if (parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithDistortion) == vpXmlParserCamera::SEQUENCE_OK) {
        std::cout << "Succeed to read camera parameters from xml file" << std::endl;
      } else {
        std::cout << "Unable to read camera parameters from xml file" << std::endl;
      }
    }
#endif
    std::cout << "Square width  : " << opt_square_width << std::endl;
    std::cout << cam << std::endl;

    // The pose container
    vpHomogeneousMatrix cMo;

    std::vector<vpDot2> dot(4);
    std::vector<vpPoint> point;   // 3D coordinates of the points
    std::vector<vpImagePoint> ip; // 2D coordinates of the points in pixels
    double L = opt_square_width / 2.;
    point.push_back(vpPoint(-L, -L, 0));
    point.push_back(vpPoint( L, -L, 0));
    point.push_back(vpPoint( L,  L, 0));
    point.push_back(vpPoint(-L,  L, 0));

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#endif

    bool quit = false;
    bool apply_cv = false; // apply computer vision
    bool init_cv = true;   // initialize tracking and pose computation

    while (! quit) {
      // Image Acquisition
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif

      vpDisplay::display(I);
      if (apply_cv) {
        try {
          ip  = track(I, dot, init_cv);
          computePose(point, ip, cam, init_cv, cMo);
          vpDisplay::displayFrame(I, cMo, cam, opt_square_width, vpColor::none, 3);
          if (init_cv)
            init_cv = false; // turn off the computer vision initialisation specific stuff

          { // Display estimated pose in [m] ans [deg]
            vpPoseVector pose(cMo);
            std::stringstream ss;
            ss << "Translation: " << pose.getTranslationVector().t() << " [m]";
            vpDisplay::displayText(I, 60, 20, ss.str(), vpColor::red);
            std::stringstream().swap(ss); // erase ss
            vpColVector tu_deg(pose.getThetaUVector());
            tu_deg.rad2deg();
            ss << "Rotation tu: " << tu_deg.t() << " [deg]";
            vpDisplay::displayText(I, 80, 20, ss.str(), vpColor::red);
          }
        }
        catch(...) {
          std::cout << "Computer vision failure." << std::endl;
          apply_cv = false;
          init_cv = true;
        }
      }
      vpDisplay::displayText(I, 20, 20, "Right click: quit", vpColor::red);
      if (apply_cv) {
        vpDisplay::displayText(I, 40, 20, "Computer vision in progress...", vpColor::red);
      } else {
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
      vpDisplay::flush(I);
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
#endif
}
