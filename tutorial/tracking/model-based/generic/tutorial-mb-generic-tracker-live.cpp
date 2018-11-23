//! \example tutorial-mb-generic-tracker.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#endif
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
//! [Include]
#include <visp3/mbt/vpMbGenericTracker.h>
//! [Include]

//! [Undef grabber]
//#undef VISP_HAVE_V4L2
//#undef VISP_HAVE_DC1394
//#undef VISP_HAVE_CMU1394
//#undef VISP_HAVE_FLYCAPTURE
//#undef VISP_HAVE_REALSENSE2
//#undef VISP_HAVE_OPENCV
//! [Undef grabber]

typedef enum {
  StateWait,
  StateInit,
  StateTrack,
  StateEnd
} TrackerState_t;

int main(int argc, char **argv)
{
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)) &&                                 \
    (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2) )

  try {
    std::string opt_modelname = "teabox";
    int opt_tracker = 2;
    int opt_device = 0;             // For OpenCV and V4l2 grabber to set the camera device
    TrackerState_t tracker_state = StateWait;
    double opt_max_projection_error = 20.;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--model") {
        opt_modelname = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--tracker") {
        opt_tracker = atoi(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--input_device" && i + 1 < argc) {
        opt_device = atoi(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--max_proj_error") {
        opt_max_projection_error = atof(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--input_device <camera device> (default: 0)]"
                  << " [--model <model name> (default: teabox)]"
                     " [--tracker <0=egde|1=keypoint|2=hybrid> (default: 2)] "
                     " [--max_proj_error <allowed projection error> (default: 20)] "
                     " [--help] [-h]\n"
                  << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_modelname);
    std::string objectname = vpIoTools::getNameWE(opt_modelname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Tracker requested config files: " << objectname << ".[init, cao]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;

    //! [Image]
    vpImage<unsigned char> I;
    vpCameraParameters cam;
    //! [Image]
    //! [cMo]
    vpHomogeneousMatrix cMo;
    //! [cMo]

    //! [Grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    std::cout << "Use Video 4 Linux grabber on device " << device.str() << std::endl;
    g.setDevice(device.str());
    g.setScale(1);
    g.open(I);
#elif defined(VISP_HAVE_DC1394)
    std::cout << "Use DC1394 grabber" << std::endl;
    vp1394TwoGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_CMU1394)
    std::cout << "Use CMU1394 grabber" << std::endl;
    vp1394CMUGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_FLYCAPTURE)
    std::cout << "Use FlyCapture grabber" << std::endl;
    vpFlyCaptureGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_REALSENSE2)
    std::cout << "Use Realsense 2 grabber" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    g.open(config);
    g.acquire(I);

    std::cout << "Read camera parameters from Realsense device" << std::endl;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
#elif defined(VISP_HAVE_OPENCV)
    std::cout << "Use OpenCV grabber on device " << opt_device << std::endl;
    cv::VideoCapture g(opt_device); // Open the default camera
    if (!g.isOpened()) {            // Check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    g >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    //! [Grabber]

    vpDisplay *display = NULL;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display = new vpDisplayGDI;
#else
    display = new vpDisplayOpenCV;
#endif
    display->init(I, 100, 100, "Model-based tracker");

    //! [Constructor]
    vpMbGenericTracker tracker;
    tracker.setProjectionErrorComputation(true); // To detect tracking failure
    if (opt_tracker == 0)
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
#if defined(VISP_HAVE_MODULE_KLT) && defined(VISP_HAVE_OPENCV)
    else if (opt_tracker == 1)
      tracker.setTrackerType(vpMbGenericTracker::KLT_TRACKER);
    else
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
#else
    else {
#  if !defined(VISP_HAVE_MODULE_KLT)
      std::cout << "klt and hybrid model-based tracker are not available since visp_klt module is not available. "
                   "In CMakeGUI turn visp_klt module ON, configure and build ViSP again."
                << std::endl;
#  else
      std::cout << "Hybrid tracking is impossible since OpenCV is not enabled. "
                << "Install OpenCV, configure and build ViSP again to run this tutorial."
                << std::endl;
#  endif
      return EXIT_SUCCESS;
    }
#endif
    //! [Constructor]

    //! [Set parameters]
    if (opt_tracker == 0 || opt_tracker == 2) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      tracker.setMovingEdge(me);
    }

#ifdef VISP_HAVE_MODULE_KLT
    if (opt_tracker == 1 || opt_tracker == 2) {
      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(5);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      tracker.setKltOpencv(klt_settings);
      tracker.setKltMaskBorder(5);
    }
#endif

    //! [Set camera parameters]
    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    //! [Set camera parameters]
    tracker.setCameraParameters(cam);
    //! [Set parameters]

    //! [Load cao]
    tracker.loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Set display]
    tracker.setDisplayFeatures(true);
    //! [Set display]

    while (tracker_state != StateEnd) {
      double t_begin = vpTime::measureTimeMs();
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);

      if (tracker_state == StateInit) {
        try {
          //! [Init tracker]
          tracker.initClick(I, objectname + ".init", true);
          //! [Init tracker]
          tracker_state = StateTrack;
        }
        catch(...) {
          std::cout << "Unable to initialize the tracker" << std::endl;
          tracker_state = StateWait;
        }
      }

      else if (tracker_state == StateTrack) {
        try {
          //! [Track]
          tracker.track(I);
          //! [Track]

          double projection_error = tracker.getProjectionError();
          {
            std::stringstream ss;
            ss << "Projection error: " << std::setprecision(3) << projection_error << std::endl;
            vpDisplay::displayText(I, 60, 20, ss.str(), vpColor::green);
          }
          if (projection_error > opt_max_projection_error) {
            std::cout << "Lost tracking: reprojection error " << projection_error << "> " << opt_max_projection_error << std::endl;
            tracker_state = StateWait;
          }
          else {
            //! [Get pose]
            tracker.getPose(cMo);
            //! [Get pose]
            //! [Display]
            tracker.getCameraParameters(cam);
            tracker.display(I, cMo, cam, vpColor::green, 2, true);
            //! [Display]
            vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);

            { // Display estimated pose in [m] and [deg]
              vpPoseVector pose(cMo);
              std::stringstream ss;
              ss << "Translation: " << std::setprecision(5) << pose[0] << " " << pose[1] << " " << pose[2] << " [m]";
              vpDisplay::displayText(I, 80, 20, ss.str(), vpColor::green);
              ss.str(""); // erase ss
              ss << "Rotation tu: " << std::setprecision(4) << vpMath::deg(pose[3]) << " " << vpMath::deg(pose[4]) << " " << vpMath::deg(pose[5]) << " [deg]";
              vpDisplay::displayText(I, 100, 20, ss.str(), vpColor::green);
            }
          }
        }
        catch(...) {
          std::cout << "Lost tracking due to an internal exception" << std::endl;
          tracker_state = StateWait;
        }
      }

      vpDisplay::displayText(I, 20, 20, "Left  click: init/reinit tracker", vpColor::green);
      vpDisplay::displayText(I, 40, 20, "Right click: quit", vpColor::green);

      {
        std::stringstream ss;
        ss << "Time: " << vpTime::measureTimeMs() - t_begin << " ms";
        vpDisplay::displayText(I, 20, I.getWidth()-100, ss.str(), vpColor::green);
      }

      vpDisplay::flush(I);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          tracker_state = StateInit;
        }
        else if (button == vpMouseButton::button3) {
          tracker_state = StateEnd;
        }
      }
    }
        //! [Cleanup]
    delete display;
    //! [Cleanup]
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
#elif (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  (void) argc;
  (void) argv;
  std::cout << "Install a 3rd party dedicated to frame grabbing (dc1394, cmu1394, v4l2, OpenCV, FlyCapture, Realsense2), configure and build ViSP again to use this example" << std::endl;
#else
  (void) argc;
  (void) argv;
  std::cout << "Install a 3rd party dedicated to image display (X11, GDI, OpenCV), configure and build ViSP again to use this example" << std::endl;
#endif
}
