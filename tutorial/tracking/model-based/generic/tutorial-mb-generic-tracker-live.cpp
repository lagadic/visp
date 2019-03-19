//! \example tutorial-mb-generic-tracker-live.cpp
#include <visp3/core/vpConfig.h>
#ifdef VISP_HAVE_MODULE_SENSOR
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/sensor/vp1394CMUGrabber.h>
#include <visp3/sensor/vp1394TwoGrabber.h>
#include <visp3/sensor/vpFlyCaptureGrabber.h>
#include <visp3/sensor/vpRealSense2.h>
#endif
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpKeyPoint.h>
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

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) &&                                 \
    (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || (VISP_HAVE_OPENCV_VERSION >= 0x020100) || \
    defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2) )

  try {
    std::string opt_modelname = "model/teabox/teabox.cao";
    int opt_tracker = 2;
    int opt_device = 0;             // For OpenCV and V4l2 grabber to set the camera device
    double opt_proj_error_threshold = 30.;
    bool opt_use_ogre = false;
    bool opt_use_scanline = false;
    bool opt_display_projection_error = false;
    bool opt_learn = false;
    bool opt_auto_init = false;
    std::string opt_learning_data = "learning/data-learned.bin";
    std::string opt_intrinsic_file = "";
    std::string opt_camera_name = "";

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--model") {
        opt_modelname = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--tracker") {
        opt_tracker = atoi(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--camera_device" && i + 1 < argc) {
        opt_device = atoi(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--max_proj_error") {
        opt_proj_error_threshold = atof(argv[i + 1]);
      } else if (std::string(argv[i]) == "--use_ogre") {
        opt_use_ogre = true;
      } else if (std::string(argv[i]) == "--use_scanline") {
        opt_use_scanline = true;
      } else if (std::string(argv[i]) == "--learn") {
        opt_learn = true;
      } else if (std::string(argv[i]) == "--learning_data" && i+1 < argc) {
        opt_learning_data = argv[i+1];
      } else if (std::string(argv[i]) == "--auto_init") {
        opt_auto_init = true;
      } else if (std::string(argv[i]) == "--display_proj_error") {
        opt_display_projection_error = true;
      } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
        opt_intrinsic_file = std::string(argv[i + 1]);
      } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
        opt_camera_name = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--camera_device <camera device> (default: 0)]"
                  << " [--intrinsic <intrinsic file> (default: empty)]"
                  << " [--camera_name <camera name>  (default: empty)]"
                  << " [--model <model name> (default: teabox)]"
                  << " [--tracker <0=egde|1=keypoint|2=hybrid> (default: 2)]"
                  << " [--use_ogre] [--use_scanline]"
                  << " [--max_proj_error <allowed projection error> (default: 30)]"
                  << " [--learn] [--auto_init] [--learning_data <data-learned.bin> (default: learning/data-learned.bin)]"
                  << " [--display_proj_error]"
                  << " [--help] [-h]\n"
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

    std::cout << "Tracked features: " << std::endl;
    std::cout << "  Use edges   : " << (opt_tracker == 0 || opt_tracker == 2) << std::endl;
    std::cout << "  Use klt     : " << (opt_tracker == 1 || opt_tracker == 2) << std::endl;
    std::cout << "Tracker options: " << std::endl;
    std::cout << "  Use ogre    : " << opt_use_ogre << std::endl;
    std::cout << "  Use scanline: " << opt_use_scanline << std::endl;
    std::cout << "  Proj. error : " << opt_proj_error_threshold << std::endl;
    std::cout << "  Display proj. error: " << opt_display_projection_error << std::endl;
    std::cout << "Config files: " << std::endl;
    std::cout << "  Config file : " << "\"" << objectname + ".xml" << "\"" << std::endl;
    std::cout << "  Model file  : " << "\"" << objectname + ".cao" << "\"" << std::endl;
    std::cout << "  Init file   : " << "\"" << objectname + ".init"  << "\"" << std::endl;
    std::cout << "Learning options   : " << std::endl;
    std::cout << "  Learn       : " << opt_learn << std::endl;
    std::cout << "  Auto init   : " << opt_auto_init << std::endl;
    std::cout << "  Learning data: " << opt_learning_data << std::endl;

    //! [Image]
#if VISP_VERSION_INT > VP_VERSION_INT(3, 2, 0)
    vpImage<vpRGBa> I;         // Since ViSP 3.2.0 we support model-based tracking on color images
#else
    vpImage<unsigned char> I;  // Tracking on gray level images
#endif
    //! [Image]

    //! [Set camera parameters]
    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    //! [Set camera parameters]
#ifdef VISP_HAVE_XML2
    vpXmlParserCamera parser;
    if (!opt_intrinsic_file.empty() && !opt_camera_name.empty())
      parser.parse(cam, opt_intrinsic_file, opt_camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
#endif

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
    (void)opt_device; // To avoid non used warning
    std::cout << "Use DC1394 grabber" << std::endl;
    vp1394TwoGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_CMU1394)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use CMU1394 grabber" << std::endl;
    vp1394CMUGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_FLYCAPTURE)
    (void)opt_device; // To avoid non used warning
    std::cout << "Use FlyCapture grabber" << std::endl;
    vpFlyCaptureGrabber g;
    g.open(I);
#elif defined(VISP_HAVE_REALSENSE2)
    (void)opt_device; // To avoid non used warning
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

    while (true) {
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif

      vpDisplay::display(I);
      vpDisplay::displayText(I, 20, 20, "Click when ready.", vpColor::red);
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false)) {
        break;
      }
    }

    //! [Constructor]
    vpMbGenericTracker tracker;
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

    bool usexml = false;
    //! [Load xml]
#ifdef VISP_HAVE_XML2
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      usexml = true;
    }
#endif
    //! [Load xml]

    if (!usexml) {
      //! [Set parameters]
      if (opt_tracker == 0 || opt_tracker == 2) {
        //! [Set moving-edges parameters]
        vpMe me;
        me.setMaskSize(5);
        me.setMaskNumber(180);
        me.setRange(8);
        me.setThreshold(10000);
        me.setMu1(0.5);
        me.setMu2(0.5);
        me.setSampleStep(4);
        tracker.setMovingEdge(me);
        //! [Set moving-edges parameters]
      }

#ifdef VISP_HAVE_MODULE_KLT
      if (opt_tracker == 1 || opt_tracker == 2) {
        //! [Set klt parameters]
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
        //! [Set klt parameters]
      }
#endif
    }

    tracker.setCameraParameters(cam);
    //! [Set parameters]

    //! [Load cao]
    tracker.loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Set display]
    tracker.setDisplayFeatures(true);
    //! [Set display]
    //! [Set visibility algorithm]
    tracker.setOgreVisibilityTest(opt_use_ogre);
    tracker.setScanLineVisibilityTest(opt_use_scanline);
    //! [Set visibility algorithm]
    //! [Set projection error computation]
    tracker.setProjectionErrorComputation(true);
    tracker.setProjectionErrorDisplay(opt_display_projection_error);
    //! [Set projection error computation]

#if (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
    std::string detectorName = "SIFT";
    std::string extractorName = "SIFT";
    std::string matcherName = "BruteForce";
#else
    std::string detectorName = "FAST";
    std::string extractorName = "ORB";
    std::string matcherName = "BruteForce-Hamming";
#endif
    vpKeyPoint keypoint;
    if (opt_learn || opt_auto_init) {
      keypoint.setDetector(detectorName);
      keypoint.setExtractor(extractorName);
      keypoint.setMatcher(matcherName);
#if !(defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
#  if (VISP_HAVE_OPENCV_VERSION < 0x030000)
      keypoint.setDetectorParameter("ORB", "nLevels", 1);
#  else
      cv::Ptr<cv::ORB> orb_detector = keypoint.getDetector("ORB").dynamicCast<cv::ORB>();
      if (orb_detector) {
        orb_detector->setNLevels(1);
      }
#  endif
#endif
    }

    if (opt_auto_init) {
      if (!vpIoTools::checkFilename(opt_learning_data)) {
        std::cout << "Cannot enable auto detection. Learning file \"" << opt_learning_data << "\" doesn't exist" << std::endl;
        return EXIT_FAILURE;
      }
      keypoint.loadLearningData(opt_learning_data, true);
    }
    else {
      tracker.initClick(I, objectname + ".init", true);
    }

    bool learn_position = false;
    bool run_auto_init = false;
    if (opt_auto_init) {
      run_auto_init = true;
    }

    //To be able to display keypoints matching with test-detection-rs2
    int learn_id = 1;
    unsigned int learn_cpt = 0;
    bool quit = false;
    bool tracking_failed = false;

    while (!quit) {
      double t_begin = vpTime::measureTimeMs();
#if defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_DC1394) || defined(VISP_HAVE_CMU1394) || defined(VISP_HAVE_FLYCAPTURE) || defined(VISP_HAVE_REALSENSE2)
      g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
      g >> frame;
      vpImageConvert::convert(frame, I);
#endif
      vpDisplay::display(I);

      // Run auto initialization from learned data
      if (run_auto_init) {
        tracking_failed = false;
        if (keypoint.matchPoint(I, cam, cMo)) {
          std::cout << "Auto init succeed" << std::endl;
          tracker.initFromPose(I, cMo);
        } else {
          vpDisplay::flush(I);
          continue;
        }
      }
      else if (tracking_failed) {
        // Manual init
        tracking_failed = false;
        tracker.initClick(I, objectname + ".init", true);
      }

      // Run the tracker
      try {
        if (run_auto_init) {
          // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
          tracker.setDisplayFeatures(false);

          run_auto_init = false;
        }
        tracker.track(I);
      } catch (const vpException &e) {
        std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
        tracking_failed = true;
        if (opt_auto_init) {
          std::cout << "Tracker needs to restart (tracking exception)" << std::endl;
          run_auto_init = true;
        }
      }

      if (! tracking_failed) {
        double proj_error = 0;
        if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
          // Check tracking errors
          proj_error = tracker.getProjectionError();
        }
        else {
          tracker.getPose(cMo);
          tracker.getCameraParameters(cam);
          proj_error = tracker.computeCurrentProjectionError(I, cMo, cam);
        }
        if (proj_error > opt_proj_error_threshold) {
          std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
          if (opt_auto_init) {
            run_auto_init = true;
          }
          tracking_failed = true;
        }
      }

      if (! tracking_failed) {
        tracker.setDisplayFeatures(true);
        //! [Get pose]
        tracker.getPose(cMo);
        //! [Get pose]
        //! [Display]
        tracker.getCameraParameters(cam);
        tracker.display(I, cMo, cam, vpColor::green, 2, false);
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

      if (learn_position) {
        learn_cpt ++;
        // Detect keypoints on the current image
        std::vector<cv::KeyPoint> trainKeyPoints;
        keypoint.detect(I, trainKeyPoints);

        // Keep only keypoints on the cube
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces();
        polygons = pair.first;
        roisPt = pair.second;

        // Compute the 3D coordinates
        std::vector<cv::Point3f> points3f;
        vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);

        // Build the reference keypoints
        keypoint.buildReference(I, trainKeyPoints, points3f, true, learn_id++);

        // Display learned data
        for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
          vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 10, vpColor::yellow, 3);
        }
        learn_position = false;
        std::cout << "Data learned" << std::endl;
      }

      std::stringstream ss;
      ss << "Loop time: " << vpTime::measureTimeMs() - t_begin << " ms";
      vpDisplay::displayText(I, 20, 20, ss.str(), vpColor::red);
      if (opt_learn)
        vpDisplay::displayText(I, 35, 20, "Left click: learn  Right click: quit", vpColor::red);
      else if (opt_auto_init)
        vpDisplay::displayText(I, 35, 20, "Left click: auto_init  Right click: quit", vpColor::red);
      else
        vpDisplay::displayText(I, 35, 20, "Right click: quit", vpColor::red);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button3) {
          quit = true;
        } else if (button == vpMouseButton::button1 && opt_learn) {
          learn_position = true;
        } else if (button == vpMouseButton::button1 && opt_auto_init && !opt_learn) {
          run_auto_init = true;
        }
      }

      vpDisplay::flush(I);
    }
    if (opt_learn && learn_cpt) {
      std::cout << "Save learning from " << learn_cpt << " images in file: " << opt_learning_data << std::endl;
      keypoint.saveLearningData(opt_learning_data, true, true);
    }

    //! [Cleanup]
    delete display;
    //! [Cleanup]
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
#elif defined(VISP_HAVE_OPENCV)
  (void) argc;
  (void) argv;
  std::cout << "Install a 3rd party dedicated to frame grabbing (dc1394, cmu1394, v4l2, OpenCV, FlyCapture, Realsense2), configure and build ViSP again to use this example" << std::endl;
#else
  (void) argc;
  (void) argv;
  std::cout << "Install OpenCV 3rd party, configure and build ViSP again to use this example" << std::endl;
#endif
}
