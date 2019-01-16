//! \example tutorial-mb-generic-tracker-rgbd-realsense.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_OPENCV)
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/vision/vpKeyPoint.h>

int main(int argc, char *argv[])
{
  std::string config_color = "", config_depth = "";
  std::string model_color = "", model_depth = "";
  std::string init_file = "";
  bool use_ogre = false;
  bool use_scanline = false;
  bool use_edges = true;
  bool use_klt = false;
  bool use_depth = true;
  bool learn = false;
  bool auto_init = false;
  double proj_error_threshold = 25;
  std::string learning_data = "learning/data-learned.bin";
  bool display_projection_error = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--config_color" && i+1 < argc) {
      config_color = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--config_depth" && i+1 < argc) {
      config_depth = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--model_color" && i+1 < argc) {
      model_color = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--model_depth" && i+1 < argc) {
      model_depth = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--init_file" && i+1 < argc) {
      init_file = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--proj_error_threshold" && i+1 < argc) {
      proj_error_threshold = std::atof(argv[i+1]);
    } else if (std::string(argv[i]) == "--use_ogre") {
      use_ogre = true;
    } else if (std::string(argv[i]) == "--use_scanline") {
      use_scanline = true;
    } else if (std::string(argv[i]) == "--use_edges" && i+1 < argc) {
      use_edges = (std::atoi(argv[i+1]) == 0 ? false : true);
    } else if (std::string(argv[i]) == "--use_klt" && i+1 < argc) {
      use_klt = (std::atoi(argv[i+1]) == 0 ? false : true);
    } else if (std::string(argv[i]) == "--use_depth" && i+1 < argc) {
      use_depth = (std::atoi(argv[i+1]) == 0 ? false : true);
    } else if (std::string(argv[i]) == "--learn") {
      learn = true;
    } else if (std::string(argv[i]) == "--learning_data" && i+1 < argc) {
      learning_data = argv[i+1];
    } else if (std::string(argv[i]) == "--auto_init") {
      auto_init = true;
    } else if (std::string(argv[i]) == "--display_proj_error") {
      display_projection_error = true;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: \n" << argv[0]
                << " [--config_color <object.xml>] [--config_depth <object.xml>] [--model_color <object.cao>] [--model_depth <object.cao>]"
                   " [--init_file <object.init>] [--use_ogre] [--use_scanline]"
                   " [--proj_error_threshold <threshold between 0 and 90> (default: "<< proj_error_threshold << ")]"
                   " [--use_edges <0|1> (default: 1)] [--use_klt <0|1> (default: 0)] [--use_depth <0|1> (default: 1)]"
                   " [--learn] [--auto_init] [--learning_data <path to .bin> (default: learning/data-learned.bin)]"
                   " [--display_proj_error]" << std::endl;
      std::cout << "\nExample to track a 4.2 cm width cube with manual initialization:\n" << argv[0]
                << " --config_color cube.xml --config_depth cube_depth.xml"
                << " --model_color cube.cao --model_depth cube.cao"
                << " --init_file cube.init"
                << " --use_edges 1 --use_klt 1 --use_depth 1"
                << std::endl;
      std::cout << "\nExample to learn the 4.2 cm width cube:\n" << argv[0]
                << " --config_color cube.xml --config_depth cube_depth.xml"
                << " --model_color cube.cao --model_depth cube.cao"
                << " --init_file cube.init"
                << " --use_edges 1 --use_klt 1 --use_depth 1"
                << " --learn"
                << std::endl;
      std::cout << "\nExample to track a 4.2 cm width cube with initialization from learning database:\n" << argv[0]
                << " --config_color cube.xml --config_depth cube_depth.xml"
                << " --model_color cube.cao --model_depth cube.cao"
                << " --init_file cube.init"
                << " --use_edges 1 --use_klt 1 --use_depth 1"
                << " --auto_init"
                << std::endl;
      return 0;
    }
  }

  std::cout << "Tracked features: " << std::endl;
  std::cout << "  Use edges   : " << use_edges << std::endl;
  std::cout << "  Use klt     : " << use_klt << std::endl;
  std::cout << "  Use depth   : " << use_depth << std::endl;
  std::cout << "Tracker options: " << std::endl;
  std::cout << "  Use ogre    : " << use_ogre << std::endl;
  std::cout << "  Use scanline: " << use_scanline << std::endl;
  std::cout << "  Proj. error : " << proj_error_threshold << std::endl;
  std::cout << "  Display proj. error: " << display_projection_error << std::endl;
  std::cout << "Config files: " << std::endl;
  std::cout << "  Config color: " << "\"" << config_color << "\"" << std::endl;
  std::cout << "  Config depth: " << "\"" << config_depth << "\"" << std::endl;
  std::cout << "  Model color : " << "\"" << model_color << "\"" << std::endl;
  std::cout << "  Model depth : " << "\"" << model_depth << "\"" << std::endl;
  std::cout << "  Init file   : " << "\"" << init_file << "\"" << std::endl;
  std::cout << "Learning options   : " << std::endl;
  std::cout << "  Learn       : " << learn << std::endl;
  std::cout << "  Auto init   : " << auto_init << std::endl;
  std::cout << "  Learning data: " << learning_data << std::endl;

  if (!use_edges && !use_klt && !use_depth) {
    std::cout << "You must choose at least one visual features between edge, KLT and depth." << std::endl;
    return EXIT_FAILURE;
  }

  if (config_color.empty() || config_depth.empty() || model_color.empty() || model_depth.empty() || init_file.empty()) {
    std::cout << "config_color.empty() || config_depth.empty() || model_color.empty() || model_depth.empty() || init_file.empty()" << std::endl;
    return EXIT_FAILURE;
  }

  vpRealSense2 realsense;
  int width = 640, height = 480;
  int fps = 30;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
  config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);

  try {
    realsense.open(config);
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
    std::cout << "Check if the Realsense camera is connected..." << std::endl;
    return EXIT_SUCCESS;
  }

  vpCameraParameters cam_color = realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
  vpCameraParameters cam_depth = realsense.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);

  std::cout << "Sensor internal camera parameters for color camera: " << cam_color << std::endl;
  std::cout << "Sensor internal camera parameters for depth camera: " << cam_depth << std::endl;

  vpImage<vpRGBa> I_color(height, width);
  vpImage<unsigned char> I_gray(height, width);
  vpImage<unsigned char> I_depth(height, width);
  vpImage<uint16_t> I_depth_raw(height, width);

  unsigned int _posx = 100, _posy = 50;

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d1, d2;
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d1, d2;
#endif
  if (use_edges || use_klt)
    d1.init(I_gray, _posx, _posy, "Color stream");
  if (use_depth)
    d2.init(I_depth, _posx + I_gray.getWidth()+10, _posy, "Depth stream");

  while (true) {
    realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, NULL, NULL);

    if (use_edges || use_klt) {
      vpImageConvert::convert(I_color, I_gray);
      vpDisplay::display(I_gray);
      vpDisplay::displayText(I_gray, 20, 20, "Click when ready.", vpColor::red);
      vpDisplay::flush(I_gray);

      if (vpDisplay::getClick(I_gray, false)) {
        break;
      }
    }
    if (use_depth) {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

      vpDisplay::display(I_depth);
      vpDisplay::displayText(I_depth, 20, 20, "Click when ready.", vpColor::red);
      vpDisplay::flush(I_depth);

      if (vpDisplay::getClick(I_depth, false)) {
        break;
      }
    }
  }

  std::vector<int> trackerTypes;
  if (use_edges && use_klt)
    trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  else if (use_edges)
    trackerTypes.push_back(vpMbGenericTracker::EDGE_TRACKER );
  else if (use_klt)
    trackerTypes.push_back(vpMbGenericTracker::KLT_TRACKER);

  if (use_depth)
    trackerTypes.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);

  vpHomogeneousMatrix depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  std::map<std::string, std::string> mapOfInitFiles;
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraPoses;

  std::vector<vpColVector> pointcloud;

  vpMbGenericTracker tracker(trackerTypes);

  if ((use_edges || use_klt) && use_depth) {
    tracker.loadConfigFile(config_color, config_depth);
    tracker.loadModel(model_color, model_depth);
    std::cout << "Sensor internal depth_M_color: \n" << depth_M_color << std::endl;
    mapOfCameraTransformations["Camera2"] = depth_M_color;
    tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
    mapOfImages["Camera1"] = &I_gray;
    mapOfImages["Camera2"] = &I_depth;
    mapOfInitFiles["Camera1"] = init_file;
    tracker.setCameraParameters(cam_color, cam_depth);
  }
  else if (use_edges || use_klt) {
    tracker.loadConfigFile(config_color);
    tracker.loadModel(model_color);
    tracker.setCameraParameters(cam_color);
  }
  else if (use_depth) {
    tracker.loadConfigFile(config_depth);
    tracker.loadModel(model_depth);
    tracker.setCameraParameters(cam_depth);
  }

  tracker.setDisplayFeatures(true);
  tracker.setOgreVisibilityTest(use_ogre);
  tracker.setScanLineVisibilityTest(use_scanline);
  tracker.setProjectionErrorComputation(true);
  tracker.setProjectionErrorDisplay(display_projection_error);

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
  if (learn || auto_init) {
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

  if (auto_init) {
    if (!vpIoTools::checkFilename(learning_data)) {
      std::cout << "Cannot enable auto detection. Learning file \"" << learning_data << "\" doesn't exist" << std::endl;
      return EXIT_FAILURE;
    }
    keypoint.loadLearningData(learning_data, true);
  } else {
    if ((use_edges || use_klt) && use_depth)
      tracker.initClick(mapOfImages, mapOfInitFiles, true);
    else if (use_edges || use_klt)
      tracker.initClick(I_gray, init_file, true);
    else if (use_depth)
      tracker.initClick(I_depth, init_file, true);

    if (learn)
      vpIoTools::makeDirectory(vpIoTools::getParent(learning_data));
  }


  bool run_auto_init = false;
  if (auto_init) {
    run_auto_init = true;
  }
  std::vector<double> times_vec;

  try {
    //To be able to display keypoints matching with test-detection-rs2
    int learn_id = 1;
    bool quit = false;
    bool learn_position = false;
    double loop_t = 0;
    vpHomogeneousMatrix cMo;

    while (!quit) {
      double t = vpTime::measureTimeMs();
      bool tracking_failed = false;

      // Acquire images and update tracker input data
      realsense.acquire((unsigned char *) I_color.bitmap, (unsigned char *) I_depth_raw.bitmap, &pointcloud, NULL, NULL);

      if (use_edges || use_klt || run_auto_init) {
        vpImageConvert::convert(I_color, I_gray);
        vpDisplay::display(I_gray);
      }
      if (use_depth) {
        vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
        vpDisplay::display(I_depth);
      }

      if ((use_edges || use_klt) && use_depth) {
        mapOfImages["Camera1"] = &I_gray;
        mapOfPointclouds["Camera2"] = &pointcloud;
        mapOfWidths["Camera2"] = width;
        mapOfHeights["Camera2"] = height;
      } else if (use_edges || use_klt) {
        mapOfImages["Camera"] = &I_gray;
      } else if (use_depth) {
        mapOfPointclouds["Camera"] = &pointcloud;
        mapOfWidths["Camera"] = width;
        mapOfHeights["Camera"] = height;
      }

      // Run auto initialization from learned data
      if (run_auto_init) {
        if (keypoint.matchPoint(I_gray, cam_color, cMo)) {
          std::cout << "Auto init succeed" << std::endl;
          if ((use_edges || use_klt) && use_depth) {
            mapOfCameraPoses["Camera1"] = cMo;
            mapOfCameraPoses["Camera2"] = depth_M_color *cMo;
            tracker.initFromPose(mapOfImages, mapOfCameraPoses);
          } else if (use_edges || use_klt) {
            tracker.initFromPose(I_gray, cMo);
          } else if (use_depth) {
            tracker.initFromPose(I_depth, depth_M_color*cMo);
          }
        } else {
          if (use_edges || use_klt) {
            vpDisplay::flush(I_gray);
          }
          if (use_depth) {
            vpDisplay::flush(I_depth);
          }
          continue;
        }
      }

      // Run the tracker
      try {
        if (run_auto_init) {
          // Turn display features off just after auto init to not display wrong moving-edge if the tracker fails
          tracker.setDisplayFeatures(false);

          run_auto_init = false;
        }
        if ((use_edges || use_klt) && use_depth) {
          tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
        } else if (use_edges || use_klt) {
          tracker.track(I_gray);
        } else if (use_depth) {
          tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
        }
      } catch (const vpException &e) {
        std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
        tracking_failed = true;
        if (auto_init) {
          std::cout << "Tracker needs to restart (tracking exception)" << std::endl;
          run_auto_init = true;
        }
      }

      // Get object pose
      cMo = tracker.getPose();

      // Check tracking errors
      double proj_error = 0;
      if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
        // Check tracking errors
        proj_error = tracker.getProjectionError();
      }
      else {
        proj_error = tracker.computeCurrentProjectionError(I_gray, cMo, cam_color);
      }

      if (auto_init && proj_error > proj_error_threshold) {
        std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
        run_auto_init = true;
        tracking_failed = true;
      }

      // Display tracking results
      if (!tracking_failed) {
        // Turn display features on
        tracker.setDisplayFeatures(true);

        if ((use_edges || use_klt) && use_depth) {
          tracker.display(I_gray, I_depth, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::red, 3);
          vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
          vpDisplay::displayFrame(I_depth, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);
        } else if (use_edges || use_klt) {
          tracker.display(I_gray, cMo, cam_color, vpColor::red, 3);
          vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
        } else if (use_depth) {
          tracker.display(I_depth, cMo, cam_depth, vpColor::red, 3);
          vpDisplay::displayFrame(I_depth, cMo, cam_depth, 0.05, vpColor::none, 3);
        }

        if (use_edges || use_klt) {
          std::stringstream ss;
          ss << "Nb features: " << tracker.getError().getRows();
          vpDisplay::displayText(I_gray, 20, I_gray.getWidth() - 150, ss.str(), vpColor::red);
        } else if (use_depth) {
          std::stringstream ss;
          ss << "Nb features: " << tracker.getError().getRows();
          vpDisplay::displayText(I_depth, 20, I_depth.getWidth() - 150, ss.str(), vpColor::red);
        }
      }

      std::stringstream ss;
      ss << "Loop time: " << loop_t << " ms";

      vpMouseButton::vpMouseButtonType button;
      if (use_edges || use_klt) {
        vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);
        if (learn)
          vpDisplay::displayText(I_gray, 35, 20, "Left click: learn  Right click: quit", vpColor::red);
        else if (auto_init)
          vpDisplay::displayText(I_gray, 35, 20, "Left click: auto_init  Right click: quit", vpColor::red);
        else
          vpDisplay::displayText(I_gray, 35, 20, "Right click: quit", vpColor::red);

        vpDisplay::flush(I_gray);

        if (vpDisplay::getClick(I_gray, button, false)) {
          if (button == vpMouseButton::button3) {
            quit = true;
          } else if (button == vpMouseButton::button1 && learn) {
            learn_position = true;
          } else if (button == vpMouseButton::button1 && auto_init && !learn) {
            run_auto_init = true;
          }
        }
      }
      if (use_depth) {
        vpDisplay::displayText(I_depth, 20, 20, ss.str(), vpColor::red);
        vpDisplay::displayText(I_depth, 40, 20, "Click to quit", vpColor::red);
        vpDisplay::flush(I_depth);

        if (vpDisplay::getClick(I_depth, false)) {
          quit = true;
        }
      }

      if (learn_position) {
        // Detect keypoints on the current image
        std::vector<cv::KeyPoint> trainKeyPoints;
        keypoint.detect(I_gray, trainKeyPoints);

        // Keep only keypoints on the cube
        std::vector<vpPolygon> polygons;
        std::vector<std::vector<vpPoint> > roisPt;
        std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces();
        polygons = pair.first;
        roisPt = pair.second;

        // Compute the 3D coordinates
        std::vector<cv::Point3f> points3f;
        vpKeyPoint::compute3DForPointsInPolygons(cMo, cam_color, trainKeyPoints, polygons, roisPt, points3f);

        // Build the reference keypoints
        keypoint.buildReference(I_gray, trainKeyPoints, points3f, true, learn_id++);

        // Display learned data
        for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
          vpDisplay::displayCross(I_gray, (int)it->pt.y, (int)it->pt.x, 10, vpColor::yellow, 3);
        }
        learn_position = false;
        std::cout << "Data learned" << std::endl;
      }
      loop_t = vpTime::measureTimeMs() - t;
      times_vec.push_back(loop_t);
    }
    if (learn) {
      std::cout << "Save learning file: " << learning_data << std::endl;
      keypoint.saveLearningData(learning_data, true, true);
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.what() << std::endl;
  }

  if (!times_vec.empty()) {
    std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec) << " ms ; Median: " << vpMath::getMedian(times_vec)
              << " ; Std: " << vpMath::getStdev(times_vec) << " ms" << std::endl;
  }

  return EXIT_SUCCESS;
}
#elif defined(VISP_HAVE_REALSENSE2)
int main() {
  std::cout << "Install OpenCV 3rd party, configure and build ViSP again to use this example" << std::endl;
  return 0;
}
#else
int main() {
  std::cout << "Install librealsense2 3rd party, configure and build ViSP again to use this example" << std::endl;
  return 0;
}
#endif
