//! \example tutorial-mb-generic-tracker-rgbd-realsense-json.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_OPENCV) && defined(VISP_HAVE_NLOHMANN_JSON)
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/sensor/vpRealSense2.h>

#include <nlohmann/json.hpp>
using json = nlohmann::json; //! json namespace shortcut


int main(int argc, char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string config_file = "";
  std::string model = "";
  std::string init_file = "";

  double proj_error_threshold = 25;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--config" && i + 1 < argc) {
      config_file = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--model" && i + 1 < argc) {
      model = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--init_file" && i + 1 < argc) {
      init_file = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--proj_error_threshold" && i + 1 < argc) {
      proj_error_threshold = std::atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: \n"
        << argv[0]
        << "--config <settings.json>"
        << " --model <object.cao>"
        " --init_file <object.init>"
        " [--proj_error_threshold <threshold between 0 and 90> (default: "
        << proj_error_threshold
        << ")]"
        << std::endl;

      std::cout << "\n** How to track a 4.2 cm width cube with manual initialization:\n"
        << argv[0] << "--config model/cube/rgbd-tracker.json --model model/cube/cube.cao" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Config files: " << std::endl;
  std::cout << "  JSON config: "
    << "\"" << config_file << "\"" << std::endl;
  std::cout << "  Model: "
    << "\"" << model << "\"" << std::endl;
  std::cout << "  Init file: "
    << "\"" << init_file << "\"" << std::endl;

  if (config_file.empty()) {
    std::cout << "No JSON configuration was provided!" << std::endl;
    return EXIT_FAILURE;
  }
  //! [Init]
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

  vpCameraParameters cam_color =
    realsense.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
  vpCameraParameters cam_depth =
    realsense.getCameraParameters(RS2_STREAM_DEPTH, vpCameraParameters::perspectiveProjWithoutDistortion);

  std::cout << "Sensor internal camera parameters for color camera: " << cam_color << std::endl;
  std::cout << "Sensor internal camera parameters for depth camera: " << cam_depth << std::endl;

  vpImage<vpRGBa> I_color(height, width); // acquired by the realsense
  vpImage<unsigned char> I_gray(height, width); // Fed to the tracker if we use edge or KLT features
  vpImage<unsigned char> I_depth(height, width); // Depth histogram used for display
  vpImage<uint16_t> I_depth_raw(height, width); // Raw depth acquired by the realsense
  std::vector<vpColVector> pointcloud; // fed to the tracker
  //! [Init]

  vpHomogeneousMatrix depth_M_color = realsense.getTransformation(RS2_STREAM_COLOR, RS2_STREAM_DEPTH);
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  std::map<std::string, std::string> mapOfInitFiles;
  std::map<std::string, const std::vector<vpColVector> *> mapOfPointclouds;
  std::map<std::string, unsigned int> mapOfWidths, mapOfHeights;
  std::map<std::string, vpCameraParameters> mapOfCameraIntrinsics;

  //! [Loading]
  vpMbGenericTracker tracker;
  tracker.loadConfigFile(config_file);
  //! [Loading]
  if (model.empty() && init_file.empty()) {
    std::ifstream config(config_file);
    const json j = json::parse(config);
    config.close();
    if (j.contains("model")) {
      model = j["model"];
    }
    else {
      std::cerr << "No model was provided in either JSON file or arguments" << std::endl;
      return EXIT_FAILURE;
    }
  }
  if (init_file.empty()) {
    const std::string parentname = vpIoTools::getParent(model);
    init_file = (parentname.empty() ? "" : (parentname + "/")) + vpIoTools::getNameWE(model) + ".init";
  }

  //! [Init maps]
  std::string color_key = "", depth_key = "";
  for (const auto &tracker_type : tracker.getCameraTrackerTypes()) {
    std::cout << "tracker key == " << tracker_type.first << std::endl;
    // Initialise for color features
    if (tracker_type.second & vpMbGenericTracker::EDGE_TRACKER || tracker_type.second & vpMbGenericTracker::KLT_TRACKER) {
      color_key = tracker_type.first;
      mapOfImages[color_key] = &I_gray;
      mapOfInitFiles[color_key] = init_file;
      mapOfWidths[color_key] = width;
      mapOfHeights[color_key] = height;
      mapOfCameraIntrinsics[color_key] = cam_color;
    }
    // Initialize for depth features
    if (tracker_type.second & vpMbGenericTracker::DEPTH_DENSE_TRACKER || tracker_type.second & vpMbGenericTracker::DEPTH_NORMAL_TRACKER) {
      depth_key = tracker_type.first;
      mapOfImages[depth_key] = &I_depth;
      mapOfWidths[depth_key] = width;
      mapOfHeights[depth_key] = height;
      mapOfCameraIntrinsics[depth_key] = cam_depth;
      mapOfCameraTransformations[depth_key] = depth_M_color;
      mapOfPointclouds[depth_key] = &pointcloud;
    }
  }
  const bool use_depth = !depth_key.empty();
  const bool use_color = !color_key.empty();
  //! [Init maps]
  //! [Load 3D model]
  if (tracker.getNbPolygon() == 0) { // Not already loaded by JSON
    tracker.loadModel(model);
  }
  //! [Load 3D model]

  //! [Update params]
  std::cout << "Updating configuration with parameters provided by RealSense SDK..." << std::endl;
  tracker.setCameraParameters(mapOfCameraIntrinsics);
  if (use_color && use_depth) {
    tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
  }
  //! [Update params]
  unsigned int _posx = 100, _posy = 50;

#ifdef VISP_HAVE_X11
  vpDisplayX d1, d2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d1, d2;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV d1, d2;
#endif
  if (use_color)
    d1.init(I_gray, _posx, _posy, "Color stream");
  if (use_depth)
    d2.init(I_depth, _posx + I_gray.getWidth() + 10, _posy, "Depth stream");

  while (true) {
    realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, nullptr, nullptr);

    if (use_color) {
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

  tracker.setProjectionErrorComputation(true);
  //! [Init tracking]
  tracker.initClick(mapOfImages, mapOfInitFiles, true);
  //! [Init tracking]
  //! [Tracking]
  std::vector<double> times_vec;
  try {
    bool quit = false;
    double loop_t = 0;
    vpHomogeneousMatrix cMo;

    while (!quit) {
      double t = vpTime::measureTimeMs();
      bool tracking_failed = false;

      // Acquire images and update tracker input data
      realsense.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap, &pointcloud, nullptr, nullptr);

      if (use_color) {
        vpImageConvert::convert(I_color, I_gray);
        vpDisplay::display(I_gray);
      }
      if (use_depth) {
        vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
        vpDisplay::display(I_depth);
      }

      if (use_color && use_depth) {
        mapOfImages[color_key] = &I_gray;
        mapOfPointclouds[depth_key] = &pointcloud;
        mapOfWidths[depth_key] = width;
        mapOfHeights[depth_key] = height;
      }
      else if (use_color) {
        mapOfImages[color_key] = &I_gray;
      }
      else if (use_depth) {
        mapOfPointclouds[depth_key] = &pointcloud;
      }

      // Run the tracker
      try {
        if (use_color && use_depth) {
          tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
        }
        else if (use_color) {
          tracker.track(I_gray);
        }
        else if (use_depth) {
          tracker.track(mapOfImages, mapOfPointclouds, mapOfWidths, mapOfHeights);
        }
      }
      catch (const vpException &e) {
        std::cout << "Tracker exception: " << e.getStringMessage() << std::endl;
        tracking_failed = true;
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

      if (proj_error > proj_error_threshold) {
        std::cout << "Tracker needs to restart (projection error detected: " << proj_error << ")" << std::endl;
      }

      // Display tracking results
      if (!tracking_failed) {
        // Turn display features on
        tracker.setDisplayFeatures(true);

        if (use_color && use_depth) {
          tracker.display(I_gray, I_depth, cMo, depth_M_color * cMo, cam_color, cam_depth, vpColor::red, 3);
          vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
          vpDisplay::displayFrame(I_depth, depth_M_color * cMo, cam_depth, 0.05, vpColor::none, 3);
        }
        else if (use_color) {
          tracker.display(I_gray, cMo, cam_color, vpColor::red, 3);
          vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
        }
        else if (use_depth) {
          tracker.display(I_depth, cMo, cam_depth, vpColor::red, 3);
          vpDisplay::displayFrame(I_depth, cMo, cam_depth, 0.05, vpColor::none, 3);
        }

        { // Display total number of features
          std::stringstream ss;
          ss << "Nb features: " << tracker.getError().size();
          vpDisplay::displayText(I_gray, I_gray.getHeight() - 50, 20, ss.str(), vpColor::red);
        }
        { // Display number of feature per type
          std::stringstream ss;
          ss << "Features: edges " << tracker.getNbFeaturesEdge() << ", klt " << tracker.getNbFeaturesKlt()
            << ", depth dense " << tracker.getNbFeaturesDepthDense() << ", depth normal" << tracker.getNbFeaturesDepthNormal();
          vpDisplay::displayText(I_gray, I_gray.getHeight() - 30, 20, ss.str(), vpColor::red);
        }
      }

      std::stringstream ss;
      loop_t = vpTime::measureTimeMs() - t;
      times_vec.push_back(loop_t);
      ss << "Loop time: " << loop_t << " ms";

      vpMouseButton::vpMouseButtonType button;
      if (use_color) {
        vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);
        vpDisplay::displayText(I_gray, 35, 20, "Right click: quit", vpColor::red);

        vpDisplay::flush(I_gray);

        if (vpDisplay::getClick(I_gray, button, false)) {
          if (button == vpMouseButton::button3) {
            quit = true;
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

    }
  }
  catch (const vpException &e) {
    std::cout << "Caught an exception: " << e.what() << std::endl;
  }
  // Show tracking performance
  if (!times_vec.empty()) {
    std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec)
      << " ms ; Median: " << vpMath::getMedian(times_vec) << " ; Std: " << vpMath::getStdev(times_vec) << " ms"
      << std::endl;
  }
  //! [Tracking]
  return EXIT_SUCCESS;
}
#elif defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_OPENCV)
int main()
{
  std::cout << "Install the JSON 3rd party library (Nlohmann JSON), reconfigure VISP and build again" << std::endl;
  return EXIT_SUCCESS;
}
#elif defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_NLOHMANN_JSON)
int main()
{
  std::cout << "Install OpenCV, reconfigure VISP and build again" << std::endl;
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "Install librealsense2 3rd party, configure and build ViSP again to use this example" << std::endl;
  return EXIT_SUCCESS;
}
#endif
