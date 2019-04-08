//! \example tutorial-mb-generic-tracker-rgbd-blender.cpp
#include <iostream>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/mbt/vpMbGenericTracker.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020403) && defined(VISP_HAVE_XML2)
namespace {
bool read_data(unsigned int cpt, const std::string &input_directory, vpImage<unsigned char> &I,
               vpImage<uint16_t> &I_depth_raw, unsigned int &depth_width, unsigned int &depth_height,
               std::vector<vpColVector> &pointcloud, const vpCameraParameters &cam,
               vpHomogeneousMatrix &cMo_ground_truth)
{
  char buffer[FILENAME_MAX];
  // Read color
  std::stringstream ss;
  ss << input_directory << "/images/%04d.jpg";
  sprintf(buffer, ss.str().c_str(), cpt);
  std::string filename_img = buffer;

  if (!vpIoTools::checkFilename(filename_img)) {
    std::cerr << "Cannot read: " << filename_img << std::endl;
    return false;
  }
  vpImageIo::read(I, filename_img);

  // Read depth
  ss.str("");
  ss << input_directory << "/depth/Image%04d.exr";
  sprintf(buffer, ss.str().c_str(), cpt);
  std::string filename_depth = buffer;

  cv::Mat depth_raw = cv::imread(filename_depth, cv::IMREAD_ANYDEPTH | cv::IMREAD_ANYCOLOR);
  if (depth_raw.empty()) {
    std::cerr << "Cannot read: " << filename_depth << std::endl;
    return false;
  }

  depth_width = static_cast<unsigned int>(depth_raw.cols);
  depth_height = static_cast<unsigned int>(depth_raw.rows);
  I_depth_raw.resize(depth_height, depth_width);
  pointcloud.resize(depth_width*depth_height);

  for (int i = 0; i < depth_raw.rows; i++) {
    for (int j = 0; j < depth_raw.cols; j++) {
      I_depth_raw[i][j] = static_cast<uint16_t>(32767.5f * depth_raw.at<cv::Vec3f>(i, j)[0]);
      double x = 0.0, y = 0.0;
      // Manually limit the field of view of the depth camera
      double Z = depth_raw.at<cv::Vec3f>(i, j)[0] > 2.0f ? 0.0 : static_cast<double>(depth_raw.at<cv::Vec3f>(i, j)[0]);
      vpPixelMeterConversion::convertPoint(cam, j, i, x, y);
      size_t idx = static_cast<size_t>(i*depth_raw.cols + j);
      pointcloud[idx].resize(3);
      pointcloud[idx][0] = x*Z;
      pointcloud[idx][1] = y*Z;
      pointcloud[idx][2] = Z;
    }
  }

  // Read ground truth
  ss.str("");
  ss << input_directory << "/camera_poses/Camera_%03d.txt";
  sprintf(buffer, ss.str().c_str(), cpt);
  std::string filename_pose = buffer;

  std::ifstream f_pose;
  f_pose.open(filename_pose.c_str()); // .c_str() to keep compat when c++11 not available
  if (!f_pose.is_open()) {
    std::cerr << "Cannot read: " << filename_pose << std::endl;
    return false;
  }

  cMo_ground_truth.load(f_pose);

  return true;
}
}

int main(int argc, char *argv[])
{
  std::string input_directory = "."; // location of the data (images, depth maps, camera poses)
  std::string config_color = "teabox.xml", config_depth = "teabox_depth.xml";
  std::string model_color = "teabox.cao", model_depth = "teabox.cao";
  std::string init_file = "teabox.init";
  std::string extrinsic_file = "depth_M_color.txt";
  unsigned int first_frame_index = 1;
  bool disable_depth = false;
  bool display_ground_truth = false;
  bool click = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input_directory" && i + 1 < argc) {
      input_directory = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--config_color" && i + 1 < argc) {
      config_color = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--config_depth" && i + 1 < argc) {
      config_depth = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--model_color" && i + 1 < argc) {
      model_color = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--model_depth" && i + 1 < argc) {
      model_depth = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--init_file" && i + 1 < argc) {
      init_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--extrinsics" && i + 1 < argc) {
      extrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--disable_depth") {
      disable_depth = true;
    } else if (std::string(argv[i]) == "--display_ground_truth") {
      display_ground_truth = true;
    } else if (std::string(argv[i]) == "--click") {
      click = true;
    } else if (std::string(argv[i]) == "--first_frame_index" && i+1 < argc) {
      first_frame_index = static_cast<unsigned int>(atoi(argv[i+1]));
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: \n" << argv[0] << " [--input_directory <data directory> (default: .)]"
                   " [--config_color <object.xml> (default: teabox.xml)] [--config_depth <object.xml> (default: teabox_depth.xml)]"
                   " [--model_color <object.cao> (default: teabox.cao)] [--model_depth <object.cao> (default: teabox.cao)]"
                   " [--init_file <object.init> (default: teabox.init)]"
                   " [--extrinsics <depth to color transformation> (default: depth_M_color.txt)] [--disable_depth]"
                   " [--display_ground_truth] [--click] [--first_frame_index <index> (default: 1)]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "input_directory: " << input_directory << std::endl;
  std::cout << "config_color: " << config_color << std::endl;
  std::cout << "config_depth: " << config_depth << std::endl;
  std::cout << "model_color: " << model_color << std::endl;
  std::cout << "model_depth: " << model_depth << std::endl;
  std::cout << "init_file: " << model_depth << std::endl;
  std::cout << "extrinsic_file: " << extrinsic_file << std::endl;
  std::cout << "first_frame_index: " << first_frame_index << std::endl;
  std::cout << "disable_depth: " << disable_depth << std::endl;
  std::cout << "display_ground_truth: " << display_ground_truth << std::endl;
  std::cout << "click: " << click << std::endl;

  std::vector<int> tracker_types;
  tracker_types.push_back(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
  if (!disable_depth)
    tracker_types.push_back(vpMbGenericTracker::DEPTH_DENSE_TRACKER);

  vpMbGenericTracker tracker(tracker_types);
  if (!disable_depth)
    tracker.loadConfigFile(config_color, config_depth);
  else
    tracker.loadConfigFile(config_color);
  tracker.loadModel(model_color);
  vpCameraParameters cam_color, cam_depth;
  if (!disable_depth)
    tracker.getCameraParameters(cam_color, cam_depth);
  else
    tracker.getCameraParameters(cam_color);
  tracker.setDisplayFeatures(true);
  std::cout << "cam_color:\n" << cam_color << std::endl;
  std::cout << "cam_depth:\n" << cam_depth << std::endl;

  vpImage<uint16_t> I_depth_raw;
  vpImage<unsigned char> I, I_depth;
  unsigned int depth_width = 0, depth_height = 0;
  std::vector<vpColVector> pointcloud;
  vpHomogeneousMatrix cMo_ground_truth;

  unsigned int frame_cpt = first_frame_index;
  read_data(frame_cpt, input_directory, I, I_depth_raw, depth_width, depth_height, pointcloud, cam_depth, cMo_ground_truth);
  vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

#if defined(VISP_HAVE_X11)
  vpDisplayX d1, d2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d1, d2;
#else
  vpDisplayOpenCV d1, d2;
#endif

  d1.init(I, 0, 0, "Color image");
  d2.init(I_depth, static_cast<int>(I.getWidth()), 0, "Depth image");

  vpHomogeneousMatrix depthMcolor;
  if (!disable_depth) {
    std::ifstream f_extrinsics;
    f_extrinsics.open(extrinsic_file.c_str()); // .c_str() to keep compat when c++11 not available

    depthMcolor.load(f_extrinsics);
    tracker.setCameraTransformationMatrix("Camera2", depthMcolor);
    std::cout << "depthMcolor:\n" << depthMcolor << std::endl;
  }

  if (display_ground_truth) {
    tracker.initFromPose(I, cMo_ground_truth); //I and I_depth must be the same size when using depth features!
  } else
    tracker.initClick(I, init_file, true); //I and I_depth must be the same size when using depth features!

  try {
    bool quit = false;
    while (!quit && read_data(frame_cpt, input_directory, I, I_depth_raw, depth_width, depth_height, pointcloud, cam_depth, cMo_ground_truth)) {
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
      vpDisplay::display(I);
      vpDisplay::display(I_depth);

      if (display_ground_truth) {
        tracker.initFromPose(I, cMo_ground_truth); //I and I_depth must be the same size when using depth features!
      } else {
        if (!disable_depth) {
          std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
          std::map<std::string, const std::vector<vpColVector> *> mapOfPointClouds;
          std::map<std::string, unsigned int> mapOfPointCloudWidths;
          std::map<std::string, unsigned int> mapOfPointCloudHeights;

          mapOfImages["Camera1"] = &I;
          mapOfPointClouds["Camera2"] = &pointcloud;
          mapOfPointCloudWidths["Camera2"] = depth_width;
          mapOfPointCloudHeights["Camera2"] = depth_height;
          tracker.track(mapOfImages, mapOfPointClouds, mapOfPointCloudWidths, mapOfPointCloudHeights);
        } else
          tracker.track(I);
      }

      vpHomogeneousMatrix cMo = tracker.getPose();
      std::cout << "\nFrame: " << frame_cpt << std::endl;
      if (!display_ground_truth)
        std::cout << "cMo:\n" << cMo << std::endl;
      std::cout << "cMo ground truth:\n" << cMo_ground_truth << std::endl;
      if (!disable_depth) {
        tracker.display(I, I_depth, cMo, depthMcolor*cMo, cam_color, cam_depth, vpColor::red, 2);
        vpDisplay::displayFrame(I_depth, depthMcolor*cMo, cam_depth, 0.05, vpColor::none, 2);
      }
      else
        tracker.display(I, cMo, cam_color, vpColor::red, 2);

      vpDisplay::displayFrame(I, cMo, cam_color, 0.05, vpColor::none, 2);
      std::ostringstream oss;
      oss << "Frame: " << frame_cpt;
      vpDisplay::displayText(I, 20, 20, oss.str(), vpColor::red);

      if (!display_ground_truth) {
        oss.str("");
        oss << "Nb features: " << tracker.getError().getRows();
        vpDisplay::displayText(I, 40, 20, oss.str(), vpColor::red);
      }

      vpDisplay::flush(I);
      vpDisplay::flush(I_depth);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, click)) {
        switch (button) {
        case vpMouseButton::button1:
          quit = !click;
          break;
        case vpMouseButton::button3:
          click = !click;
          break;

        default:
          break;
        }
      }

      frame_cpt++;
    }

    vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);
  } catch (std::exception& e) {
    std::cerr << "Catch exception: " << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "To run this tutorial, ViSP should be built with OpenCV and libXML2 libraries." << std::endl;
  return EXIT_SUCCESS;
}
#endif
