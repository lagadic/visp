//! \example tutorial-mb-generic-tracker-rgbd.cpp
#include <iostream>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
//! [Include]
#include <visp3/mbt/vpMbGenericTracker.h>
//! [Include]

#if defined (VISP_HAVE_PCL)
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

struct rs_intrinsics {
  float ppx;       /**< Horizontal coordinate of the principal point of the image,
                      as a pixel offset from the left edge */
  float ppy;       /**< Vertical coordinate of the principal point of the image, as
                      a pixel offset from the top edge */
  float fx;        /**< Focal length of the image plane, as a multiple of pixel width
                    */
  float fy;        /**< Focal length of the image plane, as a multiple of pixel
                      height */
  float coeffs[5]; /**< Distortion coefficients */
};

void rs_deproject_pixel_to_point(float point[3], const rs_intrinsics &intrin, const float pixel[2], float depth) {
  float x = (pixel[0] - intrin.ppx) / intrin.fx;
  float y = (pixel[1] - intrin.ppy) / intrin.fy;

  float r2 = x * x + y * y;
  float f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2;
  float ux = x * f + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x);
  float uy = y * f + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y);

  x = ux;
  y = uy;

  point[0] = depth * x;
  point[1] = depth * y;
  point[2] = depth;
}

//! [Read data function]
bool read_data(unsigned int cpt, const std::string &input_directory, vpImage<vpRGBa> &I_color,
               vpImage<uint16_t> &I_depth_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
//! [Read data function]
{
  char buffer[FILENAME_MAX];
  // Read color
  std::stringstream ss;
  ss << input_directory << "/color_image_%04d.jpg";
  sprintf(buffer, ss.str().c_str(), cpt);
  std::string filename_color = buffer;

  if (!vpIoTools::checkFilename(filename_color)) {
    std::cerr << "Cannot read: " << filename_color << std::endl;
    return false;
  }
  vpImageIo::read(I_color, filename_color);

  // Read raw depth
  ss.str("");
  ss << input_directory << "/depth_image_%04d.bin";
  sprintf(buffer, ss.str().c_str(), cpt);
  std::string filename_depth = buffer;

  std::ifstream file_depth(filename_depth.c_str(), std::ios::in | std::ios::binary);
  if (!file_depth.is_open()) {
    return false;
  }

  unsigned int height = 0, width = 0;
  vpIoTools::readBinaryValueLE(file_depth, height);
  vpIoTools::readBinaryValueLE(file_depth, width);

  I_depth_raw.resize(height, width);

  uint16_t depth_value = 0;
  for (unsigned int i = 0; i < height; i++) {
    for (unsigned int j = 0; j < width; j++) {
      vpIoTools::readBinaryValueLE(file_depth, depth_value);
      I_depth_raw[i][j] = depth_value;
    }
  }

  // Transform pointcloud
  pointcloud->width = width;
  pointcloud->height = height;
  pointcloud->reserve((size_t)width * height);

  // Only for Creative SR300
  const float depth_scale = 0.00100000005f;
  rs_intrinsics depth_intrinsic;
  depth_intrinsic.ppx = 320.503509521484f;
  depth_intrinsic.ppy = 235.602951049805f;
  depth_intrinsic.fx = 383.970001220703f;
  depth_intrinsic.fy = 383.970001220703f;
  depth_intrinsic.coeffs[0] = 0.0f;
  depth_intrinsic.coeffs[1] = 0.0f;
  depth_intrinsic.coeffs[2] = 0.0f;
  depth_intrinsic.coeffs[3] = 0.0f;
  depth_intrinsic.coeffs[4] = 0.0f;

  for (unsigned int i = 0; i < height; i++) {
    for (unsigned int j = 0; j < width; j++) {
      float scaled_depth = I_depth_raw[i][j] * depth_scale;
      float point[3];
      float pixel[2] = {(float)j, (float)i};
      rs_deproject_pixel_to_point(point, depth_intrinsic, pixel, scaled_depth);
      pointcloud->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }
  }

  return true;
}

int main(int argc, char *argv[])
{
  std::string input_directory = "."; // location of the data (images, depth_map, point_cloud)
  std::string config_color = "cube.xml", config_depth = "cube_depth.xml";
  std::string model_color = "cube.cao", model_depth = "cube.cao";
  std::string init_file = "cube.init";
  int frame_cpt = 0;
  bool disable_depth = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--input_directory" && i+1 < argc) {
      input_directory = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--config_color" && i+1 < argc) {
      config_color = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--config_depth" && i+1 < argc) {
      config_depth = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--model_color" && i+1 < argc) {
      model_color = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--model_depth" && i+1 < argc) {
      model_depth = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--init_file" && i+1 < argc) {
      init_file = std::string(argv[i+1]);
    } else if (std::string(argv[i]) == "--disable_depth") {
      disable_depth = true;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: \n" << argv[0] << " --input_directory <data directory> --config_color <object.xml> --config_depth <object.xml>"
                                             " --model_color <object.cao> --model_depth <object.cao> --init_file <object.init> --disable_depth" << std::endl;
      std::cout << "\nExample:\n" << argv[0] << " --config_color ../model/cube/cube.xml --config_depth ../model/cube/cube.xml"
                                                " --model_color ../model/cube/cube.cao --model_depth ../model/cube/cube.cao --init_file ../model/cube/cube.init\n" << std::endl;
      return 0;
    }
  }

  vpImage<vpRGBa> I_color;
  //! [Images]
  vpImage<unsigned char> I_gray, I_depth;
  //! [Images]
  vpImage<uint16_t> I_depth_raw;
  //! [Point cloud]
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  //! [Point cloud]
  vpCameraParameters cam_color, cam_depth;
  unsigned int _posx = 100, _posy = 50, _posdx = 10;

  read_data(frame_cpt, input_directory, I_color, I_depth_raw, pointcloud);
  vpImageConvert::convert(I_color, I_gray);
  vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

#if defined(VISP_HAVE_X11)
  vpDisplayX d1, d2;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d1, d2;
#endif
  d1.init(I_gray, _posx, _posy, "Color stream");
  d2.init(I_depth, _posx + I_gray.getWidth()+_posdx, _posy, "Depth stream");
  vpDisplay::display(I_gray);
  vpDisplay::display(I_depth);
  vpDisplay::flush(I_gray);
  vpDisplay::flush(I_depth);

  //! [Constructor]
#ifdef VISP_HAVE_OPENCV
  std::vector<int> trackerTypes = {vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER, vpMbGenericTracker::DEPTH_DENSE_TRACKER};
#else
  std::vector<int> trackerTypes = {vpMbGenericTracker::EDGE_TRACKER};
#endif
  vpMbGenericTracker tracker(trackerTypes);
  //! [Constructor]
  //! [Load config file]
  tracker.loadConfigFile(config_color, config_depth);
  //! [Load config file]
  //! [Load cao]
  tracker.loadModel(model_color, model_depth);
  //! [Load cao]

  tracker.getCameraParameters(cam_color, cam_depth);

  std::cout << "Camera parameters for color camera (from XML file): " << cam_color << std::endl;
  std::cout << "Camera parameters for depth camera (from XML file): " << cam_depth << std::endl;

  //! [Set display features]
  tracker.setDisplayFeatures(true);
  //! [Set display features]

  //! [Map transformations]
  vpHomogeneousMatrix depth_M_color;
  {
    std::ifstream file( std::string(input_directory + "/depth_M_color.txt") );
    depth_M_color.load(file);
    file.close();
  }
  std::map<std::string, vpHomogeneousMatrix> mapOfCameraTransformations;
  mapOfCameraTransformations["Camera1"] = vpHomogeneousMatrix();
  mapOfCameraTransformations["Camera2"] = depth_M_color;
  tracker.setCameraTransformationMatrix(mapOfCameraTransformations);
  //! [Map transformations]
  std::cout << "depth_M_color: \n" << depth_M_color << std::endl;

  //! [Map images]
  std::map<std::string, const vpImage<unsigned char> *> mapOfImages;
  mapOfImages["Camera1"] = &I_gray;
  mapOfImages["Camera2"] = &I_depth;
  //! [Map images]

  //! [Map init]
  std::map<std::string, std::string> mapOfInitFiles;
  mapOfInitFiles["Camera1"] = init_file;
  tracker.initClick(mapOfImages, mapOfInitFiles, true);
  //! [Map init]

  mapOfImages.clear();
  pcl::PointCloud<pcl::PointXYZ>::Ptr empty_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<double> times_vec;

  try {
    bool quit = false;
    while (! quit) {
      double t = vpTime::measureTimeMs();
      read_data(frame_cpt, input_directory, I_color, I_depth_raw, pointcloud);
      vpImageConvert::convert(I_color, I_gray);
      vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);

      vpDisplay::display(I_gray);
      vpDisplay::display(I_depth);

      mapOfImages["Camera1"] = &I_gray;
      std::map<std::string, pcl::PointCloud< pcl::PointXYZ >::ConstPtr> mapOfPointclouds;
      if (disable_depth)
        mapOfPointclouds["Camera2"] = empty_pointcloud;
      else
        mapOfPointclouds["Camera2"] = pointcloud;

      //! [Track]
      tracker.track(mapOfImages, mapOfPointclouds);
      //! [Track]

      //! [Get pose]
      vpHomogeneousMatrix cMo = tracker.getPose();
      //! [Get pose]

      std::cout << "iter: " << frame_cpt << " cMo:\n" << cMo << std::endl;

      //! [Display]
      tracker.display(I_gray, I_depth, cMo, depth_M_color*cMo, cam_color, cam_depth, vpColor::red, 3);
      vpDisplay::displayFrame(I_gray, cMo, cam_color, 0.05, vpColor::none, 3);
      vpDisplay::displayFrame(I_depth, depth_M_color*cMo, cam_depth, 0.05, vpColor::none, 3);
      //! [Display]

      t = vpTime::measureTimeMs() - t;
      times_vec.push_back(t);

      std::stringstream ss;
      ss << "Computation time: " << t << " ms";
      vpDisplay::displayText(I_gray, 20, 20, ss.str(), vpColor::red);

      vpDisplay::flush(I_gray);
      vpDisplay::flush(I_depth);

      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I_gray, button, false)) {
        quit = true;
      }

      frame_cpt ++;
    }
  } catch (const vpException &e) {
    std::cout << "Catch exception: " << e.getStringMessage() << std::endl;
  }

  std::cout << "\nProcessing time, Mean: " << vpMath::getMean(times_vec) << " ms ; Median: " << vpMath::getMedian(times_vec)
            << " ; Std: " << vpMath::getStdev(times_vec) << " ms" << std::endl;

  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "To run this tutorial, ViSP should be build with PCL library."
               " Install libpcl, configure and build again ViSP..." << std::endl;
  return EXIT_SUCCESS;
}
#endif
