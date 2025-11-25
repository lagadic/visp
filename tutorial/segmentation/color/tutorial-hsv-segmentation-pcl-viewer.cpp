//! \example tutorial-hsv-segmentation-pcl-viewer.cpp

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION) && defined(VISP_HAVE_THREADS) && defined(VISP_HAVE_DISPLAY)
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpColorDepthConversion.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
//! [Include vpDisplayPCL header]
#include <visp3/gui/vpDisplayPCL.h>
//! [Include vpDisplayPCL header]
#include <visp3/sensor/vpRealSense2.h>

//! [Enum for mode choice]
/**
 * @brief Enumeration permitting to choose between running the blocking-mode display
 * example, the threaded-mode or both mode consecutively.
 */
typedef enum DisplayMode
{
  MONOTHREAD = 0, /*!< Only the monothread-mode display example will be run.*/
  THREADED = 1, /*!< Only the threaded-mode display example will be run.*/
  MODE_COUNT = 2
} DisplayMode;

/**
 * @brief Cast a \b DisplayMode enum value into a \b std::stirng.
 *
 * @param mode The display mode we want to cast into a string.
 * @return std::string The name of the \b DisplayMode enum value.
 */
std::string displayModeToString(const DisplayMode &mode)
{
  switch (mode) {
  case MONOTHREAD:
    return "monothread";
  case THREADED:
    return "threaded";
  default:
    break;
  }
  return "unknown";
}

/**
 * @brief Cast a string into a \b DisplayMode enum value.
 * If \b name is not found, return \b DisplayMode::MODE_COUNT .
 *
 * @param name The name of the display mode.
 * @return DisplayMode The corresponding \b DisplayMode enum value, or \b DisplayMode::MODE_COUNT if not found.
 */
DisplayMode displayModeFromString(const std::string &name)
{
  DisplayMode res = DisplayMode::MODE_COUNT;
  bool wasFound = false;
  std::string lowerCaseName = vpIoTools::toLowerCase(name);
  unsigned int i = 0;
  while ((i < DisplayMode::MODE_COUNT) && (!wasFound)) {
    DisplayMode candidate = (DisplayMode)i;
    if (lowerCaseName == displayModeToString(candidate)) {
      res = candidate;
      wasFound = true;
    }
    ++i;
  }
  return res;
}

/**
 * @brief Create a string that lists the different \b DisplayMode available.
 *
 * @param prefix The string that must prefix the list of modes.
 * @param sep The separator between the different modes.
 * @param suffix The string that must suffix the list of modes.
 * @return std::string The list containing the different modes.
 */
std::string getAvailableDisplayMode(const std::string &prefix = "< ", const std::string &sep = " , ", const std::string &suffix = " >")
{
  std::string modes(prefix);
  for (unsigned int i = 0; i < DisplayMode::MODE_COUNT - 1; ++i) {
    DisplayMode candidate = (DisplayMode)i;
    modes += displayModeToString(candidate) + sep;
  }
  DisplayMode candidate = (DisplayMode)(DisplayMode::MODE_COUNT - 1);
  modes += displayModeToString(candidate) + suffix;
  return modes;
}
//! [Enum for mode choice]

int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_hsv_filename = "calib/hsv-thresholds.yml";
  bool opt_pcl_textured = false;
  bool opt_verbose = false;
  int opt_width = 848;
  int opt_height = 480;
  int opt_fps = 60;
  DisplayMode opt_mode = DisplayMode::MONOTHREAD;

  int i = 1;
  while (i < argc) {
    if (((std::string(argv[i]) == "--width") || (std::string(argv[i]) == "-v")) && ((i+1) < argc)) {
      opt_width = std::atoi(argv[++i]);
    }
    else if (((std::string(argv[i]) == "--height") || (std::string(argv[i]) == "-h")) && ((i+1) < argc)) {
      opt_height = std::atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--display-mode" && i + 1 < argc) {
      opt_mode = displayModeFromString(std::string(argv[i + 1]));
      ++i;
    }
    else if ((std::string(argv[i]) == "--fps") && ((i+1) < argc)) {
      opt_fps = std::atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--texture") {
      opt_pcl_textured = true;
    }
    else if ((std::string(argv[i]) == "--hsv-thresholds") && ((i+1) < argc)) {
      opt_hsv_filename = std::string(argv[++i]);
    }
    else if ((std::string(argv[i]) == "--verbose")  || (std::string(argv[i]) == "-v")) {
      opt_verbose = true;
    }
    else if ((std::string(argv[i]) == "--help") || (std::string(argv[i]) == "-h")) {
      std::cout << "\nSYNOPSIS " << std::endl
        << argv[0]
        << " [--width,-w <image width>]"
        << " [--height,-h <image height>]"
        << " [--fps <framerate>]"
        << " [--texture]"
        << " [--hsv-thresholds <filename.yml>]"
        << " [--verbose,-v]"
        << " [--help,-h]"
        << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --width,-w <image width>" << std::endl
        << "    Realsense camera image width." << std::endl
        << "    Default: " << opt_width << std::endl
        << std::endl
        << "  --height,-h <image height>" << std::endl
        << "    Realsense camera image height." << std::endl
        << "    Default: " << opt_height << std::endl
        << std::endl
        << "  --fps <framerate>" << std::endl
        << "    Realsense camera framerate." << std::endl
        << "    Default: " << opt_fps << std::endl
        << std::endl
        << "  --texture" << std::endl
        << "    Enable textured point cloud adding RGB information to the 3D point." << std::endl
        << std::endl
        << "  --hsv-thresholds <filename.yaml>" << std::endl
        << "    Path to a yaml filename that contains H <min,max>, S <min,max>, V <min,max> threshold values." << std::endl
        << "    An Example of such a file could be:" << std::endl
        << "      rows: 6" << std::endl
        << "      cols: 1" << std::endl
        << "      data:" << std::endl
        << "        - [0]" << std::endl
        << "        - [42]" << std::endl
        << "        - [177]" << std::endl
        << "        - [237]" << std::endl
        << "        - [148]" << std::endl
        << "        - [208]" << std::endl
        << std::endl
        << "  --verbose, -v" << std::endl
        << "    Enable verbose mode." << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Display this helper message." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
    ++i;
  }

  vpColVector hsv_values;
  if (vpColVector::loadYAML(opt_hsv_filename, hsv_values)) {
    std::cout << "Load HSV threshold values from " << opt_hsv_filename << std::endl;
    std::cout << "HSV low/high values: " << hsv_values.t() << std::endl;
  }
  else {
    std::cout << "Warning: unable to load HSV thresholds values from " << opt_hsv_filename << std::endl;
    return EXIT_FAILURE;
  }

  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, opt_width, opt_height, RS2_FORMAT_RGBA8, opt_fps);
  config.enable_stream(RS2_STREAM_DEPTH, opt_width, opt_height, RS2_FORMAT_Z16, opt_fps);
  config.disable_stream(RS2_STREAM_INFRARED, 1);
  config.disable_stream(RS2_STREAM_INFRARED, 2);
  rs2::align align_to(RS2_STREAM_COLOR);

  rs.open(config);

  float depth_scale = rs.getDepthScale();
  vpCameraParameters cam_depth = rs.getCameraParameters(RS2_STREAM_DEPTH,
                                                        vpCameraParameters::perspectiveProjWithoutDistortion);

  vpImage<vpRGBa> I(opt_height, opt_width);
  vpImage<unsigned char> mask(opt_height, opt_width);
  vpImage<uint16_t> depth_raw(opt_height, opt_width);
  vpImage<vpRGBa> I_segmented(opt_height, opt_width);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpImage<vpHSV<unsigned char, true>> Ihsv;

  std::shared_ptr<vpDisplay> d_I = vpDisplayFactory::createDisplay(I, 0, 0, "Current frame");
  std::shared_ptr<vpDisplay> d_I_segmented = vpDisplayFactory::createDisplay(I_segmented, I.getWidth()+75, 0, "HSV segmented frame");
#else
  vpImage<unsigned char> H(opt_height, opt_width);
  vpImage<unsigned char> S(opt_height, opt_width);
  vpImage<unsigned char> V(opt_height, opt_width);

  vpDisplay *d_I = vpDisplayFactory::allocateDisplay(I, 0, 0, "Current frame");
  vpDisplay *d_I_segmented = vpDisplayFactory::allocateDisplay(I_segmented, I.getWidth()+75, 0, "HSV segmented frame");
#endif

  bool quit = false;
  double loop_time = 0., total_loop_time = 0.;
  long nb_iter = 0;
  float Z_min = 0.1;
  float Z_max = 2.5;

  //! [Create point cloud]
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_color(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  //! [Create point cloud]

  //! [Create pcl viewer object]
  std::mutex pointcloud_mutex;
  vpDisplayPCL pcl_viewer(opt_width, opt_height);
  if (opt_mode == DisplayMode::THREADED) {
    if (opt_pcl_textured) {
      pcl_viewer.startThread(std::ref(pointcloud_mutex), pointcloud_color);
    }
    else {
      pcl_viewer.startThread(std::ref(pointcloud_mutex), pointcloud);
    }
  }
  else {
    if (opt_pcl_textured) {
      pcl_viewer.addPointCloud(std::ref(pointcloud_mutex), pointcloud_color);
    }
    else {
      pcl_viewer.addPointCloud(std::ref(pointcloud_mutex), pointcloud);
    }
  }
 //! [Create pcl viewer object]

  while (!quit) {
    double t = vpTime::measureTimeMs();
    rs.acquire((unsigned char *)I.bitmap, (unsigned char *)(depth_raw.bitmap), NULL, NULL, &align_to);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    vpImageConvert::convert(I, Ihsv);
    vpImageTools::inRange(Ihsv, hsv_values, mask);
#else
    vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap),
                              reinterpret_cast<unsigned char *>(H.bitmap),
                              reinterpret_cast<unsigned char *>(S.bitmap),
                              reinterpret_cast<unsigned char *>(V.bitmap), I.getSize());

    vpImageTools::inRange(reinterpret_cast<unsigned char *>(H.bitmap),
                          reinterpret_cast<unsigned char *>(S.bitmap),
                          reinterpret_cast<unsigned char *>(V.bitmap),
                          hsv_values,
                          reinterpret_cast<unsigned char *>(mask.bitmap),
                          mask.getSize());
#endif

    vpImageTools::inMask(I, mask, I_segmented);

    //! [Update point cloud with mutex protection]
    int pcl_size;
    if (opt_pcl_textured) {
      pcl_size = vpImageConvert::depthToPointCloud(I, depth_raw, depth_scale, cam_depth, pointcloud_color, &pointcloud_mutex, &mask, Z_min, Z_max);
    }
    else {
      pcl_size = vpImageConvert::depthToPointCloud(depth_raw, depth_scale, cam_depth, pointcloud, &pointcloud_mutex, &mask, Z_min, Z_max);
    }
    if (opt_verbose) {
      std::cout << "Segmented point cloud size: " << pcl_size << std::endl;
    }
    //! [Update point cloud with mutex protection]

    vpDisplay::display(I);
    vpDisplay::display(I_segmented);
    vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);

    if (vpDisplay::getClick(I, false)) {
      quit = true;
    }

    //! [Refresh monothread viewer]
    if (opt_mode ==  DisplayMode::MONOTHREAD) {
      const bool blocking_mode = false;
      pcl_viewer.display(blocking_mode);
    }
    //! [Refresh monothread viewer]

    vpDisplay::flush(I);
    vpDisplay::flush(I_segmented);
    nb_iter++;
    loop_time = vpTime::measureTimeMs() - t;
    total_loop_time += loop_time;
  }

  std::cout << "Mean loop time: " << total_loop_time / nb_iter << std::endl;

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (d_I != nullptr) {
    delete d_I;
  }

  if (d_I_segmented != nullptr) {
    delete d_I_segmented;
  }
#endif
  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "This tutorial needs librealsense as 3rd party." << std::endl;
#endif
#if !defined(VISP_HAVE_PCL)
  std::cout << "This tutorial needs pcl library as 3rd party." << std::endl;
#endif
#if !defined(VISP_HAVE_PCL_VISUALIZATION)
  std::cout << "This tutorial needs pcl visualization module." << std::endl;
#endif
#if !defined(VISP_HAVE_X11)
  std::cout << "This tutorial needs X11 3rd party enabled." << std::endl;
#endif
  std::cout << "Install missing 3rd party, configure and rebuild ViSP." << std::endl;
  return EXIT_SUCCESS;
}
#endif
