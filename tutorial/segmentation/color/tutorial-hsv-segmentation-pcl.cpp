//! \example tutorial-hsv-segmentation-pcl.cpp

#include <iostream>
#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_COMMON) && defined(VISP_HAVE_DISPLAY)
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpColorDepthConversion.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/sensor/vpRealSense2.h>

int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_hsv_filename = "calib/hsv-thresholds.yml";
  unsigned int opt_rs2_width = 640;
  unsigned int opt_rs2_height = 480;
  int opt_rs2_fps = 30;
  bool show_helper = false;

  for (int i = 1; i < argc; i++) {
    if ((std::string(argv[i]) == "--hsv-thresholds") && ((i+1) < argc)) {
      opt_hsv_filename = std::string(argv[++i]);
    }
    else if (((std::string(argv[i]) == "--rs2-width") || (std::string(argv[i]) == "-w")) && ((i+1) < argc)) {
      opt_rs2_width = static_cast<unsigned int>(std::stoi(argv[++i]));
    }
    else if (((std::string(argv[i]) == "--rs2-height") || (std::string(argv[i]) == "-h")) && ((i+1) < argc)) {
      opt_rs2_height = static_cast<unsigned int>(std::stoi(argv[++i]));
    }
    else if ((std::string(argv[i]) == "--rs2-fps") && ((i+1) < argc)) {
      opt_rs2_fps = std::atoi(argv[++i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      show_helper = true;
    }
    else {
      std::cout << "Unknown option: " << argv[i] << std::endl;
      show_helper = true;
    }
    if (show_helper) {
      std::cout << "\nSYNOPSIS " << std::endl
        << argv[0]
        << " [--hsv-thresholds <filename.yml>]"
        << " [--rs2-width,-w <width>]"
        << " [--rs2-height,-h <height>]"
        << " [--rs2-fps <framerate>]"
        << " [--help,-h]"
        << std::endl;
      std::cout << "\nOPTIONS " << std::endl
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
        << "  --rs2-width,-w <width>" << std::endl
        << "    Width of the image stream from Realsense camera." << std::endl
        << "    Default: " << opt_rs2_width << std::endl
        << std::endl
        << "  --rs2-height,-h <height>" << std::endl
        << "    Height of the image stream from Realsense camera." << std::endl
        << "    Default: " << opt_rs2_height << std::endl
        << std::endl
        << "  --rs2-fps <framerate>" << std::endl
        << "    Realsense camera framerate." << std::endl
        << "    Default: " << opt_rs2_fps << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Display this helper message." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
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

  //! [Config RS2 RGB and depth]
  vpRealSense2 rs;
  rs2::config config;
  config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(opt_rs2_width), static_cast<int>(opt_rs2_height), RS2_FORMAT_RGBA8, opt_rs2_fps);
  config.enable_stream(RS2_STREAM_DEPTH, static_cast<int>(opt_rs2_width), static_cast<int>(opt_rs2_height), RS2_FORMAT_Z16, opt_rs2_fps);
  config.disable_stream(RS2_STREAM_INFRARED, 1);
  config.disable_stream(RS2_STREAM_INFRARED, 2);
  rs2::align align_to(RS2_STREAM_COLOR);
  //! [Config RS2 RGB and depth]

  rs.open(config);

  //! [Get RS2 intrinsics]
  float depth_scale = rs.getDepthScale();
  vpCameraParameters cam_depth = rs.getCameraParameters(RS2_STREAM_DEPTH,
                                                        vpCameraParameters::perspectiveProjWithoutDistortion);
  //! [Get RS2 intrinsics]


  vpImage<vpRGBa> I;
  rs.acquire(I);

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  std::cout << "Image size: " << width << " x " << height << std::endl;

  vpImage<unsigned char> mask(height, width, 0);
  vpImage<uint16_t> depth_raw(height, width);
  vpImage<vpRGBa> I_segmented(height, width);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpImage<vpHSV<unsigned char, true>> Ihsv;

  std::shared_ptr<vpDisplay> d_I = vpDisplayFactory::createDisplay(I, 0, 0, "Current frame");
  std::shared_ptr<vpDisplay> d_I_segmented = vpDisplayFactory::createDisplay(I_segmented, static_cast<int>(I.getWidth())+75, 0, "HSV segmented frame");
#else
  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);

  vpDisplay *d_I = vpDisplayFactory::allocateDisplay(I, 0, 0, "Current frame");
  vpDisplay *d_I_segmented = vpDisplayFactory::allocateDisplay(I_segmented, static_cast<int>(I.getWidth())+75, 0, "HSV segmented frame");
#endif

  bool quit = false;
  double loop_time = 0., total_loop_time = 0.;
  long nb_iter = 0;

  //! [Allocate point cloud]
  float Z_min = 0.1;
  float Z_max = 2.5;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  //! [Allocate point cloud]

  while (!quit) {
    double t = vpTime::measureTimeMs();
    //! [Grab color and depth]
    rs.acquire((unsigned char *)I.bitmap, (unsigned char *)(depth_raw.bitmap), NULL, NULL, &align_to);
    //! [Grab color and depth]

    //! [RGB to HSV]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    vpImageConvert::convert(I, Ihsv);
#else
    vpImageConvert::RGBaToHSV(reinterpret_cast<unsigned char *>(I.bitmap),
                              reinterpret_cast<unsigned char *>(H.bitmap),
                              reinterpret_cast<unsigned char *>(S.bitmap),
                              reinterpret_cast<unsigned char *>(V.bitmap), I.getSize());
#endif
    //! [RGB to HSV]

    //! [Create mask]
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    vpImageTools::inRange(Ihsv, hsv_values, mask);
#else
    vpImageTools::inRange(reinterpret_cast<unsigned char *>(H.bitmap),
                          reinterpret_cast<unsigned char *>(S.bitmap),
                          reinterpret_cast<unsigned char *>(V.bitmap),
                          hsv_values,
                          reinterpret_cast<unsigned char *>(mask.bitmap),
                          mask.getSize());
#endif
    //! [Create mask]

    vpImageTools::inMask(I, mask, I_segmented);

    //! [Update point cloud]
    vpImageConvert::depthToPointCloud(depth_raw, depth_scale, cam_depth, pointcloud, nullptr, &mask, Z_min, Z_max);
    //! [Update point cloud]

    //! [Get point cloud size]
    int pcl_size = pointcloud->size();
    //! [Get point cloud size]

    std::cout << "Segmented point cloud size: " << pcl_size << std::endl;

    vpDisplay::display(I);
    vpDisplay::display(I_segmented);
    vpDisplay::displayText(I, 20, 20, "Click to quit...", vpColor::red);

    if (vpDisplay::getClick(I, false)) {
      quit = true;
    }

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
#if !defined(VISP_HAVE_X11)
  std::cout << "This tutorial needs X11 3rd party enabled." << std::endl;
#endif
  std::cout << "Install missing 3rd party, configure and rebuild ViSP." << std::endl;
  return EXIT_SUCCESS;
}
#endif
