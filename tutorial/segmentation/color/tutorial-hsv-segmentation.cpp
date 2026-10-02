//! \example tutorial-hsv-segmentation.cpp

#include <iostream>
#include <visp3/core/vpConfig.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpHSV.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpColorDepthConversion.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/sensor/vpRealSense2.h>

int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_DISPLAY)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_hsv_filename = "calib/hsv-thresholds.yml";
  std::string opt_video_filename;
  unsigned int opt_rs2_width = 640;
  unsigned int opt_rs2_height = 480;
  int opt_rs2_fps = 30;
  bool show_helper = false;

  for (int i = 1; i < argc; i++) {
    if ((std::string(argv[i]) == "--hsv-thresholds") && ((i+1) < argc)) {
      opt_hsv_filename = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--video") {
      if ((i+1) < argc) {
        opt_video_filename = std::string(argv[++i]);
      }
      else {
        show_helper = true;
        std::cout << "ERROR \nMissing input video name after parameter " << std::string(argv[i]) << std::endl;
      }
    }
    else if (((std::string(argv[i]) == "--rs2-width") || (std::string(argv[i]) == "-w")) && ((i+1) < argc)) {
      opt_rs2_width = static_cast<unsigned int>(std::atoi(argv[++i]));
    }
    else if (((std::string(argv[i]) == "--rs2-height") || (std::string(argv[i]) == "-h")) && ((i+1) < argc)) {
      opt_rs2_height = static_cast<unsigned int>(std::atoi(argv[++i]));
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
        << " [--video <input video>]"
        << " [--rs2-width,-w <width>]"
        << " [--rs2-height,-h <height>]"
        << " [--rs2-fps <framerate>]"
        << " [--hsv-thresholds <output filename.yml>]"
        << " [--help,-h]"
        << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --video <input video>" << std::endl
        << "    Name of the input video filename." << std::endl
        << "    When this option is set, we use the input video to tune HSV thresholds." << std::endl
        << "    Otherwise, we use librealsense to stream images from a Realsense camera. " << std::endl
        << "    When Realsense camera is used, you can specify image size with " << std::endl
        << "    --rs2-width and --rs2-height options." << std::endl
        << "    Example: --image image-%04d.jpg" << std::endl
        << std::endl
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
        << "  --help, -h" << std::endl
        << "    Display this helper message." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  bool use_realsense = false;
#if defined(VISP_HAVE_REALSENSE2)
  use_realsense = true;
#endif
  if (use_realsense) {
    if (!opt_video_filename.empty()) {
      use_realsense = false;
    }
  }
  else if (opt_video_filename.empty()) {
    std::cout << "Error: you should use --image <input image> option to specify an input image..." << std::endl;
    return EXIT_FAILURE;
  }

  if (use_realsense) {
    std::cout << "Use images from Realsense camera" << std::endl;
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

  vpImage<vpRGBa> I;

  vpVideoReader g;
#if defined(VISP_HAVE_REALSENSE2)
  vpRealSense2 rs;
#endif

  if (use_realsense) {
#if defined(VISP_HAVE_REALSENSE2)
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, static_cast<int>(opt_rs2_width), static_cast<int>(opt_rs2_height), RS2_FORMAT_RGBA8, opt_rs2_fps);
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED, 1);
    config.disable_stream(RS2_STREAM_INFRARED, 2);
    rs.open(config);
    rs.acquire(I);
#endif
  }
  else {
    try {
      g.setFileName(opt_video_filename);
      g.open(I);
    }
    catch (const vpException &e) {
      std::cout << e.getStringMessage() << std::endl;
      return EXIT_FAILURE;
    }
  }

  unsigned int width = I.getWidth();
  unsigned int height = I.getHeight();

  std::cout << "Image size: " << width << " x " << height << std::endl;

  vpImage<unsigned char> mask(height, width);
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

  while (!quit) {
    double t = vpTime::measureTimeMs();
    if (use_realsense) {
#if defined(VISP_HAVE_REALSENSE2)
      rs.acquire(I);
#endif
    }
    else {
      if (!g.end()) {
        g.acquire(I);
      }
    }
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
#else
  (void)argc;
  (void)argv;
  std::cout << "This tutorial needs X11 3rdparty that is not enabled" << std::endl;
#endif
  return EXIT_SUCCESS;
}
