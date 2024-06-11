//! \example tutorial-hsv-segmentation.cpp

#include <iostream>
#include <visp3/core/vpConfig.h>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpColorDepthConversion.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/sensor/vpRealSense2.h>

int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_X11)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_hsv_filename = "calib/hsv-thresholds.yml";
  std::string opt_video_filename;
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
    else if (show_helper || std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nSYNOPSIS " << std::endl
        << argv[0]
        << " [--video <input video>]"
        << " [--hsv-thresholds <filename.yml>]"
        << " [--help,-h]"
        << std::endl;
      std::cout << "\nOPTIONS " << std::endl
        << "  --video <input video>" << std::endl
        << "    Name of the input video filename." << std::endl
        << "    When this option is not set, we use librealsense to stream images from a Realsense camera. " << std::endl
        << "    Example: --video image-%04d.jpg" << std::endl
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
  int width = 848;
  int height = 480;

  vpVideoReader g;
#if defined(VISP_HAVE_REALSENSE2)
  vpRealSense2 rs;
#endif

  if (use_realsense) {
#if defined(VISP_HAVE_REALSENSE2)
    int fps = 60;
    rs2::config config;
    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, fps);
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
    width = I.getWidth();
    height = I.getHeight();
  }

  vpImage<unsigned char> H(height, width);
  vpImage<unsigned char> S(height, width);
  vpImage<unsigned char> V(height, width);
  vpImage<unsigned char> mask(height, width);
  vpImage<vpRGBa> I_segmented(height, width);

  vpDisplayX d_I(I, 0, 0, "Current frame");
  vpDisplayX d_I_segmented(I_segmented, I.getWidth()+75, 0, "HSV segmented frame");

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
#else
  (void)argc;
  (void)argv;
  std::cout << "This tutorial needs X11 3rdparty that is not enabled" << std::endl;
#endif
  return EXIT_SUCCESS;
}
