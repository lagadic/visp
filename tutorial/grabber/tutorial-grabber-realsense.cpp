/*! \example tutorial-grabber-realsense.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--fps <6|15|30|60>]"
    << " [--width <image width>]"
    << " [--height <image height>]"
    << " [--seqname <sequence name>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --fps <6|15|30|60>" << std::endl
    << "    Frames per second." << std::endl
    << "    Default: 30." << std::endl
    << std::endl
    << "  --width <image width>" << std::endl
    << "    Default: 640." << std::endl
    << std::endl
    << "  --height <image height>" << std::endl
    << "    Default: 480." << std::endl
    << std::endl
    << "  --seqname <sequence name>" << std::endl
    << "    Name of the sequence of image to create (ie: /tmp/image%04d.jpg)." << std::endl
    << "    Default: empty." << std::endl
    << std::endl
    << "  --record <mode>" << std::endl
    << "    Allowed values for mode are:" << std::endl
    << "      0: record all the captures images (continuous mode)," << std::endl
    << "      1: record only images selected by a user click (single shot mode)." << std::endl
    << "    Default mode: 0" << std::endl
    << std::endl
    << "  --no-display" << std::endl
    << "    Disable displaying captured images." << std::endl
    << "    When used and sequence name specified, record mode is internally set to 1 (continuous mode)."
    << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  std::cout << "USAGE" << std::endl
    << "  Example to visualize images:" << std::endl
    << "    " << argv[0] << std::endl
    << std::endl
    << "  Examples to record a sequence of successive images in 640x480 resolution:" << std::endl
    << "    " << argv[0] << " --seqname I%04d.png" << std::endl
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 0" << std::endl
    << std::endl
    << "  Examples to record single shot 640x480 images:\n"
    << "    " << argv[0] << " --seqname I%04d.png --record 1\n"
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 1" << std::endl
    << std::endl
    << "  Examples to record single shot 1280x720 images:\n"
    << "    " << argv[0] << " --seqname I%04d.png --record 1 --width 1280 --height 720" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

/*!
  Grab images from an Intel realsense camera
 */
int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_THREADS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    std::string opt_seqname;
    int opt_record_mode = 0;
    int opt_fps = 30;
    bool opt_display = true;
    unsigned int opt_width = 640;
    unsigned int opt_height = 480;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--fps") {
        opt_fps = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--seqname") {
        opt_seqname = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--width") {
        opt_width = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--height") {
        opt_height = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--record") {
        opt_record_mode = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--no-display") {
        opt_display = false;
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        usage(argv, 0);
        return EXIT_SUCCESS;
      }
      else {
        usage(argv, i);
        return EXIT_FAILURE;
      }
    }

    if ((!opt_display) && (!opt_seqname.empty())) {
      opt_record_mode = 0;
    }

    if (opt_fps != 6 && opt_fps != 15 && opt_fps != 30 && opt_fps != 60) {
      opt_fps = 30; // Default
    }
    std::cout << "Resolution : " << opt_width << " x " << opt_height << std::endl;
    std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
    std::cout << "Framerate  : " << opt_fps << std::endl;
    std::cout << "Display    : " << (opt_display ? "enabled" : "disabled") << std::endl;

    std::string text_record_mode =
      std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (!opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }
    vpImage<vpRGBa> I;

#ifdef VISP_HAVE_REALSENSE2
    std::cout << "SDK        : Realsense 2" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, opt_width, opt_height, RS2_FORMAT_RGBA8, opt_fps);
    g.open(config);
#else
    std::cout << "SDK        : Realsense 1" << std::endl;
    vpRealSense g;
    g.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(opt_width, opt_height, rs::format::rgba8, 60));
    g.open();
#endif
    g.acquire(I);

    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;

    vpCameraParameters cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithoutDistortion);
    vpXmlParserCamera p;
    std::string output_folder = vpIoTools::getParent(opt_seqname);
    if (!vpIoTools::checkDirectory(output_folder)) {
      try {
        std::cout << "Create output folder: " << output_folder << std::endl;
        vpIoTools::makeDirectory(output_folder);
      }
      catch (const vpException &e) {
        std::cout << e.getStringMessage();
        return EXIT_FAILURE;
      }
    }
    std::string cam_filename = output_folder + "/camera.xml";

    std::cout << "Save camera intrinsics in: " << cam_filename << std::endl;
    if (p.save(cam, cam_filename, "camera")) {
      std::cout << "Cannot save camera parameters in " << cam_filename << std::endl;
    }

    vpDisplay *d = nullptr;
    if (opt_display) {
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << "No image viewer is available..." << std::endl;
      opt_display = false;
#endif
    }
    if (opt_display) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
      d = new vpDisplayOpenCV(I);
#endif
    }

    vpImageQueue<vpRGBa> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<vpRGBa> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<vpRGBa>::run, &image_storage_worker);

    bool quit = false;
    while (!quit) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);

      vpDisplay::display(I);

      quit = image_queue.record(I);

      std::stringstream ss;
      ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
      vpDisplay::displayText(I, I.getHeight() - 20, 10, ss.str(), vpColor::red);
      vpDisplay::flush(I);
    }
    image_queue.cancel();
    image_storage_thread.join();

    if (d) {
      delete d;
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#if !(defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2))
  std::cout << "Install librealsense version > 2.31.0, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
