/*! \example tutorial-grabber-realsense-T265.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/sensor/vpRealSense2.h>

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--fps <6|15|30|60>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --fps <6|15|30|60>" << std::endl
    << "    Frames per second." << std::endl
    << "    Default: 30." << std::endl
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
    << "  Example to record a sequence of images:" << std::endl
    << "    " << argv[0] << " --record 0" << std::endl
    << std::endl
    << "  Example to record single shot images:\n"
    << "    " << argv[0] << " --record 1" << std::endl
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
#if defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0)) && defined(VISP_HAVE_THREADS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    std::string opt_seqname_left = "left-%04d.png", opt_seqname_right = "right-%04d.png";
    int opt_record_mode = 0;
    int opt_fps = 30;
    bool opt_display = true;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--fps") {
        opt_fps = std::atoi(argv[i + 1]);
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

    if (!opt_display) {
      opt_record_mode = 0;
    }

    std::cout << "Framerate  : " << opt_fps << std::endl;
    std::cout << "Display    : " << (opt_display ? "enabled" : "disabled") << std::endl;

    std::string text_record_mode =
      std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    std::cout << text_record_mode << std::endl;
    std::cout << "Left record name: " << opt_seqname_left << std::endl;
    std::cout << "Right record name: " << opt_seqname_right << std::endl;

    vpImage<unsigned char> I_left, I_right;

    vpRealSense2 g;
    rs2::config config;
    config.enable_stream(RS2_STREAM_FISHEYE, 1);
    config.enable_stream(RS2_STREAM_FISHEYE, 2);
    g.open(config);

    g.acquire(&I_left, &I_right);

    std::cout << "Image size : " << I_left.getWidth() << " " << I_right.getHeight() << std::endl;

    vpDisplay *display_left = nullptr, *display_right = nullptr;
    if (opt_display) {
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << "No image viewer is available..." << std::endl;
      opt_display = false;
#endif
    }
    if (opt_display) {
#ifdef VISP_HAVE_X11
      display_left = new vpDisplayX(I_left, 10, 10, "Left image");
      display_right = new vpDisplayX(I_right, I_left.getWidth(), 10, "Right image");
#elif defined(VISP_HAVE_GDI)
      display_left = new vpDisplayGDI(I_left, 10, 10, "Left image");
      display_right = new vpDisplayGDI(I_right, I_left.getWidth(), 10, "Right image");
#elif defined(HAVE_OPENCV_HIGHGUI)
      display_left = new vpDisplayOpenCV(I_left, 10, 10, "Left image");
      display_right = new vpDisplayOpenCV(I_right, I_left.getWidth(), 10, "Right image");
#endif
    }

    vpImageQueue<unsigned char> image_queue_left(opt_seqname_left, opt_record_mode);
    vpImageQueue<unsigned char> image_queue_right(opt_seqname_right, opt_record_mode);
    vpImageStorageWorker<unsigned char> image_left_storage_worker(std::ref(image_queue_left));
    vpImageStorageWorker<unsigned char> image_right_storage_worker(std::ref(image_queue_right));
    std::thread image_left_storage_thread(&vpImageStorageWorker<unsigned char>::run, &image_left_storage_worker);
    std::thread image_right_storage_thread(&vpImageStorageWorker<unsigned char>::run, &image_right_storage_worker);

    bool quit = false;
    while (!quit) {
      double t = vpTime::measureTimeMs();

      g.acquire(&I_left, &I_right);

      vpDisplay::display(I_left);
      vpDisplay::display(I_right);

      quit = image_queue_left.record(I_left);
      quit |= image_queue_right.record(I_right, nullptr, image_queue_left.getRecordingTrigger(), true);

      std::stringstream ss;
      ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
      vpDisplay::displayText(I_left, I_left.getHeight() - 20, 10, ss.str(), vpColor::red);
      vpDisplay::flush(I_left);
      vpDisplay::flush(I_right);
    }
    image_queue_left.cancel();
    image_queue_right.cancel();
    image_left_storage_thread.join();
    image_right_storage_thread.join();

    if (display_left) {
      delete display_left;
    }
    if (display_right) {
      delete display_right;
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
}
#else
  (void)argc;
  (void)argv;
#if !(defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0)))
  std::cout << "Install librealsense version > 2.31.0, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
