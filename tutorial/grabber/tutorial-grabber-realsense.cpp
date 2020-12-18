/*! \example tutorial-grabber-realsense.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/io/vpImageStorageWorker.h>

/*!
  Grab images from an Intel realsense camera
 */
int main(int argc, char **argv)
{
#if defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  try {
    std::string opt_seqname;
    int opt_record_mode = 0;
    int opt_fps = 30;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--seqname")
        opt_seqname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--fps")
        opt_fps = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--seqname <sequence name (default: empty>] [--record <0: continuous | 1: single shot (default: 0)>]"
                     " [--fps <6|15|30|60>] [--help] [-h]\n"
                  << "\nExample to visualize images:\n"
                  << "  " << argv[0] << "\n"
                  << "\nExamples to record a sequence:\n"
                  << "  " << argv[0] << " --seqname I%04d.png \n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 0\n"
                  << "\nExamples to record single shot images:\n"
                  << "  " << argv[0] << " --seqname I%04d.png --record 1\n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 1\n"
                  << std::endl;
        return 0;
      }
    }

    if (opt_fps != 6 && opt_fps != 15 && opt_fps != 30 && opt_fps != 60) {
      opt_fps = 30; // Default
    }
    std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
    std::cout << "Framerate  : " << opt_fps << std::endl;

    std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (! opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }
    vpImage<unsigned char> I;

#ifdef VISP_HAVE_REALSENSE2
    std::cout << "SDK        : Realsense 2" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, opt_fps);
    g.open(config);
#else
    std::cout << "SDK        : Realsense 1" << std::endl;
    vpRealSense g;
    unsigned int width = 640, height = 480;
    g.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(width, height, rs::format::rgba8, 60));
    g.open();
#endif
    g.acquire(I);

    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpImageQueue<unsigned char> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<unsigned char> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<unsigned char>::run, &image_storage_worker);

    bool quit = false;
    while (! quit) {
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
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void) argc;
  (void) argv;
#if !(defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2))
  std::cout << "Install librealsense version > 2.31.0, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This turorial should be built with c++11 support" << std::endl;
#endif
#endif
}
