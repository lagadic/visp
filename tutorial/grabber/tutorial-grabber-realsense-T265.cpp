/*! \example tutorial-grabber-realsense-T265.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/io/vpImageStorageWorker.h>

/*!
  Grab images from an Intel realsense camera
 */
int main(int argc, char **argv)
{
#if defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0)) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  try {
    std::string opt_seqname_left = "left-%04d.png", opt_seqname_right = "right-%04d.png";
    int opt_record_mode = 0;
    int opt_fps = 30;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--fps")
        opt_fps = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--record <0: continuous | 1: single shot (default: 0)>]"
                     " [--help] [-h]\n"
                  << "\nExample to visualize images:\n"
                  << "  " << argv[0] << "\n"
                  << "\nExamples to record single shot images:\n"
                  << "  " << argv[0] << " --record 1\n"
                  << "\nExamples to record a sequence of images:\n"
                  << "  " << argv[0] << " --record 0\n"
                  << std::endl;
        return 0;
      }
    }

    std::cout << "Framerate  : " << opt_fps << std::endl;

    std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

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

#ifdef VISP_HAVE_X11
    vpDisplayX display_left(I_left, 10, 10, "Left image");
    vpDisplayX display_right(I_right, I_left.getWidth(), 10, "Right image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display_left(I_left);
    vpDisplayGDI display_right(I_right);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display_left(I_left);
    vpDisplayOpenCV display_right(I_right);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

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
      quit |= image_queue_right.record(I_right, NULL, image_queue_left.getRecordingTrigger(), true);

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
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void) argc;
  (void) argv;
#if !(defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0)))
  std::cout << "Install librealsense version > 2.31.0, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This turorial should be built with c++11 support" << std::endl;
#endif
#endif
}
