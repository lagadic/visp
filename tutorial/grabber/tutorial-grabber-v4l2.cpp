/*! \example tutorial-grabber-v4l2.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/io/vpImageStorageWorker.h>

//#define USE_COLOR // Comment to acquire gray level images

/*!
  Usage :
    To get the help    : ./tutorial-grabber-v4l2 --help
    To set the device  : ./tutorial-grabber-v4l2 --device 1 (to use /dev/video1)
    To subsample images: ./tutorial-grabber-v4l2 --scale 2
 */
int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_V4L2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  try {
    int opt_device = 0;
    unsigned int opt_scale = 1; // Default value is 2 in the constructor. Turn
                                // it to 1 to avoid subsampling
    std::string opt_seqname;
    int opt_record_mode = 0;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--camera_device")
        opt_device = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--scale")
        opt_scale = (unsigned int)atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--seqname")
        opt_seqname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--camera_device <camera device (default: 0)>] [--scale <subsampling factor (default: 1)>]"
                     " [--seqname <sequence name (default: empty>] [--record <0: continuous | 1: single shot (default: 0)>]"
                     " [--help] [-h]\n"
                  << "\nExample to visualize images:\n"
                  << "  " << argv[0] << "\n"
                  << "\nExample to visualize images from a second camera:\n"
                  << "  " << argv[0] << " --camera_device 1\n"
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

    std::cout << "Use device : " << opt_device << std::endl;
    std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;

    std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (! opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }

#ifdef USE_COLOR
    vpImage<vpRGBa> I;        // To acquire color images
#else
    vpImage<unsigned char> I; // To acquire gray images
#endif

    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.setScale(opt_scale);
    g.open(I);

    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

#ifdef USE_COLOR
    vpImageQueue<vpRGBa> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<vpRGBa> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<vpRGBa>::run, &image_storage_worker);
#else
    vpImageQueue<unsigned char> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<unsigned char> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<unsigned char>::run, &image_storage_worker);
#endif

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
#ifndef VISP_HAVE_V4L2
  std::cout << "Install Video 4 Linux 2 (v4l2), configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This turorial should be built with c++11 support" << std::endl;
#endif
#endif
}
