//! \example tutorial-grabber-1394.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/sensor/vp1394TwoGrabber.h>

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--change-settings]"
    << " [--seqname <sequence name>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --change-settings" << std::endl
    << "    Force camera settings to acquire 640x480 images in M0NO8 at 60 fps." << std::endl
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
    << "  Examples to record a sequence:" << std::endl
    << "    " << argv[0] << " --seqname I%04d.png" << std::endl
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 0" << std::endl
    << std::endl
    << "  Examples to record single shot images:\n"
    << "    " << argv[0] << " --seqname I%04d.png --record 1\n"
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 1" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_DC1394) && defined(VISP_HAVE_THREADS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    std::string opt_seqname;
    int opt_record_mode = 0;
    bool opt_change_settings = false;
    bool opt_display = true;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--change-settings") {
        opt_change_settings = true;
      }
      else if (std::string(argv[i]) == "--seqname") {
        opt_seqname = std::string(argv[i + 1]);
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

    std::cout << "Settings   : " << (opt_change_settings ? "modified" : "current") << std::endl;
    std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
    std::cout << "Display    : " << (opt_display ? "enabled" : "disabled") << std::endl;

    std::string text_record_mode =
      std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (!opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }

    vpImage<vpRGBa> I;  // Create a color image container
    bool reset = false; // Disable bus reset during construction (default)
    //! [vp1394TwoGrabber construction]
    vp1394TwoGrabber g(reset); // Create a grabber based on libdc1394-2.x third party lib
    //! [vp1394TwoGrabber construction]

    //! [vp1394TwoGrabber settings]
    if (opt_change_settings) {
      try {
        g.setVideoMode(vp1394TwoGrabber::vpVIDEO_MODE_640x480_MONO8);
        g.setFramerate(vp1394TwoGrabber::vpFRAMERATE_60);
      }
      catch (...) { // If settings are not available just catch execption to
     // continue with default settings
        std::cout << "Warning: cannot modify camera settings" << std::endl;
      }
    }
    //! [vp1394TwoGrabber settings]
    //! [vp1394TwoGrabber open]
    g.open(I);
    //! [vp1394TwoGrabber open]

    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;

    vpDisplay *d = nullptr;
    if (opt_display) {
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_OPENCV))
      std::cout << "No image viewer is available..." << std::endl;
      opt_display = false;
#endif
    }
    if (opt_display) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
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
      //! [vp1394TwoGrabber acquire]
      g.acquire(I);
      //! [vp1394TwoGrabber acquire]
      vpDisplay::display(I);
      //! [vp1394TwoGrabber click to exit]
      quit = image_queue.record(I);
      //! [vp1394TwoGrabber click to exit]
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
#ifndef VISP_HAVE_DC1394
  std::cout << "Install libdc1394, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
