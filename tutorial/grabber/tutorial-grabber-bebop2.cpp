/*! \example tutorial-grabber-bebop2.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageStorageWorker.h>

#ifdef VISP_HAVE_MODULE_ROBOT
#include <visp3/robot/vpRobotBebop2.h>

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--ip <address>]"
    << " [--hd-resolution]"
    << " [--seqname <sequence name>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --ip <address>" << std::endl
    << "    IP address of the drone to which you want to connect." << std::endl
    << "    Default: 192.168.42.1" << std::endl
    << std::endl
    << "  --hd-resolution" << std::endl
    << "    Enables HD 720p images instead of default 480p." << std::endl
    << "    Caution : The camera settings are different depending on whether the resolution is 720p or 480p."
    << std::endl
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
    << "  Examples to record a sequence of 720p images from drone with ip different from default:" << std::endl
    << "    " << argv[0] << " --seqname I%04d.png --ip 192.168.42.3 --hd_resolution" << std::endl
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 0 --ip 192.168.42.3 --hd-resolution"
    << std::endl
    << std::endl
    << "  Examples to record single shot images:" << std::endl
    << "    " << argv[0] << " --seqname I%04d.png --record 1" << std::endl
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 1" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

/*!
  Grab images from a Parrot Bebop 2 drone
 */
int main(int argc, const char **argv)
{
#if defined(VISP_HAVE_ARSDK) && defined(VISP_HAVE_FFMPEG) && defined(VISP_HAVE_THREADS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    std::string opt_seqname;
    int opt_record_mode = 0;
    int image_res = 0;
    std::string ip_address = "192.168.42.1";
    bool opt_display = true;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--seqname") {
        opt_seqname = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--record") {
        opt_record_mode = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
        ip_address = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--hd-resolution") {
        image_res = 1;
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

    std::cout << "Recording       : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
    std::cout << "Display         : " << (opt_display ? "enabled" : "disabled") << std::endl;

    std::string text_record_mode =
      std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (!opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }
    std::cout << "Image resolution : " << (image_res == 0 ? "480p." : "720p.") << std::endl << std::endl;

    vpImage<vpRGBa> I(1, 1, 0);

    vpRobotBebop2 drone(false, true, ip_address);

    if (drone.isRunning()) {

      drone.setVideoResolution(image_res);

      drone.startStreaming();
      drone.setExposure(1.5f);
      drone.getRGBaImage(I);
    }
    else {
      std::cout << "Error : failed to setup drone control" << std::endl;
      return EXIT_FAILURE;
    }

    std::cout << "Image size : " << I.getWidth() << " " << I.getHeight() << std::endl;

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
      drone.getRGBaImage(I);

      vpDisplay::display(I);

      quit = image_queue.record(I);

      std::stringstream ss;
      ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
      vpDisplay::displayText(I, static_cast<int>(I.getHeight()) - 20, 10, ss.str(), vpColor::red);
      vpDisplay::flush(I);
    }
    image_queue.cancel();
    image_storage_thread.join();

    if (d) {
      delete d;
    }
  }
  catch (const vpException &e) {
    std::cout << "Caught an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_ARSDK
  std::cout << "Install Parrot ARSDK3, configure and build ViSP again to use this example" << std::endl;
#endif
#ifndef VISP_HAVE_FFMPEG
  std::cout << "Install ffmpeg, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
#else
int main() { std::cout << "This tutorial needs visp_robot module that is not built." << std::endl; }
#endif
