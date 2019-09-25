/*! \example tutorial-grabber-bebop2.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotBebop2.h>

#include "record_helper.h"

/*!
  Grab images from a Parrot Bebop 2 drone
 */
int main(int argc, char **argv)
{
#if defined(VISP_HAVE_ARSDK) && defined(VISP_HAVE_FFMPEG)
  try {
    std::string opt_seqname;
    int opt_record_mode = 0;
    int image_res = 0;
    std::string ip_address = "192.168.42.1";

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--seqname")
        opt_seqname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
        ip_address = std::string(argv[i + 1]);
        i++;
      } else if (std::string(argv[i]) == "--hd_resolution")
        image_res = 1;
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage:\n"
                  << "  " << argv[0] << " [--seqname <sequence name>] [--record <0: continuous | 1: single shot>]"
                  << " [--ip <drone ip>] [--hd_resolution]"
                  << " [--help] [-h]\n"
                  << std::endl
                  << "Description:\n"
                  << " --seqname <sequence name>\n"
                  << "     Desired name for picture sequence (default: empty).\n\n"
                  << " --record <0: continuous | 1: single shot>\n"
                  << "     Record mode (default : 0, continuous).\n\n"
                  << " --ip <drone ip>\n"
                  << "     IP address of the drone to which you want to connect (default : 192.168.42.1).\n\n"
                  << " --hd_resolution\n"
                  << "     Enables HD 720p images instead of default 480p.\n"
                  << "     Caution : camera calibration settings are different for the two resolutions.\n\n"
                  << "--help, -h\n"
                  << "     Print help message.\n\n"

                  << "\nExample to visualize images:\n"
                  << "  " << argv[0] << "\n"
                  << "\nExamples to record a sequence of 720p images from drone with ip different from default:\n"
                  << "  " << argv[0] << " --seqname I%04d.png --ip 192.168.42.3 --hd_resolution\n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 0 --ip 192.168.42.3 --hd_resolution\n"
                  << "\nExamples to record single shot images:\n"
                  << "  " << argv[0] << " --seqname I%04d.png --record 1\n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 1\n"
                  << std::endl;
        return 0;
      } else {
        std::cout << "Error : unknown parameter " << argv[i] << std::endl
                  << "See " << argv[0] << " --help" << std::endl;
        return 0;
      }
    }

    std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;

    std::string text_record_mode =
        std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (!opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }
    std::cout << "Image resolution : " << (image_res == 0 ? "480p." : "720p.") << std::endl << std::endl;

    vpImage<unsigned char> I(1, 1, 0);

    vpRobotBebop2 drone(false, true, ip_address);

    if (drone.isRunning()) {

      drone.setVideoResolution(image_res);

      drone.startStreaming();
      drone.setExposure(1.5f);
      drone.getGrayscaleImage(I);
    } else {
      std::cout << "Error : failed to setup drone control" << std::endl;
      return 1;
    }

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

    bool quit = false;
    while (!quit) {
      double t = vpTime::measureTimeMs();
      drone.getGrayscaleImage(I);

      vpDisplay::display(I);

      quit = record_helper(opt_seqname, opt_record_mode, I);

      std::stringstream ss;
      ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
      vpDisplay::displayText(I, static_cast<int>(I.getHeight()) - 20, 10, ss.str(), vpColor::red);
      vpDisplay::flush(I);
    }
  } catch (const vpException &e) {
    std::cout << "Caught an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install Parrot ARSDK3 and ffmpeg, configure and build ViSP again to use this example" << std::endl;
#endif // #if defined(VISP_HAVE_ARSDK) && defined(VISP_HAVE_FFMPEG)
}
