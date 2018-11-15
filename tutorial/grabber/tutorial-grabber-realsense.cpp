/*! \example tutorial-grabber-realsense.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>

/*!
  Grab images from an Intel realsense camera
 */
int main(int argc, char **argv)
{
#if defined(VISP_HAVE_REALSENSE) || defined(VISP_HAVE_REALSENSE2)
  try {
    std::string opt_seqname;
    int opt_record_mode = 0;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--seqname")
        opt_seqname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--seqname <sequence name>] [--record <0: continuous (default) | 1: single shot>] "
                     "[--help] [-h]\n"
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

    std::cout << "Recording: " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;

    std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (! opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name: " << opt_seqname << std::endl;
    }
    vpImage<unsigned char> I;

#ifdef VISP_HAVE_REALSENSE2
    std::cout << "SDK: Realsense 2" << std::endl;
    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    g.open(config);
#else
    std::cout << "SDK: Realsense 1" << std::endl;
    vpRealSense g;
    unsigned int width = 640, height = 480;
    g.setStreamSettings(rs::stream::color, vpRealSense::vpRsStreamParams(width, height, rs::format::rgba8, 60));
    g.open();
#endif
    g.acquire(I);

    std::cout << "Image size: " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    unsigned int counter = 1;
    bool start_record = false;

    while (1) {
      double t = vpTime::measureTimeMs();
      g.acquire(I);

      vpDisplay::display(I);
      if (! opt_seqname.empty()) {
        if (! opt_record_mode) { // continuous
          if (start_record) {
            vpDisplay::displayText(I, 20, 10, "Left  click: stop recording", vpColor::red);
          }
          else {
            vpDisplay::displayText(I, 20, 10, "Left  click: start recording", vpColor::red);
          }
        }
        else {
          vpDisplay::displayText(I, 20, 10, "Left  click: record image", vpColor::red);
        }
        vpDisplay::displayText(I, 40, 10, "Right click: quit", vpColor::red);
      }
      else {
        vpDisplay::displayText(I, 20, 10, "Click to quit", vpColor::red);
      }

      if (! opt_seqname.empty()) {
        vpDisplay::displayText(I, 60, 10, text_record_mode, vpColor::red);
      }
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (! opt_seqname.empty()) { // Recording requested
          if (button == vpMouseButton::button1) { // enable/disable recording
            start_record = !start_record;
          }
          else if (button == vpMouseButton::button3) { // quit
            break;
          }
        }
        else { // any button to quit
          break;
        }
      }
      if (start_record) {
        char filename[FILENAME_MAX];
        sprintf(filename, opt_seqname.c_str(), counter);
        {
          // check if parent folder exists. Create otherwise
          static bool parent_exists = false;
          if (! parent_exists) {
            std::string parent = vpIoTools::getParent(filename);
            if (! parent.empty()) {
              if (! vpIoTools::checkDirectory(parent)) {
                vpIoTools::makeDirectory(parent);
              }
            }
            parent_exists = true;
          }
        }

        counter ++;
        std::string text = std::string("Save: ") + std::string(filename);
        vpDisplay::displayText(I, 80, 10, text, vpColor::red);
        std::cout << text << std::endl;
        vpImageIo::write(I, filename);
        if (opt_record_mode == 1) { // single shot mode
          start_record = false;
        }
      }

      std::stringstream ss;
      ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
      vpDisplay::displayText(I, I.getHeight() - 20, 10, ss.str(), vpColor::red);
      vpDisplay::flush(I);
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  std::cout << "Install librealsense, configure and build ViSP again to use this example" << std::endl;
#endif
}
