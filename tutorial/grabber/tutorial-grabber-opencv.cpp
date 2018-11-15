/*! \example tutorial-grabber-opencv.cpp */
#include <stdlib.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>

// usage: binary -h
// device name: 0 is the default to dial with the first camera,
// 1 to dial with a second camera attached to the computer
int main(int argc, char **argv)
{
  int opt_device = 0;
  std::string opt_seqname;
  int opt_record_mode = 0;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--device")
      opt_device = std::atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--seqname")
      opt_seqname = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--record")
      opt_record_mode = std::atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
                << " [--device <device name (default: 0)>] [--seqname <sequence name>] [--record <0: continuous (default) | 1: single shot>] "
                   "[--help] [-h]\n"
                << "\nExample to visualize images:\n"
                << "  " << argv[0] << "\n"
                << "\nExample to visualize images from a second camera:\n"
                << "  " << argv[0] << "--device 1\n"
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

  std::cout << "Use device: " << opt_device << std::endl;
  std::cout << "Recording: " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;

  std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

  if (! opt_seqname.empty()) {
    std::cout << text_record_mode << std::endl;
    std::cout << "Record name: " << opt_seqname << std::endl;
  }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  try {
    cv::VideoCapture cap(opt_device); // open the default camera
    if (!cap.isOpened()) {        // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    int i = 0;
    while ((i++ < 20) && !cap.read(frame)) {
    }; // warm up camera by skiping unread frames

    std::cout << "Image size: " << frame.rows << " " << frame.cols << std::endl;

    // vpImage<vpRGBa> I; // for color images
    vpImage<unsigned char> I; // for gray images
    vpImageConvert::convert(frame, I);

    vpDisplayOpenCV d(I);

    unsigned int counter = 1;
    bool start_record = false;

    for (;;) {
      double t = vpTime::measureTimeMs();
      cap >> frame; // get a new frame from camera
      // Convert the image in ViSP format and display it
      vpImageConvert::convert(frame, I);

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
#endif
}
