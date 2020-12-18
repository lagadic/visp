/*! \example tutorial-grabber-opencv.cpp */
#include <stdlib.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageStorageWorker.h>

//#define USE_COLOR // Comment to acquire gray level images

// usage: binary -h
// device name: 0 is the default to dial with the first camera,
// 1 to dial with a second camera attached to the computer
int main(int argc, char **argv)
{
  int opt_device = 0;
  std::string opt_seqname;
  int opt_record_mode = 0;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--camera_device")
      opt_device = std::atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--seqname")
      opt_seqname = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--record")
      opt_record_mode = std::atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
                << " [--camera_device <camera device (default: 0>]"
                << " [--seqname <sequence name (default: empty>] [--record <0: continuous | 1: single shot (default: 0)>]"
                << " [--help] [-h]\n"
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

#if (VISP_HAVE_OPENCV_VERSION >= 0x020100) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
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

    std::cout << "Image size : " << frame.rows << " " << frame.cols << std::endl;

#ifdef USE_COLOR
    vpImage<vpRGBa> I;        // To acquire color images
#else
    vpImage<unsigned char> I; // To acquire gray images
#endif
    vpImageConvert::convert(frame, I);

    vpDisplayOpenCV d(I);

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
      cap >> frame; // get a new frame from camera
      // Convert the image in ViSP format and display it
      vpImageConvert::convert(frame, I);

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
#if (VISP_HAVE_OPENCV_VERSION < 0x020100)
  std::cout << "Install OpenCV, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This turorial should be built with c++11 support" << std::endl;
#endif
#endif
}
