/*! \example tutorial-grabber-opencv.cpp */
#include <stdlib.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageStorageWorker.h>

#if defined(HAVE_OPENCV_VIDEOIO)
#include <opencv2/videoio.hpp>
#endif

#define USE_COLOR // Comment to acquire gray level images

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--device <index>]"
    << " [--seqname <sequence name>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --device <index> " << std::endl
    << "    Camera device index. Set 0 to dial with the first camera," << std::endl
    << "    and 1 to dial with the second camera attached to the computer." << std::endl
    << "    Default: 0 to consider /dev/video0 device." << std::endl
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
    << "  Example to visualize images from a second camera:" << std::endl
    << "    " << argv[0] << " --device 1" << std::endl
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

// usage: binary -h
// device name: 0 is the default to dial with the first camera,
// 1 to dial with a second camera attached to the computer
int main(int argc, const char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  int opt_device = 0;
  std::string opt_seqname;
  int opt_record_mode = 0;
  bool opt_display = true;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--device") {
      opt_device = std::atoi(argv[i + 1]);
      i++;
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

  std::cout << "Use device : " << opt_device << std::endl;
  std::cout << "Recording  : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
  std::cout << "Display    : " << (opt_display ? "enabled" : "disabled") << std::endl;

  std::string text_record_mode =
    std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

  if (!opt_seqname.empty()) {
    std::cout << text_record_mode << std::endl;
    std::cout << "Record name: " << opt_seqname << std::endl;
  }

#if defined(HAVE_OPENCV_VIDEOIO) && defined(HAVE_OPENCV_HIGHGUI) && defined(VISP_HAVE_THREADS)
  try {
    cv::VideoCapture cap(opt_device); // open the default camera
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    int i = 0;
    while ((i++ < 20) && !cap.read(frame)) {
    }; // warm up camera by skiping unread frames

    std::cout << "Image size : " << frame.rows << " " << frame.cols << std::endl;

#ifdef USE_COLOR
    vpImage<vpRGBa> I; // To acquire color images
#else
    vpImage<unsigned char> I; // To acquire gray images
#endif
    vpImageConvert::convert(frame, I);

    vpDisplayOpenCV *d = nullptr;
    if (opt_display) {
      d = new vpDisplayOpenCV(I);
    }

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
    while (!quit) {
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
#if !defined(HAVE_OPENCV_VIDEOIO)
  std::cout << "Install OpenCV videoio module, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
  }
