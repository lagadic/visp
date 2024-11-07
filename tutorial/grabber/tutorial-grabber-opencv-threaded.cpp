//! \example tutorial-grabber-opencv-threaded.cpp
//! [capture-multi-threaded declaration]
#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

#if defined(HAVE_OPENCV_VIDEOIO) && defined(VISP_HAVE_THREADS)

#include <thread>
#include <mutex>

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

// Possible capture states
typedef enum { capture_waiting, capture_started, capture_stopped } t_CaptureState;
//! [capture-multi-threaded declaration]

//! [capture-multi-threaded captureFunction]
void captureFunction(cv::VideoCapture &cap, std::mutex &mutex_capture, cv::Mat &frame, t_CaptureState &capture_state)
{
  if (!cap.isOpened()) { // check if we succeeded
    std::cout << "Unable to start capture" << std::endl;
    return;
  }

  cv::Mat frame_;
  int i = 0;
  while ((i++ < 100) && !cap.read(frame_)) {
  }; // warm up camera by skipping unread frames

  bool stop_capture_ = false;

  double start_time = vpTime::measureTimeSecond();
  while ((vpTime::measureTimeSecond() - start_time) < 30 && !stop_capture_) {
    // Capture in progress
    cap >> frame_; // get a new frame from camera

    // Update shared data
    {
      std::lock_guard<std::mutex> lock(mutex_capture);
      if (capture_state == capture_stopped)
        stop_capture_ = true;
      else
        capture_state = capture_started;
      frame = frame_;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_capture);
    capture_state = capture_stopped;
  }

  std::cout << "End of capture thread" << std::endl;
}
//! [capture-multi-threaded captureFunction]

//! [capture-multi-threaded displayFunction]
void displayFunction(std::mutex &mutex_capture, cv::Mat &frame, t_CaptureState &capture_state)
{
  vpImage<unsigned char> I_;

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d_ = nullptr;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *d_ = nullptr;
#endif

  do {
    mutex_capture.lock();
    capture_state_ = capture_state;
    mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      // Get the frame and convert it to a ViSP image used by the display
      // class
      {
        std::lock_guard<std::mutex> lock(mutex_capture);
        vpImageConvert::convert(frame, I_);
      }

      // Check if we need to initialize the display with the first frame
      if (!display_initialized_) {
        // Initialize the display
#if defined(VISP_HAVE_X11)
        d_ = new vpDisplayX(I_);
        display_initialized_ = true;
#elif defined(VISP_HAVE_GDI)
        d_ = new vpDisplayGDI(I_);
        display_initialized_ = true;
#endif
      }

      // Display the image
      vpDisplay::display(I_);

      // Trigger end of acquisition with a mouse click
      vpDisplay::displayText(I_, 10, 10, "Click to exit...", vpColor::red);
      if (vpDisplay::getClick(I_, false)) {
        std::lock_guard<std::mutex> lock(mutex_capture);
        capture_state = capture_stopped;
      }

      // Update the display
      vpDisplay::flush(I_);
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while (capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  delete d_;
#endif

  std::cout << "End of display thread" << std::endl;
}
//! [capture-multi-threaded displayFunction]

//! [capture-multi-threaded mainFunction]
int main(int argc, const char *argv[])
{
  int opt_device = 0;

  // Command line options
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--camera_device")
      opt_device = atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--camera_device <camera device (default: 0)>] [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  // Instantiate the capture
  cv::VideoCapture cap;
  cap.open(opt_device);

  cv::Mat frame;
  t_CaptureState capture_state = capture_waiting;
  // Create a mutex for capture
  std::mutex mutex_capture;
  // Start the threads
  std::thread thread_capture(&captureFunction, std::ref(cap), std::ref(mutex_capture), std::ref(frame), std::ref(capture_state));
  std::thread thread_display(&displayFunction, std::ref(mutex_capture), std::ref(frame), std::ref(capture_state));

  // Wait until thread ends up
  thread_capture.join();
  thread_display.join();

  return EXIT_SUCCESS;
}
//! [capture-multi-threaded mainFunction]

#else
int main()
{
#ifndef VISP_HAVE_OPENCV
  std::cout << "You should install OpenCV videoio module to make this example working..." << std::endl;
#elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif
