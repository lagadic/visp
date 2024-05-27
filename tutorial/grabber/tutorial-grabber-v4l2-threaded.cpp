//! \example tutorial-grabber-v4l2-threaded.cpp
//! [capture-multi-threaded declaration]
#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#if defined(VISP_HAVE_V4L2) && defined(VISP_HAVE_THREADS) && defined(VISP_HAVE_DISPLAY)

#include <thread>
#include <mutex>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

// Shared vars
typedef enum { capture_waiting, capture_started, capture_stopped } t_CaptureState;
//! [capture-multi-threaded declaration]

//! [capture-multi-threaded captureFunction]
void captureFunction(vpV4l2Grabber &cap, std::mutex &mutex_capture, vpImage<unsigned char> &frame, t_CaptureState &capture_state)
{
  vpImage<unsigned char> frame_;
  bool stop_capture_ = false;

  cap.open(frame_);

  double start_time = vpTime::measureTimeSecond();
  while ((vpTime::measureTimeSecond() - start_time) < 30 && !stop_capture_) {
    // Capture in progress
    cap.acquire(frame_); // get a new frame from camera

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
void displayFunction(std::mutex &mutex_capture, vpImage<unsigned char> &frame, t_CaptureState &capture_state)
{
  vpImage<unsigned char> I_;

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
  vpDisplay *d_ = vpDisplayFactory::displayFactory();

  do {
    mutex_capture.lock();
    capture_state_ = capture_state;
    mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      // Create a copy of the captured frame
      {
        std::lock_guard<std::mutex> lock(mutex_capture);
        I_ = frame;
      }

      // Check if we need to initialize the display with the first frame
      if (!display_initialized_) {
      // Initialize the display
        d_->init(I_);
        display_initialized_ = true;
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

  delete d_;

  std::cout << "End of display thread" << std::endl;
}
//! [capture-multi-threaded displayFunction]

//! [capture-multi-threaded mainFunction]
int main(int argc, const char *argv[])
{
  unsigned int opt_device = 0; // Default is opening /dev/video0
  unsigned int opt_scale = 2;  // Default value is 2 in the constructor. Turn
                               // it to 1 to avoid subsampling

  // Command line options
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--camera_device")
      opt_device = (unsigned int)atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--scale")
      opt_scale = (unsigned int)atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "--h") {
      std::cout << "Usage: " << argv[0]
        << " [--camera_device <camera device (default: 0)>] [--scale <subsampling factor>]"
        << " [--help] [-h]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  // Instantiate the grabber
  vpV4l2Grabber g;
  std::ostringstream device;
  device << "/dev/video" << opt_device;
  g.setDevice(device.str());
  g.setScale(opt_scale);

  vpImage<unsigned char> frame;
  std::mutex mutex_capture;
  t_CaptureState capture_state = capture_waiting;

  // Start the threads
  std::thread thread_capture(&captureFunction, std::ref(g), std::ref(mutex_capture), std::ref(frame), std::ref(capture_state));
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
#ifndef VISP_HAVE_V4L2
  std::cout << "You should enable V4L2 to make this example working..." << std::endl;
#elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#elif !defined(VISP_HAVE_DISPLAY)
  std::cout << "You should have at least one GUI library installed to use this example." << std::endl;
#else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#endif
}

#endif
