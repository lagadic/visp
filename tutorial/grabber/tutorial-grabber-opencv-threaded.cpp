//! \example tutorial-grabber-opencv-threaded.cpp
//! [capture-multi-threaded declaration]
#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)

#include <opencv2/highgui/highgui.hpp>

// Shared vars
unsigned int s_frame_index = 0;
bool s_frame_available = false;
cv::Mat s_frame;
vpMutex s_mutex_frame;
//! [capture-multi-threaded declaration]

//! [capture-multi-threaded captureFunction]
vpThread::Return captureFunction(vpThread::Args args)
{
  cv::VideoCapture cap = *((cv::VideoCapture *) args);

  if(!cap.isOpened()) { // check if we succeeded
    std::cout << "Unable to start capture" << std::endl;
    return 0;
  }

  cv::Mat frame_;

  double start_time = vpTime::measureTimeSecond();
  while ((vpTime::measureTimeSecond() - start_time) < 4) {
    // Capture in progress
    cap >> frame_; // get a new frame from camera

    // Update shared data
    {
      vpMutex::vpScopedLock lock(s_mutex_frame);
      s_frame_available = true;
      s_frame_index ++;
      s_frame = frame_;
    }
  }
  // Update the flag indicating end of capture
  {
    vpMutex::vpScopedLock lock(s_mutex_frame);
    s_frame_available = false;
  }

  std::cout << "End of capture thread" << std::endl;
  return 0;
}
//! [capture-multi-threaded captureFunction]

//! [capture-multi-threaded displayFunction]
vpThread::Return displayFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args
  vpImage<unsigned char> I_;

  bool frame_available_ = false;
  unsigned int frame_prev_index_ = 0;
  bool display_initialized_ = false;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d_ = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *d_ = NULL;
#endif

  do {
    s_mutex_frame.lock();
    frame_available_ = s_frame_available;
    unsigned int frame_index_ = s_frame_index;
    s_mutex_frame.unlock();

    // Check if a new frame is available
    if (frame_available_ && frame_index_ > frame_prev_index_) {
      // Get the frame and convert it to a ViSP image used by the display class
      {
        vpMutex::vpScopedLock lock(s_mutex_frame);
        vpImageConvert::convert(s_frame, I_);
      }

      // Check if we need to initialize the display with the first frame
      if (! display_initialized_) {
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

      // Update the display
      vpDisplay::flush(I_);

      // Update the previous frame index for next iteration
      frame_prev_index_ = frame_index_;
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(frame_available_ || frame_prev_index_ == 0);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  delete d_;
#endif

  std::cout << "End of display thread" << std::endl;
  return 0;
}
//! [capture-multi-threaded displayFunction]

//! [capture-multi-threaded mainFunction]
int main(int argc, const char* argv[])
{
  int opt_device = 0;

  // Command line options
  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--device")
      opt_device = atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--device <camera device>] [--help]" << std::endl;
      return 0;
    }
  }

  // Instanciate the capture
  cv::VideoCapture cap;
  cap.open(opt_device);

  // Start the threads
  vpThread thread_capture(captureFunction, (vpThread::Args)&cap);
  vpThread thread_display(displayFunction);

  // Wait until thread ends up
  thread_capture.join();
  thread_display.join();

  return 0;
}
//! [capture-multi-threaded mainFunction]

#else
int main()
{
  std::cout << "You should install OpenCV to make this example working..." << std::endl;
}

#endif
