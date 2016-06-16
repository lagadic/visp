//! \example tutorial-face-detector-live-threaded.cpp
#include <iostream>

#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/detection/vpDetectorFace.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020200) && (defined(VISP_HAVE_PTHREAD) || defined(_WIN32))

#include <opencv2/highgui/highgui.hpp>

// Shared vars
typedef enum {
  capture_waiting,
  capture_started,
  capture_stopped
} t_CaptureState;
t_CaptureState s_capture_state = capture_waiting;
bool s_face_available = false;
#if defined(VISP_HAVE_V4L2)
vpImage<unsigned char> s_frame;
#elif defined(VISP_HAVE_OPENCV)
cv::Mat s_frame;
#endif
vpMutex s_mutex_capture;
vpMutex s_mutex_face;
vpRect s_face_bbox;

vpThread::Return captureFunction(vpThread::Args args)
{
#if defined(VISP_HAVE_V4L2)
  vpV4l2Grabber cap = *((vpV4l2Grabber *) args);
#elif defined(VISP_HAVE_OPENCV)
  cv::VideoCapture cap = *((cv::VideoCapture *) args);
#endif

  // If the image is larger than 640 by 480, we subsample
#if defined(VISP_HAVE_V4L2)
  vpImage<unsigned char> frame_;
#elif defined(VISP_HAVE_OPENCV)
  cv::Mat frame_;
#endif
  bool stop_capture_ = false;

  double start_time = vpTime::measureTimeSecond();
  while ((vpTime::measureTimeSecond() - start_time) < 30 && !stop_capture_) {
    // Capture in progress
    cap >> frame_; // get a new frame from camera

    // Update shared data
    {
      vpMutex::vpScopedLock lock(s_mutex_capture);
      if (s_capture_state == capture_stopped)
        stop_capture_ = true;
      else
        s_capture_state = capture_started;
      s_frame = frame_;
    }
  }
  {
    vpMutex::vpScopedLock lock(s_mutex_capture);
    s_capture_state = capture_stopped;
  }

  std::cout << "End of capture thread" << std::endl;
  return 0;
}

vpThread::Return displayFunction(vpThread::Args args)
{
  (void)args; // Avoid warning: unused parameter args
  vpImage<unsigned char> I_;

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
  bool face_available_ = false;
  vpRect face_bbox_;
#if defined(VISP_HAVE_X11)
  vpDisplayX *d_ = NULL;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI *d_ = NULL;
#endif

  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      // Get the frame and convert it to a ViSP image used by the display class
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
#if defined(VISP_HAVE_V4L2)
        I_ = s_frame;
#elif defined(VISP_HAVE_OPENCV)
        vpImageConvert::convert(s_frame, I_);
#endif
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

      // Check if a face was detected
      {
        vpMutex::vpScopedLock lock(s_mutex_face);
        face_available_ = s_face_available;
        face_bbox_ = s_face_bbox;
      }
      if (face_available_) {
        // Access to the face bounding box to display it
        vpDisplay::displayRectangle(I_, face_bbox_, vpColor::green, false, 4);
        face_available_ = false;
      }

      // Trigger end of acquisition with a mouse click
      vpDisplay::displayText(I_, 10, 10, "Click to exit...", vpColor::red);
      if (vpDisplay::getClick(I_, false)) {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        s_capture_state = capture_stopped;
      }

      // Update the display
      vpDisplay::flush(I_);
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(capture_state_ != capture_stopped);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  delete d_;
#endif

  std::cout << "End of display thread" << std::endl;
  return 0;
}

//! [face-detection-threaded detectionFunction]
vpThread::Return detectionFunction(vpThread::Args args)
{
  std::string opt_face_cascade_name = *((std::string *) args);

  vpDetectorFace face_detector_;
  face_detector_.setCascadeClassifierFile(opt_face_cascade_name);

  t_CaptureState capture_state_;
#if defined(VISP_HAVE_V4L2)
  vpImage<unsigned char> frame_;
#elif defined(VISP_HAVE_OPENCV)
  cv::Mat frame_;
#endif
  do {
    s_mutex_capture.lock();
    capture_state_ = s_capture_state;
    s_mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      // Backup the frame
      {
        vpMutex::vpScopedLock lock(s_mutex_capture);
        frame_ = s_frame;
      }

      // Detect faces
      bool face_found_ = face_detector_.detect(frame_);
      if (face_found_) {
        vpMutex::vpScopedLock lock(s_mutex_face);
        s_face_available = true;
        s_face_bbox = face_detector_.getBBox(0); // Get largest face bounding box
      }
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while(capture_state_ != capture_stopped);
  std::cout << "End of face detection thread" << std::endl;

  return 0;
}
//! [face-detection-threaded detectionFunction]

//! [face-detection-threaded mainFunction]
int main(int argc, const char* argv[])
{
  std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";
  unsigned int opt_device = 0;
  unsigned int opt_scale = 2; // Default value is 2 in the constructor. Turn it to 1 to avoid subsampling

  for (int i=0; i<argc; i++) {
    if (std::string(argv[i]) == "--haar")
      opt_face_cascade_name = std::string(argv[i+1]);
    else if (std::string(argv[i]) == "--device")
      opt_device = (unsigned int)atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--scale")
      opt_scale = (unsigned int)atoi(argv[i+1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0] << " [--haar <haarcascade xml filename>] [--device <camera device>] [--scale <subsampling factor>] [--help]" << std::endl;
      return 0;
    }
  }

  // Instanciate the capture
#if defined(VISP_HAVE_V4L2)
  vpV4l2Grabber cap;
  std::ostringstream device;
  device << "/dev/video" << opt_device;
  cap.setDevice(device.str());
  cap.setScale(opt_scale);
#elif defined(VISP_HAVE_OPENCV)
  cv::VideoCapture cap;
  cap.open(opt_device);
#  if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    int width  = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
    int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width/opt_scale);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height/opt_scale);
#  else
    int width  = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, width/opt_scale);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, height/opt_scale);
#  endif
#endif

  // Start the threads
  vpThread thread_capture((vpThread::Fn)captureFunction, (vpThread::Args)&cap);
  vpThread thread_display((vpThread::Fn)displayFunction);
  vpThread thread_detection((vpThread::Fn)detectionFunction, (vpThread::Args)&opt_face_cascade_name);

  // Wait until thread ends up
  thread_capture.join();
  thread_display.join();
  thread_detection.join();

  return 0;
}
//! [face-detection-threaded mainFunction]

#else
int main()
{
#  ifndef VISP_HAVE_OPENCV
  std::cout << "You should install OpenCV to make this example working..." << std::endl;
#  elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#  else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#  endif
}

#endif
