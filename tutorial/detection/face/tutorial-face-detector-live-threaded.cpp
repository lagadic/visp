//! \example tutorial-face-detector-live-threaded.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/core/vpTime.h>
#include <visp3/detection/vpDetectorFace.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#if defined(HAVE_OPENCV_OBJDETECT) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) \
  && defined(HAVE_OPENCV_VIDEOIO) && defined(VISP_HAVE_THREADS)

#include <thread>
#include <mutex>

#include <opencv2/videoio.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

// Shared vars
typedef enum { capture_waiting, capture_started, capture_stopped } t_CaptureState;

#if defined(VISP_HAVE_V4L2)
void captureFunction(vpV4l2Grabber &cap, std::mutex &mutex_capture, vpImage<unsigned char> &frame, t_CaptureState &capture_state)
#elif defined(HAVE_OPENCV_VIDEOIO)
void captureFunction(cv::VideoCapture &cap, std::mutex &mutex_capture, cv::Mat &frame, t_CaptureState &capture_state)
#endif
{
  // If the image is larger than 640 by 480, we subsample
#if defined(VISP_HAVE_V4L2)
  vpImage<unsigned char> frame_;
#elif defined(HAVE_OPENCV_VIDEOIO)
  cv::Mat frame_;
#endif
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

#if defined(VISP_HAVE_V4L2)
void displayFunction(std::mutex &mutex_capture, std::mutex &mutex_face, vpImage<unsigned char> &frame, t_CaptureState &capture_state, vpRect &face_bbox, bool &face_available)
#elif defined(HAVE_OPENCV_VIDEOIO)
void displayFunction(std::mutex &mutex_capture, std::mutex &mutex_face, cv::Mat &frame, t_CaptureState &capture_state, vpRect &face_bbox, bool &face_available)
#endif
{
  vpImage<unsigned char> I_;

  t_CaptureState capture_state_;
  bool display_initialized_ = false;
  bool face_available_ = false;
  vpRect face_bbox_;
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
#if defined(VISP_HAVE_V4L2)
        I_ = frame;
#elif defined(VISP_HAVE_OPENCV)
        vpImageConvert::convert(frame, I_);
#endif
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

      // Check if a face was detected
      {

        std::lock_guard<std::mutex> lock(mutex_face);
        face_available_ = face_available;
        face_bbox_ = face_bbox;
      }
      if (face_available_) {
        // Access to the face bounding box to display it
        vpDisplay::displayRectangle(I_, face_bbox_, vpColor::green, false, 4);
        face_available_ = false;
      }

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

//! [face-detection-threaded detectionFunction]
#if defined(VISP_HAVE_V4L2)
void detectionFunction(std::mutex &mutex_capture, std::mutex &mutex_face, vpImage<unsigned char> &frame, t_CaptureState &capture_state, vpRect &face_bbox, std::string &face_cascade_name, bool &face_available)
#elif defined(HAVE_OPENCV_VIDEOIO)
void detectionFunction(std::mutex &mutex_capture, std::mutex &mutex_face, cv::Mat &frame, t_CaptureState &capture_state, vpRect &face_bbox, std::string &face_cascade_name, bool &face_available)
#endif
{
  vpDetectorFace face_detector_;
  face_detector_.setCascadeClassifierFile(face_cascade_name);

  t_CaptureState capture_state_;
#if defined(VISP_HAVE_V4L2)
  vpImage<unsigned char> frame_;
#elif defined(VISP_HAVE_OPENCV)
  cv::Mat frame_;
#endif
  do {
    mutex_capture.lock();
    capture_state_ = capture_state;
    mutex_capture.unlock();

    // Check if a frame is available
    if (capture_state_ == capture_started) {
      // Backup the frame
      {
        std::lock_guard<std::mutex> lock(mutex_capture);
        frame_ = frame;
      }

      // Detect faces
      bool face_found_ = face_detector_.detect(frame_);
      if (face_found_) {
        std::lock_guard<std::mutex> lock(mutex_face);
        face_available = true;
        face_bbox = face_detector_.getBBox(0); // Get largest face bounding box
      }
    }
    else {
      vpTime::wait(2); // Sleep 2ms
    }
  } while (capture_state_ != capture_stopped);
  std::cout << "End of face detection thread" << std::endl;
}
//! [face-detection-threaded detectionFunction]

//! [face-detection-threaded mainFunction]
int main(int argc, const char *argv[])
{
  std::string opt_face_cascade_name = "./haarcascade_frontalface_alt.xml";
  unsigned int opt_device = 0;
  unsigned int opt_scale = 2; // Default value is 2 in the constructor. Turn
  // it to 1 to avoid subsampling

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--haar")
      opt_face_cascade_name = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--device")
      opt_device = (unsigned int)atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--scale")
      opt_scale = (unsigned int)atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--help") {
      std::cout << "Usage: " << argv[0]
        << " [--haar <haarcascade xml filename>] [--device <camera "
        "device>] [--scale <subsampling factor>] [--help]"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  // Instantiate the capture
#if defined(VISP_HAVE_V4L2)
  vpImage<unsigned char> frame;
  vpV4l2Grabber cap;
  std::ostringstream device;
  device << "/dev/video" << opt_device;
  cap.setDevice(device.str());
  cap.setScale(opt_scale);
#elif defined(HAVE_OPENCV_VIDEOIO)
  cv::Mat frame;
  cv::VideoCapture cap;
  cap.open(opt_device);
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
  int width = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
  int height = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
  cap.set(cv::CAP_PROP_FRAME_WIDTH, width / opt_scale);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height / opt_scale);
#else
  int width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
  int height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
  cap.set(CV_CAP_PROP_FRAME_WIDTH, width / opt_scale);
  cap.set(CV_CAP_PROP_FRAME_HEIGHT, height / opt_scale);
#endif
#endif

  std::mutex mutex_capture;
  std::mutex mutex_face;
  vpRect face_bbox;
  t_CaptureState capture_state = capture_waiting;
  bool face_available = false;

  // Start the threads
  std::thread thread_capture(&captureFunction, std::ref(cap), std::ref(mutex_capture), std::ref(frame), std::ref(capture_state));
  std::thread thread_display(&displayFunction, std::ref(mutex_capture), std::ref(mutex_face), std::ref(frame),
                             std::ref(capture_state), std::ref(face_bbox), std::ref(face_available));
  std::thread thread_detection(&detectionFunction, std::ref(mutex_capture), std::ref(mutex_face), std::ref(frame),
                               std::ref(capture_state), std::ref(face_bbox), std::ref(opt_face_cascade_name), std::ref(face_available));

  // Wait until thread ends up
  thread_capture.join();
  thread_display.join();
  thread_detection.join();

  return EXIT_SUCCESS;
}
//! [face-detection-threaded mainFunction]

#else
int main()
{
#ifndef VISP_HAVE_OPENCV
  std::cout << "You should install OpenCV to make this example working..." << std::endl;
#elif !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif
