/*! \example tutorial-grabber-opencv.cpp */
#include <stdlib.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayOpenCV.h>

// usage: binary <device name>
// device name: 0 is the default to dial with the first camera,
// 1 to dial with a second camera attached to the computer
int main(int argc, char **argv)
{
  int device = 0;
  if (argc > 1)
    device = atoi(argv[1]);
  std::cout << "Use device: " << device << std::endl;

#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  try {
    cv::VideoCapture cap(device); // open the default camera
    if (!cap.isOpened()) {        // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return -1;
    }
    cv::Mat frame;
    int i = 0;
    while ((i++ < 100) && !cap.read(frame)) {
    }; // warm up camera by skiping unread frames

    std::cout << "Image size: " << frame.rows << " " << frame.cols << std::endl;

    // vpImage<vpRGBa> I; // for color images
    vpImage<unsigned char> I; // for gray images
    vpImageConvert::convert(frame, I);

    vpDisplayOpenCV d(I);

    for (;;) {
      cap >> frame; // get a new frame from camera
      // Convert the image in ViSP format and display it
      vpImageConvert::convert(frame, I);
      vpDisplay::display(I);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) // a click to exit
        break;
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
