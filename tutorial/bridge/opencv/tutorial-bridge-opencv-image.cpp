//! \example tutorial-bridge-opencv-image.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_IMGCODECS)

#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  // From ViSP to OpenCV conversion
  {
    //! [Load ViSP color image]
    vpImage<vpRGBa> Irgba;
    std::string image_name = "monkey.jpeg";
    std::cout << "Read color image: " << image_name << std::endl;
    vpImageIo::read(Irgba, image_name);
    //! [Load ViSP color image]

    //! [Load ViSP gray image]
    vpImage<unsigned char> Igray;
    std::cout << "Read gray image: " << image_name << std::endl;
    vpImageIo::read(Igray, image_name);
    //! [Load ViSP gray image]

    //! [Convert to OpenCV color image]
    cv::Mat cv_img_color;
    vpImageConvert::convert(Irgba, cv_img_color);
    //! [Convert to OpenCV color image]

    //! [Convert to OpenCV gray image]
    cv::Mat cv_img_gray;
    vpImageConvert::convert(Igray, cv_img_gray);
    //! [Convert to OpenCV gray image]

    std::cout << "Save converted images from vpImage to cv::Mat" << std::endl;
    std::cout << "- monkey-cv-color.jpeg" << std::endl;
    std::cout << "- monkey-cv-gray.jpeg" << std::endl;
    //! [Save OpenCV color image]
    cv::imwrite("monkey-cv-color.jpeg", cv_img_color);
    //! [Save OpenCV color image]
    //! [Save OpenCV gray image]
    cv::imwrite("monkey-cv-gray.jpeg", cv_img_gray);
    //! [Save OpenCV gray image]
  }

  // From OpenCV to ViSP conversion
  {
    //! [Load OpenCV color image]
    std::string image_name = "monkey.jpeg";
    std::cout << "Read color image: " << image_name << std::endl;
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    cv::Mat cv_img_color = cv::imread("monkey.jpeg", cv::IMREAD_COLOR);
#else
    cv::Mat cv_img_color = cv::imread("monkey.jpeg", CV_LOAD_IMAGE_COLOR);
#endif
    //! [Load OpenCV color image]

    //! [Convert to ViSP color image]
    vpImage<vpRGBa> Irgba;
    vpImageConvert::convert(cv_img_color, Irgba);
    //! [Convert to ViSP color image]

    //! [Load OpenCV gray image]
    std::cout << "Read gray image: " << image_name << std::endl;
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    cv::Mat cv_img_gray = cv::imread("monkey.jpeg", cv::IMREAD_GRAYSCALE);
#else
    cv::Mat cv_img_gray = cv::imread("monkey.jpeg", CV_LOAD_IMAGE_GRAYSCALE);
#endif
    //! [Load OpenCV gray image]

    //! [Convert to ViSP gray image]
    vpImage<unsigned char> Igray;
    vpImageConvert::convert(cv_img_gray, Igray);
    //! [Convert to ViSP gray image]

    std::cout << "Save converted images from cv::Mat to vpImage" << std::endl;
    std::cout << "- monkey-vp-color.jpeg" << std::endl;
    std::cout << "- monkey-vp-gray.jpeg" << std::endl;
    //! [Save ViSP color image]
    vpImageIo::write(Irgba, "monkey-vp-color.jpeg");
    //! [Save ViSP color image]
    //! [Save ViSP gray image]
    vpImageIo::write(Igray, "monkey-vp-gray.jpeg");
    //! [Save ViSP gray image]
  }
}
#else
int main()
{
#if !defined(HAVE_OPENCV_IMGPROC)
  std::cout << "This tutorial requires OpenCV imgproc module." << std::endl;
#endif
#if !defined(HAVE_OPENCV_IMGPROC)
  std::cout << "This tutorial requires OpenCV imgcodecs module." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
