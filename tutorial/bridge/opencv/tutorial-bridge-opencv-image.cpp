//! \example tutorial-bridge-opencv-image.cpp
#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_IMGCODECS)
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
    vpImageIo::read(Irgba, "monkey.jpeg");
    //! [Load ViSP color image]

    //! [Load ViSP grey image]
    vpImage<unsigned char> Igrey;
    vpImageIo::read(Igrey, "monkey.jpeg");
    //! [Load ViSP grey image]

    //! [Convert to OpenCV color image]
    cv::Mat cv_img_color;
    vpImageConvert::convert(Irgba, cv_img_color);
    //! [Convert to OpenCV color image]

    //! [Convert to OpenCV grey image]
    cv::Mat cv_img_grey;
    vpImageConvert::convert(Igrey, cv_img_grey);
    //! [Convert to OpenCV grey image]

    std::cout << "Save converted images from vpImage to cv::Mat" << std::endl;
    std::cout << "- monkey-cv-color.jpeg" << std::endl;
    std::cout << "- monkey-cv-grey.jpeg" << std::endl;
    //! [Save OpenCV color image]
    cv::imwrite("monkey-cv-color.jpeg", cv_img_color);
    //! [Save OpenCV color image]
    //! [Save OpenCV grey image]
    cv::imwrite("monkey-cv-grey.jpeg", cv_img_grey);
    //! [Save OpenCV grey image]
  }

  // From OpenCV to ViSP conversion
  {
    //! [Load OpenCV color image]
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

    //! [Load OpenCV grey image]
#if VISP_HAVE_OPENCV_VERSION >= 0x030000
    cv::Mat cv_img_grey = cv::imread("monkey.jpeg", cv::IMREAD_GRAYSCALE);
#else
    cv::Mat cv_img_grey = cv::imread("monkey.jpeg", CV_LOAD_IMAGE_GRAYSCALE);
#endif
    //! [Load OpenCV grey image]

    //! [Convert to ViSP grey image]
    vpImage<unsigned char> Igrey;
    vpImageConvert::convert(cv_img_grey, Igrey);
    //! [Convert to ViSP grey image]

    std::cout << "Save converted images from cv::Mat to vpImage" << std::endl;
    std::cout << "- monkey-vp-color.jpeg" << std::endl;
    std::cout << "- monkey-vp-grey.jpeg" << std::endl;
    //! [Save ViSP color image]
    vpImageIo::write(Irgba, "monkey-vp-color.jpeg");
    //! [Save ViSP color image]
    //! [Save ViSP grey image]
    vpImageIo::write(Igrey, "monkey-vp-grey.jpeg");
    //! [Save ViSP grey image]
  }
}
#else
int main()
{
  std::cout << "This tutorial required OpenCV imgproc and imgcodecs modules." << std::endl;
  return EXIT_SUCCESS;
}
#endif
