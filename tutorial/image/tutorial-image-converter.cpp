/*! \example tutorial-image-converter.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGCODECS) && defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

int main()
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_HIGHGUI) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_IMGCODECS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    cv::Mat A;
#if VISP_HAVE_OPENCV_VERSION >= 0x030200
    int flags = cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION;
#elif VISP_HAVE_OPENCV_VERSION >= 0x030000
    int flags = cv::IMREAD_GRAYSCALE;
#else
    int flags = CV_LOAD_IMAGE_GRAYSCALE;
#endif

    A = cv::imread("monkey.bmp", flags);

    vpImage<unsigned char> I;
    vpImageConvert::convert(A, I);

#ifdef VISP_HAVE_PNG
    vpImageIo::write(I, "monkey.png"); // Gray
#endif
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
