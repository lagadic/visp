/*! \example tutorial-image-converter.cpp */
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageConvert.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  try {
    cv::Mat A;
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    A = cv::imread("monkey.bmp", cv::IMREAD_GRAYSCALE);
#else
    A = cv::imread("monkey.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif

    vpImage<unsigned char> I;
    vpImageConvert::convert(A, I);

#  ifdef VISP_HAVE_PNG
    vpImageIo::write(I, "monkey.png"); // Gray
#  endif
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
