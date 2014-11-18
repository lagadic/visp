/*! \example tutorial-image-converter.cpp */
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)
  try {
    cv::Mat A;
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    A = cv::imread("lena.bmp", cv::IMREAD_GRAYSCALE);
#else
    A = cv::imread("lena.bmp", CV_LOAD_IMAGE_GRAYSCALE);
#endif

    vpImage<unsigned char> I;
    vpImageConvert::convert(A, I);

#  ifdef VISP_HAVE_LIBPNG
    vpImageIo::write(I, "lena.png"); // Gray
#  endif
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#endif
}
