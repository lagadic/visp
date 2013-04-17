/*! \example tutorial-image-converter.cpp */
#include <visp/vpImageIo.h>
#include <visp/vpImageConvert.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  cv::Mat A;
  A = cv::imread("lena.bmp", CV_LOAD_IMAGE_GRAYSCALE);

  vpImage<unsigned char> I;
  vpImageConvert::convert(A, I);

#  ifdef VISP_HAVE_LIBPNG
  vpImageIo::write(I, "lena.png"); // Gray
#  endif
#endif
}
