/*! \example tutorial-image-reader.cpp */
#include <visp/vpImageIo.h>

int main()
{
#ifdef VISP_HAVE_LIBJPEG
  vpImage<vpRGBa> I;
  vpImageIo::read(I, "lena.jpg");

#  ifdef VISP_HAVE_LIBPNG
  vpImageIo::write(I, "lena.png"); // color
#  endif

#endif
}

