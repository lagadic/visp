/*! \example tutorial-image-viewer.cpp */
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>

int main()
{
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  vpImage<vpRGBa> I;
  vpImageIo::read(I, "lena.jpg");

#ifdef UNIX
  vpDisplayX d(I);
#elif WIN32
  vpDisplayGDI d(I);
#endif
  vpDisplay::setTitle(I, "Lena");
  vpDisplay::display(I);

  vpDisplay::displayRectangle(I, vpImagePoint(90,90), 70, 90, vpColor::red, false, 2);
  vpDisplay::flush(I);

  vpImage<vpRGBa> O;
  vpDisplay::getImage(I, O);
#ifdef VISP_HAVE_LIBJPEG
  vpImageIo::write(I, "lena-out.jpg");
  vpImageIo::write(O, "lena-out-with-overlay.jpg");
#endif

  vpDisplay::getClick(I);
#endif
}
