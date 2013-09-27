/*! \example tutorial-image-viewer.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>

int main()
{
  try {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, "lena.ppm");

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif
    vpDisplay::setTitle(I, "Lena");
    vpDisplay::display(I);

    vpDisplay::displayRectangle(I, vpImagePoint(90,90), 70, 90, vpColor::red, false, 2);
    vpDisplay::flush(I);

    vpImage<vpRGBa> O;
    vpDisplay::getImage(I, O);

    try {
      vpImageIo::write(I, "lena-out.jpg");
      vpImageIo::write(O, "lena-out-with-overlay.jpg2");
    }
    catch(...) {
      std::cout << "Cannot write the image: unsupported format..." << std::endl;
    }

    vpDisplay::getClick(I);
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
