/*! \example tutorial-image-reader.cpp */
#include <visp/vpImageIo.h>

int main()
{
  try {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, "lena.jpg");
    vpImageIo::write(I, "lena.png");
  }
  catch(vpException e) {
    std::cout << e.getMessage() << std::endl;
  }
  catch(...) {
    std::cout << "Unsupported image format" << std::endl;
  }
}

