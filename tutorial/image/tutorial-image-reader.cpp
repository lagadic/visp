/*! \example tutorial-image-reader.cpp */
#include <visp3/io/vpImageIo.h>

int main()
{
  try {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, "monkey.jpeg");
    vpImageIo::write(I, "monkey.png");
  }
  catch(vpException e) {
    std::cout << e.getMessage() << std::endl;
  }
  catch(...) {
    std::cout << "Unsupported image format" << std::endl;
  }
}

