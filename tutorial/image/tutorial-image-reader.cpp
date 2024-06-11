/*! \example tutorial-image-reader.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    vpImage<vpRGBa> I;
    vpImageIo::read(I, "monkey.jpeg");
    vpImageIo::write(I, "monkey.png");
  }
  catch (const vpException &e) {
    std::cout << e.getMessage() << std::endl;
  }
  catch (...) {
    std::cout << "Unsupported image format" << std::endl;
  }
}
