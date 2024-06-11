#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpImageIo.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::cout << "ViSP Hello World example" << std::endl;

  vpImage<unsigned char> I(288, 384);

  I = 128;

  std::cout << "ViSP creates \"./myimage.pgm\" B&W image " << std::endl;
  vpImageIo::write(I, "./myimage.pgm");

  return EXIT_SUCCESS;
}
