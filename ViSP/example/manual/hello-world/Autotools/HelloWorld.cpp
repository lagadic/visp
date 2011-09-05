#include <iostream>

#include <visp/vpDebug.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>


int main()
{
  std::cout << "ViSP Hello World example" <<std::endl;

  vpImage<unsigned char> I(288, 384);

  I = 128;

  std::cout << "ViSP creates \"./myimage.pgm\" B&W image " <<std::endl;
  vpImageIo::writePGM(I, "./myimage.pgm");

  return 0;
}
