/*! \example tutorial-image-manipulation.cpp */
#include <visp/vpImage.h>

int main()
{
  vpImage<unsigned char> gray_image(240, 320);
  vpImage<vpRGBa> color_image(240, 320);

  gray_image = 128;
  vpRGBa color(255, 0, 0);
  color_image = color;

  std::cout << "Gray  image, last pixel intensity: " <<  (int)gray_image[239][319] << std::endl;
  std::cout << "Color image, last pixel Red component: " << (int)color_image[239][319].R << std::endl;
}
