//! \example tutorial-image-filter.cpp

#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageFilter.h>

void display(vpImage<unsigned char> &I, const std::string &title);
void display(vpImage<double> &D, const std::string &title);

void display(vpImage<unsigned char> &I, const std::string &title)
{
#if defined(VISP_HAVE_X11)
  vpDisplayX d(I);
#elif defined(VISP_HAVE_OPENCV)
  vpDisplayOpenCV d(I);
#elif defined(VISP_HAVE_GTK)
  vpDisplayGTK d(I);
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI d(I);
#elif defined(VISP_HAVE_D3D9)
  vpDisplayD3d d(I);
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif

  vpDisplay::setTitle(I, title.c_str());
  vpDisplay::display(I);
  vpDisplay::displayText(I, 15,15, "Click to continue...", vpColor::red);
  vpDisplay::flush(I);
  vpDisplay::getClick(I);
}

void display(vpImage<double> &D, const std::string &title)
{
  vpImage<unsigned char> I; // Image to display
  vpImageConvert::convert(D, I);
  display(I, title);
}

int main(int argc, char** argv )
{
  try {
    if(argc != 2) {
      printf( "Usage: %s <image name.[pgm,ppm,jpeg,png,bmp]>\n", argv[0] );
      return -1;
    }
    //! [vpImage construction]
    vpImage<unsigned char> I;
    //! [vpImage construction]

    try {
      vpImageIo::read(I, argv[1]);
    }
    catch(...) {
      std::cout << "Cannot read image \"" << argv[1] << "\"" << std::endl;
      return -1;
    }

    display(I, "Original image");

    //! [Gaussian blur]
    vpImage<double> F;
    vpImageFilter::gaussianBlur(I, F);
    //! [Gaussian blur]
    display(F, "Blur (default)");

    vpImageFilter::gaussianBlur(I, F, 7, 2);
    display(F, "Blur (var=2)");

    //! [Gradients x]
    vpImage<double> dIx;
    vpImageFilter::getGradX(I, dIx);
    //! [Gradients x]
    display(dIx, "Gradient dIx");

    //! [Gradients y]
    vpImage<double> dIy;
    vpImageFilter::getGradY(I, dIy);
    //! [Gradients y]
    display(dIy, "Gradient dIy");

    //! [Canny]
#if (VISP_HAVE_OPENCV_VERSION >= 0x020100)
    vpImage<unsigned char> C;
    vpImageFilter::canny(I, C, 5, 15, 3);
    display(C, "Canny");
#endif
    //! [Canny]

    //! [Convolution kernel]
    vpMatrix K(3,3); // Sobel kernel along x
    K[0][0] = 1; K[0][1] = 0; K[0][2] = -1;
    K[1][0] = 2; K[1][1] = 0; K[1][2] = -2;
    K[2][0] = 1; K[2][1] = 0; K[2][2] = -1;
    //! [Convolution kernel]
    //! [Convolution]
    vpImage<double> Gx;
    vpImageFilter::filter(I, Gx, K);
    //! [Convolution]
    display(Gx, "Sobel x");

    //! [Gaussian pyramid]
    size_t nlevel = 3;
    std::vector< vpImage<unsigned char> > pyr(nlevel);
    pyr[0] = I;
    for (size_t i=1; i < nlevel; i++) {
      vpImageFilter::getGaussPyramidal(pyr[i-1], pyr[i]);
      display(pyr[i], "Pyramid");
    }
    //! [Gaussian pyramid]
    return 0;
  }
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
