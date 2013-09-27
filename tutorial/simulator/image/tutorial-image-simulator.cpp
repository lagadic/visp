/*! \example tutorial-image-simulator.cpp */
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageSimulator.h>

int main()
{
  try {
    vpImage<unsigned char> target;
    vpImageIo::read(target, "./target_square.pgm");

    vpColVector X[4];
    for (int i = 0; i < 4; i++) X[i].resize(3);
    // Top left     Top right       Bottom right   Bottom left
    X[0][0] = -0.1; X[1][0] =  0.1; X[2][0] = 0.1; X[3][0] = -0.1;
    X[0][1] = -0.1; X[1][1] = -0.1; X[2][1] = 0.1; X[3][1] =  0.1;
    X[0][2] =  0;   X[1][2] =  0;   X[2][2] = 0;   X[3][2] =  0;

    vpImage<unsigned char> I(480, 640);
    vpCameraParameters cam(840, 840, I.getWidth()/2, I.getHeight()/2);
    vpHomogeneousMatrix cMo(0, 0, 0.35, 0, vpMath::rad(30), vpMath::rad(15));

    vpImageSimulator sim;
    sim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
    sim.init(target, X);

    // Get the new image of the projected planar image target
    sim.setCleanPreviousImage(true);
    sim.setCameraPosition(cMo);
    sim.getImage(I, cam);

    try {
      vpImageIo::write(I, "./rendered_image.jpg");
    }
    catch(...) {
      std::cout << "Unsupported image format" << std::endl;
    }

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpDisplay::setTitle(I, "Planar image projection");
    vpDisplay::display(I);
    vpDisplay::flush(I);
    std::cout << "A click to quit..." << std::endl;
    vpDisplay::getClick(I);
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
}
