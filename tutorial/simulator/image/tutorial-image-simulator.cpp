//! \example tutorial-image-simulator.cpp
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpImageIo.h>
//! [Include]
#include <visp/vpImageSimulator.h>
//! [Include]

int main()
{
  try {
    //! [Read image]
    vpImage<unsigned char> target;
    vpImageIo::read(target, "./target_square.pgm");
    //! [Read image]

    //! [Set model]
    vpColVector X[4];
    for (int i = 0; i < 4; i++) X[i].resize(3);
    // Top left     Top right       Bottom right   Bottom left
    X[0][0] = -0.1; X[1][0] =  0.1; X[2][0] = 0.1; X[3][0] = -0.1;
    X[0][1] = -0.1; X[1][1] = -0.1; X[2][1] = 0.1; X[3][1] =  0.1;
    X[0][2] =  0;   X[1][2] =  0;   X[2][2] = 0;   X[3][2] =  0;
    //! [Set model]

    //! [Image construction]
    vpImage<unsigned char> I(480, 640);
    //! [Image construction]
    //! [Camera parameters]
    vpCameraParameters cam(840, 840, I.getWidth()/2, I.getHeight()/2);
    //! [Camera parameters]
    //! [Set cMo]
    vpHomogeneousMatrix cMo(0, 0, 0.35, 0, vpMath::rad(30), vpMath::rad(15));
    //! [Set cMo]

    //! [Create simulator]
    vpImageSimulator sim;
    sim.setInterpolationType(vpImageSimulator::BILINEAR_INTERPOLATION);
    sim.init(target, X);
    //! [Create simulator]

    // Get the new image of the projected planar image target
    //! [Render image]
    sim.setCleanPreviousImage(true);
    sim.setCameraPosition(cMo);
    sim.getImage(I, cam);
    //! [Render image]

    //! [Write image]
    try {
      vpImageIo::write(I, "./rendered_image.jpg");
    }
    catch(...) {
      std::cout << "Unsupported image format" << std::endl;
    }
    //! [Write image]

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
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
