//! \example tutorial-draw-frame.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/core/vpHomogeneousMatrix.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  vpImage<unsigned char> I(2160, 3840, 128);

    // Initialize the display with the image I. Display and image are now linked together
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> d = vpDisplayFactory::createDisplay(I, -1, -1, "Frame drawing", vpDisplay::SCALE_AUTO);
#else
  vpDisplay *d = vpDisplayFactory::allocateDisplay(I, -1, -1, "Frame drawing", vpDisplay::SCALE_AUTO);
#endif

  vpDisplay::display(I);

  double dTheta = 45;
  double dOffset = 0.25;
  std::string axisName[3] = { "x", "y", "z" };
  double px = 600; double py = 600;
  double u0 = 320; double v0 = 240;

  // Create a camera parameter container
  vpCameraParameters cam;

  // Camera initialization with a perspective projection without distortion model
  cam.initPersProjWithoutDistortion(px, py, u0, v0);

  for (unsigned int idAxis = 0; idAxis < 3; idAxis++) {
    unsigned int tOffset = 0;
    for (double theta = -180; theta < 180; theta += dTheta) {
      vpTranslationVector t(0.05, 0.25 * (idAxis + 1), 0.37);
      vpRxyzVector r(0, 0, 0);
      t[0] = t[0] + tOffset * dOffset;
      tOffset++;
      r[idAxis] = vpMath::rad(theta);
      vpHomogeneousMatrix cMo;
      cMo.buildFrom(t, vpRotationMatrix(r));
      std::stringstream ss_name;
      ss_name << "cMo_";
      ss_name << static_cast<int>(theta);
      ss_name << "_";
      ss_name << axisName[idAxis];

      //! [frame]
      vpDisplay::displayFrame(I, cMo, cam, 0.1, vpColor::none, 1, vpImagePoint(), ss_name.str(), vpColor::yellow, vpImagePoint(40, 40));
      //! [frame]
    }
  }

  vpDisplay::flush(I);
  std::cout << "A click to quit..." << std::endl;
  vpDisplay::getClick(I);

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  delete d;
#endif

#else
  std::cout << "No gui available to display an image..." << std::endl;
#endif

  return EXIT_SUCCESS;
}
