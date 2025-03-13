/*! \example tutorial-pose-from-qrcode-image.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/detection/vpDetectorQRCode.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/vision/vpPose.h>

#include "pose_helper.h"

int main(int, char *argv[])
{
#if defined(VISP_HAVE_ZBAR)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display;
#else
  vpDisplay *display = nullptr;
#endif
  try {
    vpImage<unsigned char> I;
    vpImageIo::read(I, vpIoTools::getParent(argv[0]) + "/data/bar-code.jpg");

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
    display = vpDisplayFactory::createDisplay(I);
#else
    display = vpDisplayFactory::allocateDisplay(I);
#endif

    // Camera parameters should be adapted to your camera
    vpCameraParameters cam(840, 840, I.getWidth() / 2, I.getHeight() / 2);

    // 3D model of the QRcode: here we consider a 12cm by 12cm QRcode
    std::vector<vpPoint> point;
    point.push_back(vpPoint(-0.06, -0.06, 0)); // QRcode point 0 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, -0.06, 0));  // QRcode point 1 3D coordinates in plane Z=0
    point.push_back(vpPoint(0.06, 0.06, 0));   // QRcode point 2 3D coordinates in plane Z=0
    point.push_back(vpPoint(-0.06, 0.06, 0));  // QRcode point 3 3D coordinates in plane Z=0

    vpHomogeneousMatrix cMo;
    bool init = true;

    vpDetectorQRCode detector;

    while (1) {
      vpImageIo::read(I, vpIoTools::getParent(argv[0]) + "/data/bar-code.jpg");
      vpDisplay::display(I);

      bool status = detector.detect(I);

      std::ostringstream legend;
      legend << detector.getNbObjects() << " bar code detected";
      vpDisplay::displayText(I, (int)I.getHeight() - 30, 10, legend.str(), vpColor::red);

      if (status) { // true if at least one QRcode is detected
        for (size_t i = 0; i < detector.getNbObjects(); i++) {

          std::vector<vpImagePoint> p = detector.getPolygon(i); // get the four corners location in the image

          for (size_t j = 0; j < p.size(); j++) {
            vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            vpDisplay::displayText(I, p[j] + vpImagePoint(15, 5), number.str(), vpColor::blue);
          }

          computePose(point, p, cam, init, cMo); // resulting pose is available in cMo var
          std::cout << "Pose translation (meter): " << cMo.getTranslationVector().t() << std::endl
            << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t() << std::endl;
          vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        }
      }
      vpDisplay::displayText(I, (int)I.getHeight() - 15, 10, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false))
        break;

      vpTime::wait(40);
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
#else
  (void)argv;
  std::cout << "ViSP is not build with zbar 3rd party." << std::endl;
#endif
}
