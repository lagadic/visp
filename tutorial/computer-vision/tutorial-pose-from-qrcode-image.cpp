/*! \example tutorial-pose-from-qrcode-image.cpp */
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/vision/vpPose.h>
#include <visp3/detection/vpDetectorQRCode.h>

void computePose(std::vector<vpPoint> &point, const std::vector<vpImagePoint> &ip,
                 const vpCameraParameters &cam, bool init, vpHomogeneousMatrix &cMo)
{
  vpPose pose;     double x=0, y=0;
  for (unsigned int i=0; i < point.size(); i ++) {
    vpPixelMeterConversion::convertPoint(cam, ip[i], x, y);
    point[i].set_x(x);
    point[i].set_y(y);
    pose.addPoint(point[i]);
  }

  if (init == true) {
    vpHomogeneousMatrix cMo_dem;
    vpHomogeneousMatrix cMo_lag;
    pose.computePose(vpPose::DEMENTHON, cMo_dem);
    pose.computePose(vpPose::LAGRANGE, cMo_lag);
    double residual_dem = pose.computeResidual(cMo_dem);
    double residual_lag = pose.computeResidual(cMo_lag);
    if (residual_dem < residual_lag)
      cMo = cMo_dem;
    else
      cMo = cMo_lag;
  }
  pose.computePose(vpPose::VIRTUAL_VS, cMo);
}

int main()
{
#if defined(VISP_HAVE_ZBAR)
  try {
    vpImage<unsigned char> I;
    vpImageIo::read(I, "bar-code.pgm");

#if defined(VISP_HAVE_X11)
    vpDisplayX d(I);
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d(I);
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d(I);
#endif

    // Camera parameters should be adapted to your camera
    vpCameraParameters cam(840, 840, I.getWidth()/2, I.getHeight()/2);

    // 3D model of the QRcode: here we consider a 12cm by 12cm QRcode
    std::vector<vpPoint> point;
    point.push_back( vpPoint(-0.06, -0.06, 0) ); // QCcode point 0 3D coordinates in plane Z=0
    point.push_back( vpPoint( 0.06, -0.06, 0) ); // QCcode point 1 3D coordinates in plane Z=0
    point.push_back( vpPoint( 0.06,  0.06, 0) ); // QCcode point 2 3D coordinates in plane Z=0
    point.push_back( vpPoint(-0.06,  0.06, 0) ); // QCcode point 3 3D coordinates in plane Z=0

    vpHomogeneousMatrix cMo;
    bool init = true;

    vpDetectorQRCode detector;

    while(1) {
      vpImageIo::read(I, "bar-code.pgm");
      vpDisplay::display(I);

      bool status = detector.detect(I);

      std::ostringstream legend;
      legend << detector.getNbObjects() << " bar code detected";
      vpDisplay::displayText(I, (int)I.getHeight()-30, 10, legend.str(), vpColor::red);

      if (status) { // true if at least one QRcode is detected
        for(size_t i=0; i < detector.getNbObjects(); i++) {

          std::vector<vpImagePoint> p = detector.getPolygon(i); // get the four corners location in the image

          for(size_t j=0; j < p.size(); j++) {
            vpDisplay::displayCross(I, p[j], 14, vpColor::red, 3);
            std::ostringstream number;
            number << j;
            vpDisplay::displayText(I, p[j]+vpImagePoint(15,5), number.str(), vpColor::blue);
          }

          computePose(point, p, cam, init, cMo); // resulting pose is available in cMo var
          std::cout << "Pose translation (meter): " << cMo.getTranslationVector().t() << std::endl
                    << "Pose rotation (quaternion): " << vpQuaternionVector(cMo.getRotationMatrix()).t() << std::endl;
          vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        }
      }
      vpDisplay::displayText(I, (int)I.getHeight()-15, 10, "A click to quit...", vpColor::red);
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false)) break;

      vpTime::wait(40);
    }
  }
  catch(const vpException &e) {
    std::cout << "Catch an exception: " << e.getMessage() << std::endl;
  }
#else
  std::cout << "ViSP is not build with zbar 3rd party." << std::endl;
#endif
}
