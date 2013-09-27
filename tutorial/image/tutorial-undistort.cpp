/*! \example tutorial-undistort.cpp */
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>
#include <visp/vpXmlParserCamera.h>

int main()
{
  try {
    vpImage<unsigned char> I;
    vpImageIo::read(I, "chessboard.pgm");

    vpCameraParameters cam;
#ifdef VISP_HAVE_XML2
    vpXmlParserCamera p;
    vpCameraParameters::vpCameraParametersProjType projModel;
    projModel = vpCameraParameters::perspectiveProjWithDistortion;
    if (p.parse(cam, "camera.xml", "Camera", projModel, I.getWidth(), I.getHeight()) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Cannot found parameters for camera named \"Camera\"" << std::endl;
    }
#else
    cam.initPersProjWithDistortion(582.7, 580.6, 326.6, 215.0, -0.3372, 0.4021);
#endif

    std::cout << cam << std::endl;

    vpImage<unsigned char> Iud;
    vpImageTools::undistort(I, cam, Iud);
    vpImageIo::write(Iud, "chessboard-undistort.pgm");
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

  return 0;
}
