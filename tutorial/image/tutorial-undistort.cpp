//! \example tutorial-undistort.cpp
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpXmlParserCamera.h>

int main()
{
  try {
    //! [Load image]
    vpImage<unsigned char> I;
    vpImageIo::read(I, "chessboard.pgm");
    //! [Load image]

    //! [Load camera parameters from xml]
    vpCameraParameters cam;
#ifdef VISP_HAVE_XML2
    vpXmlParserCamera p;
    vpCameraParameters::vpCameraParametersProjType projModel;
    projModel = vpCameraParameters::perspectiveProjWithDistortion;
    if (p.parse(cam, "camera.xml", "Camera", projModel, I.getWidth(), I.getHeight()) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Cannot found parameters for camera named \"Camera\"" << std::endl;
    }
    //! [Load camera parameters from xml]
    //! [Set camera parameters]
#else
    cam.initPersProjWithDistortion(582.7, 580.6, 326.6, 215.0, -0.3372, 0.4021);
#endif
    //! [Set camera parameters]

    std::cout << cam << std::endl;

    //! [Create image without distorsion]
    vpImage<unsigned char> Iud;
    vpImageTools::undistort(I, cam, Iud);
    vpImageIo::write(Iud, "chessboard-undistort.pgm");
    //! [Create image without distorsion]
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

  return 0;
}
