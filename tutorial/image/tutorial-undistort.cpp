//! \example tutorial-undistort.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/io/vpImageIo.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_input_image = "chessboard.jpg";
  std::string opt_camera_file = "camera.xml";
  std::string opt_camera_name = "Camera";

  try {
    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--image" && i + 1 < argc) {
        opt_input_image = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--camera-file" && i + 1 < argc) {
        opt_camera_file = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
        opt_camera_name = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << argv[0] << " [--image <input image (pgm,ppm,jpeg,png,tiff,bmp,ras,jp2)>]"
          << " [--camera-file <xml file>] [--camera-name <name>] [--help] [-h]\n"
          << std::endl;
        std::cout << "Examples: " << std::endl
          << argv[0] << std::endl
          << argv[0] << " --image chessboard.jpg --camera-file camera.xml --camera-name Camera" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    //! [Load image]
    vpImage<unsigned char> I;
    std::cout << "Read input image: " << opt_input_image << std::endl;
    vpImageIo::read(I, opt_input_image);
    //! [Load image]


    vpCameraParameters cam;
#if defined(VISP_HAVE_PUGIXML)
    //! [Load camera parameters from xml]
    vpXmlParserCamera p;
    vpCameraParameters::vpCameraParametersProjType projModel;
    projModel = vpCameraParameters::perspectiveProjWithDistortion;
    if (p.parse(cam, opt_camera_file, opt_camera_name, projModel, I.getWidth(), I.getHeight()) !=
        vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Cannot found parameters for camera named \"Camera\"" << std::endl;
    }
    //! [Load camera parameters from xml]
#else
    //! [Set camera parameters]
    cam.initPersProjWithDistortion(582.7, 580.6, 326.6, 215.0, -0.3372, 0.4021);
    //! [Set camera parameters]
#endif

    std::cout << cam << std::endl;

    //! [Create image without distortion]
    vpImage<unsigned char> Iud;
    vpImageTools::undistort(I, cam, Iud);
    std::string name_we = vpIoTools::getNameWE(opt_input_image);
    std::string ext = vpIoTools::getFileExtension(opt_input_image);
    std::string output_image = name_we + "-undistort" + ext;
    std::cout << "Save undistorted image in: " << output_image << std::endl;
    vpImageIo::write(Iud, output_image);
    //! [Create image without distortion]
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
