//! \example tutorial-franka-acquire-calib-data.cpp
#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/sensor/vpRealSense2.h>
#include <visp3/robot/vpRobotFranka.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_FRANKA)

int main(int argc, char **argv)
{
  try {
    std::string robot_ip = "192.168.1.1";

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
        robot_ip = std::string(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << argv[0] << " [--ip 192.168.1.1] [--help] [-h]"
                             << "\n";
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I;

    vpRobotFranka robot;
    robot.connect(robot_ip);

    vpRealSense2 g;
    rs2::config config;
    config.disable_stream(RS2_STREAM_DEPTH);
    config.disable_stream(RS2_STREAM_INFRARED);
    config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
    g.open(config);
    g.acquire(I);

    unsigned int width = I.getWidth();
    unsigned int height = I.getHeight();

    std::cout << "Image size: " << width << " x " << height << std::endl;
    // Save intrinsics
    vpCameraParameters cam;
    vpXmlParserCamera xml_camera;
    cam = g.getCameraParameters(RS2_STREAM_COLOR, vpCameraParameters::perspectiveProjWithDistortion);
    xml_camera.save(cam, "camera.xml", "Camera", width, height);

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    bool end = false;
    unsigned cpt = 0;
    while (! end) {
      g.acquire(I);

      vpDisplay::display(I);

      vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
      vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          cpt ++;

          vpPoseVector fPe;
          robot.getPosition(vpRobot::END_EFFECTOR_FRAME, fPe);

          std::stringstream ss_img, ss_pos;

          ss_img << "image-" << cpt << ".png";
          ss_pos << "pose_fPe_" << cpt << ".yaml";
          std::cout << "Save: " << ss_img.str() << " and " << ss_pos.str() << std::endl;
          vpImageIo::write(I, ss_img.str());
          fPe.saveYAML(ss_pos.str(), fPe);
        }
        else if (button == vpMouseButton::button3) {
          end = true;
        }
      }
      vpDisplay::flush(I);

    }
  } catch (const vpException &e) {
    std::cerr << "RealSense error " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}
#else
int main()
{
#if !defined(VISP_HAVE_REALSENSE2)
  std::cout << "Install librealsense-2.x." << std::endl;
#endif
#if !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::cout << "Build ViSP with c++11 or higher compiler flag (cmake -DUSE_CXX_STANDARD=11)." << std::endl;
#endif
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka." << std::endl;
#endif

  std::cout << "After installation of the missing 3rd parties, configure ViSP with cmake"
            << " and build ViSP again." << std::endl;
  return 0;
}
#endif
