//! \example tutorial-universal-robots-acquire-calib-data.cpp
#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpRobotUniversalRobots.h>
#include <visp3/sensor/vpRealSense2.h>

#if defined(VISP_HAVE_REALSENSE2) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) &&                                    \
    (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)) && defined(VISP_HAVE_UR_RTDE)

int main(int argc, char **argv)
{
  try {
    std::string opt_robot_ip = "192.168.0.100";

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
        opt_robot_ip = std::string(argv[i + 1]);
      } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << argv[0] << " [--ip " << opt_robot_ip << "] [--help] [-h]" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    vpImage<unsigned char> I;

    vpRobotUniversalRobots robot;

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
    xml_camera.save(cam, "ur_camera.xml", "Camera", width, height);

#if defined(VISP_HAVE_X11)
    vpDisplayX dc(I, 10, 10, "Color image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI dc(I, 10, 10, "Color image");
#endif

    bool end = false;
    unsigned cpt = 0;
    while (!end) {
      g.acquire(I);

      vpDisplay::display(I);

      vpDisplay::displayText(I, 15, 15, "Left click to acquire data", vpColor::red);
      vpDisplay::displayText(I, 30, 15, "Right click to quit", vpColor::red);
      vpMouseButton::vpMouseButtonType button;
      if (vpDisplay::getClick(I, button, false)) {
        if (button == vpMouseButton::button1) {
          cpt++;

          vpPoseVector fPe;
          std::cout << "Connect to robot to get its position..." << std::endl;
          robot.connect(opt_robot_ip);
          robot.getPosition(vpRobot::END_EFFECTOR_FRAME, fPe);
          robot.disconnect();

          std::stringstream ss_img, ss_pos;

          ss_img << "ur_image-" << cpt << ".png";
          ss_pos << "ur_pose_fPe_" << cpt << ".yaml";
          std::cout << "Save: " << ss_img.str() << " and " << ss_pos.str() << std::endl;
          vpImageIo::write(I, ss_img.str());
          fPe.saveYAML(ss_pos.str(), fPe);
        } else if (button == vpMouseButton::button3) {
          end = true;
        }
      }
      vpDisplay::flush(I);
    }
  } catch (const vpException &e) {
    std::cerr << "ViSP exception " << e.what() << std::endl;
  } catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
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
#if !defined(VISP_HAVE_UR_RTDE)
  std::cout << "ViSP is not build with libur_rtde 3rd party used to control a robot from Universal Robots..."
            << std::endl;
#endif

  std::cout << "After installation of the missing 3rd parties, configure ViSP with cmake"
            << " and build ViSP again." << std::endl;
  return EXIT_SUCCESS;
}
#endif
