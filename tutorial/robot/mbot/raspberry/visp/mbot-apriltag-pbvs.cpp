//! \example mbot-apriltag-pbvs.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpSerial.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpUnicycle.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/visual_features/vpFeaturePoint3D.h>
#include <visp3/vs/vpServo.h>

int main(int argc, const char **argv)
{
#if defined(VISP_HAVE_APRILTAG) && defined(VISP_HAVE_V4L2)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  int device = 0;
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.065;
  float quad_decimate = 4.0;
  int nThreads = 2;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  bool display_tag = false;
  bool display_on = false;
  bool serial_off = false;
  bool save_image = false; // Only possible if display_on = true

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--tag-size" && i + 1 < argc) {
      tagSize = std::atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      device = std::atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--quad-decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = std::atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--camera-name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--display-tag") {
      display_tag = true;
#if defined(VISP_HAVE_DISPLAY)
    }
    else if (std::string(argv[i]) == "--display-on") {
      display_on = true;
    }
    else if (std::string(argv[i]) == "--save-image") {
      save_image = true;
#endif
    }
    else if (std::string(argv[i]) == "--serial-off") {
      serial_off = true;
    }
    else if (std::string(argv[i]) == "--tag-family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <camera input>] [--tag-size <tag size in m>]"
        " [--quad-decimate <quad decimate>] [--nthreads <nb>]"
        " [--intrinsic <intrinsic file>] [--camera-name <camera name>]"
        " [--tag-family <family> (0: TAG_36h11, 1: TAG_36h10, 2: TAG_36ARTOOLKIT, 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5)]"
        " [--display-tag]";
#if defined(VISP_HAVE_DISPLAY)
      std::cout << " [--display-on] [--save-image]";
#endif
      std::cout << " [--serial-off] [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  // Me Auriga led ring
  // if serial com ok: led 1 green
  // if exception: led 1 red
  // if tag detected: led 2 green, else led 2 red
  // if motor left: led 3 blue
  // if motor right: led 4 blue

  vpSerial *serial = nullptr;
  if (!serial_off) {
    serial = new vpSerial("/dev/ttyAMA0", 115200);

    serial->write("LED_RING=0,0,0,0\n");  // Switch off all led
    serial->write("LED_RING=1,0,10,0\n"); // Switch on led 1 to green: serial ok
  }

  try {
    vpImage<unsigned char> I;

    vpV4l2Grabber g;
    std::ostringstream device_name;
    device_name << "/dev/video" << device;
    g.setDevice(device_name.str());
    g.setScale(1);
    g.acquire(I);

    vpDisplay *d = nullptr;
    vpImage<vpRGBa> O;
#ifdef VISP_HAVE_DISPLAY
    if (display_on) {
      d = vpDisplayFactory::displayFactory(I);
    }
#endif

    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, I.getWidth() / 2., I.getHeight() / 2.);

#if defined(VISP_HAVE_PUGIXML)
    vpXmlParserCamera parser;
    if (!intrinsic_file.empty() && !camera_name.empty()) {
      parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
    }
#endif

    std::cout << "cam:\n" << cam << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;

    vpDetectorAprilTag detector(tagFamily);

    detector.setAprilTagQuadDecimate(quad_decimate);
    detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
    detector.setAprilTagNbThreads(nThreads);
    detector.setDisplayTag(display_tag);

    vpServo task;
    vpAdaptiveGain lambda;
    if (display_on)
      lambda.initStandard(2.5, 0.4, 30); // lambda(0)=2.5, lambda(oo)=0.4 and lambda'(0)=30
    else
      lambda.initStandard(4, 0.4, 30); // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30

    vpUnicycle robot;
    task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
    task.setInteractionMatrixType(vpServo::CURRENT, vpServo::PSEUDO_INVERSE);
    task.setLambda(lambda);
    vpRotationMatrix cRe;
    cRe[0][0] = 0;
    cRe[0][1] = -1;
    cRe[0][2] = 0;
    cRe[1][0] = 0;
    cRe[1][1] = 0;
    cRe[1][2] = -1;
    cRe[2][0] = 1;
    cRe[2][1] = 0;
    cRe[2][2] = 0;

    vpHomogeneousMatrix cMe(vpTranslationVector(), cRe);
    vpVelocityTwistMatrix cVe(cMe);
    task.set_cVe(cVe);

    vpMatrix eJe(6, 2, 0);
    eJe[0][0] = eJe[5][1] = 1.0;

    std::cout << "eJe: \n" << eJe << std::endl;

    // Desired distance to the target
    double Z_d = 0.4;
    double X = 0, Y = 0, Z = Z_d;

    // Create X_3D visual features
    vpFeaturePoint3D s_XZ, s_XZ_d;
    s_XZ.build(0, 0, Z_d);
    s_XZ_d.build(0, 0, Z_d);

    // Create Point 3D X, Z coordinates visual features
    s_XZ.build(X, Y, Z);
    s_XZ_d.build(0, 0, Z_d); // The value of s* is X=Y=0 and Z=Z_d meter

    // Add the features
    task.addFeature(s_XZ, s_XZ_d, vpFeaturePoint3D::selectX() | vpFeaturePoint3D::selectZ());

    std::vector<double> time_vec;
    for (;;) {
      g.acquire(I);

      vpDisplay::display(I);

      double t = vpTime::measureTimeMs();
      std::vector<vpHomogeneousMatrix> cMo_vec;
      detector.detect(I, tagSize, cam, cMo_vec);

      t = vpTime::measureTimeMs() - t;
      time_vec.push_back(t);

      {
        std::stringstream ss;
        ss << "Detection time: " << t << " ms";
        vpDisplay::displayText(I, 40, 20, ss.str(), vpColor::red);
      }

      if (detector.getNbObjects() == 1) {
        // Display visual features
        vpHomogeneousMatrix cdMo(0, 0, Z_d, 0, 0, 0);
        vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);
        vpDisplay::displayFrame(I, cdMo, cam, tagSize / 3, vpColor::red, 3);

        if (!serial_off) {
          serial->write("LED_RING=2,0,10,0\n"); // Switch on led 2 to green: tag detected
        }

        X = cMo_vec[0][0][3];
        Y = cMo_vec[0][1][3];
        Z = cMo_vec[0][2][3];

        // Update Point 3D feature
        s_XZ.set_XYZ(X, Y, Z);

        std::cout << "X: " << X << " Z: " << Z << std::endl;

        task.set_cVe(cVe);
        task.set_eJe(eJe);

        // Compute the control law. Velocities are computed in the mobile robot reference frame
        vpColVector v = task.computeControlLaw();

        std::cout << "Send velocity to the mbot: " << v[0] << " m/s " << vpMath::deg(v[1]) << " deg/s" << std::endl;

        task.print();
        double radius = 0.0325;
        double L = 0.0725;
        double motor_left = (-v[0] - L * v[1]) / radius;
        double motor_right = (v[0] - L * v[1]) / radius;
        std::cout << "motor left vel: " << motor_left << " motor right vel: " << motor_right << std::endl;
        if (!serial_off) {
          //          serial->write("LED_RING=3,0,0,10\n"); // Switch on led 3 to blue: motor left servoed
          //          serial->write("LED_RING=4,0,0,10\n"); // Switch on led 4 to blue: motor right servoed
        }
        std::stringstream ss;
        double rpm_left = motor_left * 30. / M_PI;
        double rpm_right = motor_right * 30. / M_PI;
        ss << "MOTOR_RPM=" << vpMath::round(rpm_left) << "," << vpMath::round(rpm_right) << "\n";
        std::cout << "Send: " << ss.str() << std::endl;
        if (!serial_off) {
          serial->write(ss.str());
        }
      }
      else {
        // stop the robot
        if (!serial_off) {
          serial->write("LED_RING=2,10,0,0\n"); // Switch on led 2 to red: tag not detected
          //          serial->write("LED_RING=3,0,0,0\n");  // Switch on led 3 to blue: motor left not servoed
          //          serial->write("LED_RING=4,0,0,0\n");  // Switch on led 4 to blue: motor right not servoed
          serial->write("MOTOR_RPM=0,-0\n"); // Stop the robot
        }
      }

      vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
      vpDisplay::flush(I);
      if (display_on && save_image) {
        vpDisplay::getImage(I, O);
        vpImageIo::write(O, "image.png");
      }
      if (vpDisplay::getClick(I, false)) {
        break;
      }
    }

    if (!serial_off) {
      serial->write("LED_RING=0,0,0,0\n"); // Switch off all led
    }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
      << " ; " << vpMath::getMedian(time_vec) << " ms"
      << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

    if (display_on) {
      delete d;
    }
    if (!serial_off) {
      delete serial;
    }
  }
  catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
    if (!serial_off) {
      serial->write("LED_RING=1,10,0,0\n"); // Switch on led 1 to red
}
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#ifndef VISP_HAVE_V4L2
  std::cout << "ViSP is not build with v4l2 support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
  return EXIT_SUCCESS;
#endif
}
