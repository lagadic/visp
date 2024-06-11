//! \example mbot-apriltag-2D-half-vs.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpSerial.h>
#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/robot/vpUnicycle.h>
#include <visp3/sensor/vpV4l2Grabber.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureDepth.h>
#include <visp3/visual_features/vpFeaturePoint.h>
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
  bool use_pose = true;
  bool save_image = false; // Only possible if display_on = true

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--without_pose_computation") {
      use_pose = false;
    }
    else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = std::atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      device = std::atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = std::atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--display_tag") {
      display_tag = true;
#if defined(VISP_HAVE_X11)
    }
    else if (std::string(argv[i]) == "--display_on") {
      display_on = true;
    }
    else if (std::string(argv[i]) == "--save_image") {
      save_image = true;
#endif
    }
    else if (std::string(argv[i]) == "--serial_off") {
      serial_off = true;
    }
    else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0]
        << " [--input <camera input>] [--tag_size <tag_size in m>]"
        " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
        " [--intrinsic <intrinsic file>] [--camera_name <camera name>] [--without_pose_computation]"
        " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10, 2: TAG_36ARTOOLKIT,"
        " 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5)]"
        " [--display_tag]";
#if defined(VISP_HAVE_X11)
      std::cout << " [--display_on] [--save_image]";
#endif
      std::cout << " [--serial_off] [--help]" << std::endl;
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
#ifdef VISP_HAVE_X11
    if (display_on) {
      d = new vpDisplayX(I);
    }
#endif

    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(615.1674805, 615.1675415, I.getWidth() / 2., I.getHeight() / 2.);
    vpXmlParserCamera parser;
    if (!intrinsic_file.empty() && !camera_name.empty())
      parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);

    std::cout << "cam:\n" << cam << std::endl;
    std::cout << "use pose: " << use_pose << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;

    vpDetectorAprilTag detector(tagFamily);

    detector.setAprilTagQuadDecimate(quad_decimate);
    if (use_pose)
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

    // Current and desired visual feature associated to the x coordinate of the point
    vpFeaturePoint s_x, s_xd;
    vpImagePoint cog;
    double Z, Z_d;
    Z = Z_d = 0.4;

    // Create the current x visual feature
    vpFeatureBuilder::create(s_x, cam, cog);

    // Create the desired x* visual feature
    s_xd.build(0, 0, Z_d);

    // Add the point feature
    task.addFeature(s_x, s_xd, vpFeaturePoint::selectX());

    // Create the log(Z/Z*) visual feature
    vpFeatureDepth s_Z, s_Z_d;

    std::cout << "Z " << Z << std::endl;
    s_Z.build(s_x.get_x(), s_x.get_y(), Z, 0); // log(Z/Z*) = 0 that's why the last parameter is 0
    s_Z_d.build(0, 0, Z_d, 0);                 // The value of s* is 0 with Z=1 meter

    // Add the feature
    task.addFeature(s_Z, s_Z_d);

    std::vector<double> time_vec;
    for (;;) {
      g.acquire(I);

      vpDisplay::display(I);

      double t = vpTime::measureTimeMs();
      std::vector<vpHomogeneousMatrix> cMo_vec;
      if (use_pose)
        detector.detect(I, tagSize, cam, cMo_vec);
      else
        detector.detect(I);

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
        vpDisplay::displayFrame(I, cdMo, cam, tagSize / 3, vpColor::red, 3);
        vpDisplay::displayCross(I, detector.getCog(0), 15, vpColor::green,
                                3); // Current polygon used to compure an moment
        vpDisplay::displayLine(I, 0, cam.get_u0(), I.getHeight() - 1, cam.get_u0(), vpColor::red,
                               3); // Vertical line as desired x position
        if (use_pose) {
          // Display visual features
          vpDisplay::displayFrame(I, cMo_vec[0], cam, tagSize / 2, vpColor::none, 3);
        }

        if (!serial_off) {
          serial->write("LED_RING=2,0,10,0\n"); // Switch on led 2 to green: tag detected
        }

        if (use_pose) {
          Z = cMo_vec[0][2][3];
        }
        else {
          vpPolygon polygon(detector.getPolygon(0));
          double surface = polygon.getArea();
          std::cout << "Surface: " << surface << std::endl;

          // Compute the distance from target surface and 3D size
          Z = tagSize * cam.get_px() / sqrt(surface);
        }

        vpFeatureBuilder::create(s_x, cam, detector.getCog(0));
        s_x.set_Z(Z);

        // Update log(Z/Z*) feature
        s_Z.build(s_x.get_x(), s_x.get_y(), Z, log(Z / Z_d));

        std::cout << "cog: " << detector.getCog(0) << " Z: " << Z << std::endl;

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
      if (vpDisplay::getClick(I, false))
        break;
    }

    if (!serial_off) {
      serial->write("LED_RING=0,0,0,0\n"); // Switch off all led
    }

    std::cout << "Benchmark computation time" << std::endl;
    std::cout << "Mean / Median / Std: " << vpMath::getMean(time_vec) << " ms"
      << " ; " << vpMath::getMedian(time_vec) << " ms"
      << " ; " << vpMath::getStdev(time_vec) << " ms" << std::endl;

    if (display_on)
      delete d;
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
