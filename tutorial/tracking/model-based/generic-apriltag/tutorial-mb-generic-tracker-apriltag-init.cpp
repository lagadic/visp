//! \example tutorial-mb-generic-tracker-aprilTag-init.cpp
#include <fstream>
#include <ios>
#include <iostream>

#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#ifdef VISP_HAVE_XML2
#include <visp3/core/vpXmlParserCamera.h>
#endif
#ifdef VISP_HAVE_V4L2
#include <visp3/sensor/vpV4l2Grabber.h>
#endif
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpUniRand.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpCalibration.h>

// creates a cube.cao file in your current directory
// cubeEdgeSize : size of cube edges in meters
void createCaoFile(double cubeEdgeSize)
{
  std::ofstream fileStream;
  fileStream.open("cube.cao", std::ofstream::out | std::ofstream::trunc);
  fileStream << "V1\n";
  fileStream << "# 3D Points\n";
  fileStream << "8                  # Number of points\n";
  fileStream <<  cubeEdgeSize / 2 << " " <<  cubeEdgeSize / 2 << " " << 0 << "    # Point 0: (X, Y, Z)\n";
  fileStream <<  cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << 0 << "    # Point 1\n";
  fileStream << -cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << 0 << "    # Point 2\n";
  fileStream << -cubeEdgeSize / 2 << " " <<  cubeEdgeSize / 2 << " " << 0 << "    # Point 3\n";
  fileStream << -cubeEdgeSize / 2 << " " <<  cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 4\n";
  fileStream << -cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 5\n";
  fileStream <<  cubeEdgeSize / 2 << " " << -cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 6\n";
  fileStream <<  cubeEdgeSize / 2 << " " <<  cubeEdgeSize / 2 << " " << -cubeEdgeSize << "    # Point 7\n";
  fileStream << "# 3D Lines\n";
  fileStream << "0                  # Number of lines\n";
  fileStream << "# Faces from 3D lines\n";
  fileStream << "0                  # Number of faces\n";
  fileStream << "# Faces from 3D points\n";
  fileStream << "6                  # Number of faces\n";
  fileStream << "4 0 3 2 1          # Face 0: [number of points] [index of the 3D points]...\n";
  fileStream << "4 1 2 5 6\n";
  fileStream << "4 4 7 6 5\n";
  fileStream << "4 0 7 4 3\n";
  fileStream << "4 5 2 3 4\n";
  fileStream << "4 0 1 6 7          # Face 5\n";
  fileStream << "# 3D cylinders\n";
  fileStream << "0                  # Number of cylinders\n";
  fileStream << "# 3D circles\n";
  fileStream << "0                  # Number of circles\n";
  fileStream.close();
}

vpHomogeneousMatrix detectAprilTag(vpDetectorAprilTag::vpAprilTagFamily tagFamily, float quad_decimate,
                                   vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod, int nThreads,
#if defined(VISP_HAVE_V4L2)
                                   vpV4l2Grabber &g,
#elif defined(VISP_HAVE_OPENCV)
                                   cv::VideoCapture &cap,
#endif
                                   vpImage<unsigned char> &I, double tagSize, vpCameraParameters cam)
{

  // Define aprilTag options
  vpDetectorAprilTag detector(tagFamily);
  detector.setAprilTagQuadDecimate(quad_decimate);
  detector.setAprilTagPoseEstimationMethod(poseEstimationMethod);
  detector.setAprilTagNbThreads(nThreads);

  std::vector<vpHomogeneousMatrix> cMo_vec;
  cv::Mat frame;

  bool runTagDetection = true;
  while (runTagDetection) {
#if defined(VISP_HAVE_V4L2)
    g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
    cap >> frame;
    vpImageConvert::convert(frame, I);
#endif

    vpDisplay::display(I);

    // detection
    detector.detect(I, tagSize, cam, cMo_vec);

    // Display camera pose
    for (size_t i = 0; i < cMo_vec.size(); i++) {
      vpDisplay::displayFrame(I, cMo_vec[i], cam, tagSize / 2, vpColor::none, 3);
    }

    vpDisplay::displayText(I, 20, 20, "Waiting tag detection.", vpColor::red);
    vpDisplay::flush(I);

    if (detector.getNbObjects() > 0) { // if tag detected, we pick the first one
      runTagDetection = false;
    }
  }
  std::cout << "AprilTag detected, camera to aprilTag transform :\n" << cMo_vec.at(0) << std::endl;
  return cMo_vec.at(0);
}

int main(int argc, const char **argv)
{
//! [Macro defined]
#if defined(VISP_HAVE_APRILTAG) && (defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV)) &&                           \
    defined(VISP_HAVE_MODULE_MBT)
  //! [Macro defined]

  int opt_device = 0;
  vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
  vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
  double tagSize = 0.08;
  float quad_decimate = 1.0;
  int nThreads = 1;
  std::string intrinsic_file = "";
  std::string camera_name = "";
  double cubeEdgeSize = 0.125; // 12.5cm by default
  bool useTexture = false;

#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
  bool display_off = true;
#else
  bool display_off = false;
#endif

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--pose_method" && i + 1 < argc) {
      poseEstimationMethod = (vpDetectorAprilTag::vpPoseEstimationMethod)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--tag_size" && i + 1 < argc) {
      tagSize = atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--input" && i + 1 < argc) {
      opt_device = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--quad_decimate" && i + 1 < argc) {
      quad_decimate = (float)atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--nthreads" && i + 1 < argc) {
      nThreads = atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--intrinsic" && i + 1 < argc) {
      intrinsic_file = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--camera_name" && i + 1 < argc) {
      camera_name = std::string(argv[i + 1]);
    } else if (std::string(argv[i]) == "--display_off") {
      display_off = true;
    } else if (std::string(argv[i]) == "--tag_family" && i + 1 < argc) {
      tagFamily = (vpDetectorAprilTag::vpAprilTagFamily)atoi(argv[i + 1]);
    } else if (std::string(argv[i]) == "--cube_edge_size" && i + 1 < argc) {
      cubeEdgeSize = atof(argv[i + 1]);
    } else if (std::string(argv[i]) == "--texture" && i + 1 < argc) {
      useTexture = true;
    } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Usage: " << argv[0] << " [--input <camera input>] [--tag_size <tag_size in m>]"
                                           " [--quad_decimate <quad_decimate>] [--nthreads <nb>]"
                                           " [--intrinsic <intrinsic file>] [--camera_name <camera name>]"
                                           " [--pose_method <method> (0: HOMOGRAPHY, 1: "
                                           "HOMOGRAPHY_VIRTUAL_VS,"
                                           " 2: DEMENTHON_VIRTUAL_VS, 3: LAGRANGE_VIRTUAL_VS,"
                                           " 4: BEST_RESIDUAL_VIRTUAL_VS)]"
                                           " [--tag_family <family> (0: TAG_36h11, 1: TAG_36h10, 2: "
                                           "TAG_36ARTOOLKIT,"
                                           " 3: TAG_25h9, 4: TAG_25h7, 5: TAG_16h5)]";
#if (defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << " [--display_off]";
#endif
      std::cout << " [--cube_edge_size <cube_edge_size in m>]" << std::endl;
      std::cout << " [--texture]" << std::endl;
      std::cout << " [--help]" << std::endl;
      return EXIT_SUCCESS;
    }
  }

  createCaoFile(cubeEdgeSize);

  vpCameraParameters cam;
  bool camIsInit = false;
#ifdef VISP_HAVE_XML2
  vpXmlParserCamera parser;
  if (!intrinsic_file.empty() && !camera_name.empty()) {
    parser.parse(cam, intrinsic_file, camera_name, vpCameraParameters::perspectiveProjWithoutDistortion);
    camIsInit = true;
  }
#endif

  try {
    vpImage<unsigned char> I;

//! [Construct grabber]
#if defined(VISP_HAVE_V4L2)
    vpV4l2Grabber g;
    std::ostringstream device;
    device << "/dev/video" << opt_device;
    g.setDevice(device.str());
    g.setScale(1);
    g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
    cv::VideoCapture cap(opt_device); // open the default camera
    if (!cap.isOpened()) {            // check if we succeeded
      std::cout << "Failed to open the camera" << std::endl;
      return EXIT_FAILURE;
    }
    cv::Mat frame;
    cap >> frame; // get a new frame from camera
    vpImageConvert::convert(frame, I);
#endif
    if (!camIsInit) {
      cam.initPersProjWithoutDistortion(600, 600, I.getWidth() / 2., I.getHeight() / 2.);
    }

    std::cout << "cam:\n" << cam << std::endl;
    std::cout << "poseEstimationMethod: " << poseEstimationMethod << std::endl;
    std::cout << "tagFamily: " << tagFamily << std::endl;

    // Construct display
    vpDisplay *d = NULL;
    if (!display_off) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI(I);
#elif defined(VISP_HAVE_OPENCV)
      d = new vpDisplayOpenCV(I);
#endif
    }

    // Prepare MBT
    vpMbGenericTracker tracker;
    if (useTexture)
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
    else
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
    // edges
    vpMe me;
    me.setMaskSize(5);
    me.setMaskNumber(180);
    me.setRange(8);
    me.setThreshold(10000);
    me.setMu1(0.5);
    me.setMu2(0.5);
    me.setSampleStep(4);
    tracker.setMovingEdge(me);
    // camera calibration params
    tracker.setCameraParameters(cam);
    // model definition
    tracker.loadModel("cube.cao");
    tracker.setDisplayFeatures(true);

    bool run = true;
    vpHomogeneousMatrix cMapril;
    // wait for a tag detection
    while (run) {

      // first we wait for an aprilTag detection
      cMapril = detectAprilTag(tagFamily, quad_decimate, poseEstimationMethod, nThreads,
#if defined(VISP_HAVE_V4L2)
                               g,
#elif defined(VISP_HAVE_OPENCV)
                               cap,
#endif
                               I, tagSize, cam);

      // then we init & run the mbt
      // build cube to camera transformation
      tracker.initFromPose(I, cMapril);

      // Track model
      vpHomogeneousMatrix cMo;
      bool trackingActive = true;
      while (trackingActive) {
#if defined(VISP_HAVE_V4L2)
        g.acquire(I);
#elif defined(VISP_HAVE_OPENCV)
        cap >> frame;
        vpImageConvert::convert(frame, I);
#endif

        vpDisplay::display(I);

        // Track
        tracker.track(I);
        tracker.getPose(cMo);

        // Compute error
        double projectionError = tracker.computeCurrentProjectionError(I, cMo, cam);
        if (projectionError > 30.0) {
          trackingActive = false;
          std::cout << "Tracking error too high (" << projectionError << "), exiting MBT.\n";
        }

        // Display
        tracker.getCameraParameters(cam);
        tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);

        vpDisplay::displayText(I, 20, 20, "Click to quit.", vpColor::red);
        vpDisplay::flush(I);
        if (vpDisplay::getClick(I, false)) { // exit
          run = false;
          trackingActive = false;
        }
      }
    }

    if (!display_off)
      delete d;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.getMessage() << std::endl;
  }

  return EXIT_SUCCESS;
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_APRILTAG
  std::cout << "ViSP is not build with Apriltag support" << std::endl;
#endif
#if !(defined(VISP_HAVE_V4L2) || defined(VISP_HAVE_OPENCV))
  std::cout << "ViSP is not build with v4l2 or OpenCV support" << std::endl;
#endif
  std::cout << "Install missing 3rd parties, configure and build ViSP to run this tutorial" << std::endl;
#endif
  return EXIT_SUCCESS;
}
