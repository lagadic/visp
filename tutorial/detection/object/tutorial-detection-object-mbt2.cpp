//! \example tutorial-detection-object-mbt2.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpKeyPoint.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
void learnCube(const vpImage<unsigned char> &I, vpMbGenericTracker &tracker, vpKeyPoint &keypoint_learning, int id)
{
  //! [Keypoints reference detection]
  std::vector<cv::KeyPoint> trainKeyPoints;
  double elapsedTime;
  keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
  //! [Keypoints reference detection]

  //! [Keypoints selection on faces]
  std::vector<vpPolygon> polygons;
  std::vector<std::vector<vpPoint> > roisPt;
  std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces();
  polygons = pair.first;
  roisPt = pair.second;

  std::vector<cv::Point3f> points3f;
  vpHomogeneousMatrix cMo;
  tracker.getPose(cMo);
  vpCameraParameters cam;
  tracker.getCameraParameters(cam);
  vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);
  //! [Keypoints selection on faces]

  //! [Keypoints build reference]
  keypoint_learning.buildReference(I, trainKeyPoints, points3f, true, id);
  //! [Keypoints build reference]

  //! [Display reference keypoints]
  for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
    vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
  }
  //! [Display reference keypoints]
}
#endif

int main(int argc, char **argv)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
  //! [MBT code]
  try {
    std::string videoname = "cube.mpeg";

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(videoname);
    std::string objectname = vpIoTools::getNameWE(videoname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname << ".[init,"
#ifdef VISP_HAVE_XML2
              << "xml,"
#endif
              << "cao or wrl]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;

    vpImage<unsigned char> I;
    vpHomogeneousMatrix cMo;
    vpCameraParameters cam;

    vpMbGenericTracker tracker(vpMbGenericTracker::EDGE_TRACKER);
    bool usexml = false;
#ifdef VISP_HAVE_XML2
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      tracker.getCameraParameters(cam);
      usexml = true;
    }
#endif
    if (!usexml) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(7);
      me.setThreshold(5000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      me.setNbTotalSample(250);
      tracker.setMovingEdge(me);
      cam.initPersProjWithoutDistortion(547, 542, 339, 235);
      tracker.setCameraParameters(cam);
      tracker.setAngleAppear(vpMath::rad(89));
      tracker.setAngleDisappear(vpMath::rad(89));
      tracker.setNearClippingDistance(0.01);
      tracker.setFarClippingDistance(10.0);
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
    }

    tracker.setOgreVisibilityTest(false);
    if (vpIoTools::checkFilename(objectname + ".cao"))
      tracker.loadModel(objectname + ".cao");
    else if (vpIoTools::checkFilename(objectname + ".wrl"))
      tracker.loadModel(objectname + ".wrl");
    tracker.setDisplayFeatures(true);
    //! [MBT code]

    //! [Keypoint declaration]
    vpKeyPoint keypoint_learning("ORB", "ORB", "BruteForce-Hamming");
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
    keypoint_learning.setDetectorParameter("ORB", "nLevels", 1);
#else
    cv::Ptr<cv::ORB> orb_learning = keypoint_learning.getDetector("ORB").dynamicCast<cv::ORB>();
    if (orb_learning) {
      orb_learning->setNLevels(1);
    }
#endif
//! [Keypoint declaration]

#if defined(VISP_HAVE_X11)
    vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display;
#else
    std::cout << "No image viewer is available..." << std::endl;
    return 0;
#endif

    /*
     * Start the part of the code dedicated to object learning from 3 images
     */
    std::string imageName[] = {"cube0001.png", "cube0150.png", "cube0200.png"};
    vpHomogeneousMatrix initPoseTab[] = {
        vpHomogeneousMatrix(0.02143385294, 0.1098083886, 0.5127439561, 2.087159614, 1.141775176, -0.4701291124),
        vpHomogeneousMatrix(0.02651282185, -0.03713587374, 0.6873765919, 2.314744454, 0.3492296488, -0.1226054828),
        vpHomogeneousMatrix(0.02965448956, -0.07283091786, 0.7253526051, 2.300529617, -0.4286674806, 0.1788761025)};
    for (int i = 0; i < 3; i++) {
      vpImageIo::read(I, imageName[i]);
      if (i == 0) {
        display.init(I, 10, 10);
      }
      std::stringstream title;
      title << "Learning cube on image: " << imageName[i];
      vpDisplay::setTitle(I, title.str().c_str());

      vpDisplay::display(I);

      //! [Set tracker pose]
      tracker.setPose(I, initPoseTab[i]);
      //! [Set tracker pose]

      //! [Refine pose]
      tracker.track(I);
      //! [Refine pose]

      //! [Display tracker pose]
      tracker.getPose(cMo);
      tracker.display(I, cMo, cam, vpColor::red);
      //! [Display tracker pose]

      //! [Learn cube call]
      learnCube(I, tracker, keypoint_learning, i);
      //! [Learn cube call]

      vpDisplay::displayText(I, 10, 10, "Learning step: keypoints are detected on visible cube faces", vpColor::red);
      if (i < 2) {
        vpDisplay::displayText(I, 30, 10, "Click to continue the learning...", vpColor::red);
      } else {
        vpDisplay::displayText(I, 30, 10, "Click to continue with the detection...", vpColor::red);
      }

      vpDisplay::flush(I);
      vpDisplay::getClick(I, true);
    }

    //! [Save learning data]
    keypoint_learning.saveLearningData("cube_learning_data.bin", true);
    //! [Save learning data]

    /*
     * Start the part of the code dedicated to detection and localization
     */
    //! [Init keypoint detection]
    vpKeyPoint keypoint_detection("ORB", "ORB", "BruteForce-Hamming");
#if (VISP_HAVE_OPENCV_VERSION < 0x030000)
    keypoint_detection.setDetectorParameter("ORB", "nLevels", 1);
#else
    cv::Ptr<cv::ORB> orb_detector = keypoint_detection.getDetector("ORB").dynamicCast<cv::ORB>();
    orb_detector = keypoint_detection.getDetector("ORB").dynamicCast<cv::ORB>();
    if (orb_detector) {
      orb_detector->setNLevels(1);
    }
#endif
    //! [Init keypoint detection]

    //! [Load teabox learning data]
    keypoint_detection.loadLearningData("cube_learning_data.bin", true);
    //! [Load teabox learning data]

    //! [Create image matching]
    vpImage<unsigned char> IMatching;
    keypoint_detection.createImageMatching(I, IMatching);
    //! [Create image matching]

    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);

#if defined VISP_HAVE_X11
    vpDisplayX display2;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display2;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display2;
#else
    vpDisplayOpenCV display2;
#endif
    display2.init(IMatching, 50, 50, "Display matching between learned and current images");
    vpDisplay::setTitle(I, "Cube detection and localization");

    double error;
    bool click_done = false;

    while (!g.end()) {
      g.acquire(I);
      vpDisplay::display(I);

      //! [Insert image matching]
      keypoint_detection.insertImageMatching(I, IMatching);
      //! [Insert image matching]

      vpDisplay::display(IMatching);
      vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);

      double elapsedTime;
      //! [Matching and pose estimation]
      if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime)) {
        //! [Matching and pose estimation]

        //! [Tracker set pose]
        tracker.setPose(I, cMo);
        //! [Tracker set pose]

        //! [Display]
        tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.05, vpColor::none, 3);
        //! [Display]

        keypoint_detection.displayMatching(I, IMatching);

        //! [Get RANSAC inliers outliers]
        std::vector<vpImagePoint> ransacInliers = keypoint_detection.getRansacInliers();
        std::vector<vpImagePoint> ransacOutliers = keypoint_detection.getRansacOutliers();
        //! [Get RANSAC inliers outliers]

        //! [Display RANSAC inliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacInliers.begin(); it != ransacInliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::green);
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::green);
        }
        //! [Display RANSAC inliers]

        //! [Display RANSAC outliers]
        for (std::vector<vpImagePoint>::const_iterator it = ransacOutliers.begin(); it != ransacOutliers.end(); ++it) {
          vpDisplay::displayCircle(I, *it, 4, vpColor::red);
          vpImagePoint imPt(*it);
          imPt.set_u(imPt.get_u() + I.getWidth());
          imPt.set_v(imPt.get_v() + I.getHeight());
          vpDisplay::displayCircle(IMatching, imPt, 4, vpColor::red);
        }
        //! [Display RANSAC outliers]

        //! [Display image matching]
        keypoint_detection.displayMatching(I, IMatching);
        //! [Display image matching]

        //! [Display model image matching]
        vpCameraParameters cam2;
        cam2.initPersProjWithoutDistortion(cam.get_px(), cam.get_py(), cam.get_u0() + I.getWidth(),
                                           cam.get_v0() + I.getHeight());
        tracker.setCameraParameters(cam2);
        tracker.setPose(IMatching, cMo);
        tracker.display(IMatching, cMo, cam2, vpColor::red, 2);
        vpDisplay::displayFrame(IMatching, cMo, cam2, 0.05, vpColor::none, 3);
        //! [Display model image matching]
      }

      vpDisplay::flush(I);
      vpDisplay::displayText(IMatching, 30, 10, "A click to exit.", vpColor::red);
      vpDisplay::flush(IMatching);
      if (vpDisplay::getClick(I, false)) {
        click_done = true;
        break;
      }
      if (vpDisplay::getClick(IMatching, false)) {
        click_done = true;
        break;
      }
    }

    if (!click_done)
      vpDisplay::getClick(IMatching);

#ifdef VISP_HAVE_XML2
    vpXmlParser::cleanup();
#endif
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION >= 2)
    SoDB::finish();
#endif
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif

  return 0;
}
