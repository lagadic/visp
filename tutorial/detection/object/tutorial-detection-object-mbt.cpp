//! \example tutorial-detection-object-mbt.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoReader.h>
#include <visp3/mbt/vpMbGenericTracker.h>
#include <visp3/vision/vpKeyPoint.h>

int main(int argc, char **argv)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020400)
  //! [MBT code]
  try {
    std::string videoname = "teabox.mpg";

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
    vpCameraParameters cam;
    vpHomogeneousMatrix cMo;

    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);

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

    display.init(I, 100, 100, "Model-based edge tracker");

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
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      me.setNbTotalSample(250);
      tracker.setMovingEdge(me);
      cam.initPersProjWithoutDistortion(839, 839, 325, 243);
      tracker.setCameraParameters(cam);
      tracker.setAngleAppear(vpMath::rad(70));
      tracker.setAngleDisappear(vpMath::rad(80));
      tracker.setNearClippingDistance(0.1);
      tracker.setFarClippingDistance(100.0);
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
    }

    tracker.setOgreVisibilityTest(false);
    if (vpIoTools::checkFilename(objectname + ".cao"))
      tracker.loadModel(objectname + ".cao");
    else if (vpIoTools::checkFilename(objectname + ".wrl"))
      tracker.loadModel(objectname + ".wrl");
    tracker.setDisplayFeatures(true);
    tracker.initClick(I, objectname + ".init", true);
    tracker.track(I);
//! [MBT code]

//! [Keypoint selection]
#if (defined(VISP_HAVE_OPENCV_NONFREE) || defined(VISP_HAVE_OPENCV_XFEATURES2D))
    std::string detectorName = "SIFT";
    std::string extractorName = "SIFT";
    std::string matcherName = "BruteForce";
    std::string configurationFile = "detection-config-SIFT.xml";
#else
    std::string detectorName = "FAST";
    std::string extractorName = "ORB";
    std::string matcherName = "BruteForce-Hamming";
    std::string configurationFile = "detection-config.xml";
#endif
    //! [Keypoint selection]

    //! [Keypoint declaration]
    vpKeyPoint keypoint_learning;
    //! [Keypoint declaration]
    if (usexml) {
//! [Keypoint xml config]
#ifdef VISP_HAVE_XML2
      keypoint_learning.loadConfigFile(configurationFile);
#endif
      //! [Keypoint xml config]
    } else {
      //! [Keypoint code config]
      keypoint_learning.setDetector(detectorName);
      keypoint_learning.setExtractor(extractorName);
      keypoint_learning.setMatcher(matcherName);
      //! [Keypoint code config]
    }

    //! [Keypoints reference detection]
    std::vector<cv::KeyPoint> trainKeyPoints;
    double elapsedTime;
    keypoint_learning.detect(I, trainKeyPoints, elapsedTime);
    //! [Keypoints reference detection]

    //! [Keypoints selection on faces]
    std::vector<vpPolygon> polygons;
    std::vector<std::vector<vpPoint> > roisPt;
    std::pair<std::vector<vpPolygon>, std::vector<std::vector<vpPoint> > > pair = tracker.getPolygonFaces(false);
    polygons = pair.first;
    roisPt = pair.second;

    std::vector<cv::Point3f> points3f;
    tracker.getPose(cMo);
    vpKeyPoint::compute3DForPointsInPolygons(cMo, cam, trainKeyPoints, polygons, roisPt, points3f);
    //! [Keypoints selection on faces]

    //! [Keypoints build reference]
    keypoint_learning.buildReference(I, trainKeyPoints, points3f);
    //! [Keypoints build reference]

    //! [Save learning data]
    keypoint_learning.saveLearningData("teabox_learning_data.bin", true);
    //! [Save learning data]

    //! [Display reference keypoints]
    vpDisplay::display(I);
    for (std::vector<cv::KeyPoint>::const_iterator it = trainKeyPoints.begin(); it != trainKeyPoints.end(); ++it) {
      vpDisplay::displayCross(I, (int)it->pt.y, (int)it->pt.x, 4, vpColor::red);
    }
    vpDisplay::displayText(I, 10, 10, "Learning step: keypoints are detected on visible teabox faces", vpColor::red);
    vpDisplay::displayText(I, 30, 10, "Click to continue with detection...", vpColor::red);
    vpDisplay::flush(I);
    vpDisplay::getClick(I, true);
    //! [Display reference keypoints]

    //! [Init keypoint detection]
    vpKeyPoint keypoint_detection;
    if (usexml) {
#ifdef VISP_HAVE_XML2
      keypoint_detection.loadConfigFile(configurationFile);
#endif
    } else {
      keypoint_detection.setDetector(detectorName);
      keypoint_detection.setExtractor(extractorName);
      keypoint_detection.setMatcher(matcherName);
      keypoint_detection.setFilterMatchingType(vpKeyPoint::ratioDistanceThreshold);
      keypoint_detection.setMatchingRatioThreshold(0.8);
      keypoint_detection.setUseRansacVVS(true);
      keypoint_detection.setUseRansacConsensusPercentage(true);
      keypoint_detection.setRansacConsensusPercentage(20.0);
      keypoint_detection.setRansacIteration(200);
      keypoint_detection.setRansacThreshold(0.005);
    }
    //! [Init keypoint detection]

    //! [Load teabox learning data]
    keypoint_detection.loadLearningData("teabox_learning_data.bin", true);
    //! [Load teabox learning data]

    double error;
    bool click_done = false;

    while (!g.end()) {
      g.acquire(I);
      vpDisplay::display(I);

      vpDisplay::displayText(I, 10, 10, "Detection and localization in process...", vpColor::red);

      //! [Matching and pose estimation]
      if (keypoint_detection.matchPoint(I, cam, cMo, error, elapsedTime)) {
        //! [Matching and pose estimation]

        //! [Tracker set pose]
        tracker.setPose(I, cMo);
        //! [Tracker set pose]
        //! [Display]
        tracker.display(I, cMo, cam, vpColor::red, 2);
        vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
        //! [Display]
      }

      vpDisplay::displayText(I, 30, 10, "A click to exit.", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false)) {
        click_done = true;
        break;
      }
    }
    if (!click_done)
      vpDisplay::getClick(I);
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
