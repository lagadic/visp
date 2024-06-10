//! \example tutorial-mb-tracker-full.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
//! [Include]
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  try {
    std::string opt_videoname = "teabox.mp4";
    std::string opt_modelname = "teabox.cao";
    int opt_tracker = 0;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--video")
        opt_videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--model")
        opt_modelname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--tracker")
        opt_tracker = atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
          << " [--video <video name>] [--model <model name>] "
          "[--tracker <0=egde|1=keypoint|2=hybrid>] [--help] [-h]\n"
          << std::endl;
        return EXIT_SUCCESS;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_modelname);
    std::string objectname = vpIoTools::getNameWE(opt_modelname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << opt_videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname << ".[init,"
      << "xml,"
      << "cao or wrl]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;

    //! [Image]
    vpImage<unsigned char> I;
    vpCameraParameters cam;
    //! [Image]
    //! [cMo]
    vpHomogeneousMatrix cMo;
    //! [cMo]

    vpVideoReader g;
    g.setFileName(opt_videoname);
    g.open(I);

    vpDisplay *display = nullptr;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display = new vpDisplayGDI;
#elif defined(HAVE_OPENCV_HIGHGUI)
    display = new vpDisplayOpenCV;
#endif
    display->init(I, 100, 100, "Model-based tracker");

    //! [Constructor]
    vpMbTracker *tracker;
    if (opt_tracker == 0)
      tracker = new vpMbEdgeTracker;
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    else if (opt_tracker == 1)
      tracker = new vpMbKltTracker;
    else
      tracker = new vpMbEdgeKltTracker;
#else
    else {
      std::cout << "klt and hybrid model-based tracker are not available "
        "since visp_klt module is missing"
        << std::endl;
      return EXIT_FAILURE;
    }
#endif
    //! [Constructor]

    bool usexml = false;
    //! [Load xml]
#if defined(VISP_HAVE_PUGIXML)
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      tracker->loadConfigFile(objectname + ".xml");
      usexml = true;
    }
#endif
    //! [Load xml]

    if (!usexml) {
      //! [Set parameters]
      if (opt_tracker == 0 || opt_tracker == 2) {
        //! [Set moving-edges parameters]
        vpMe me;
        me.setMaskSize(5);
        me.setMaskNumber(180);
        me.setRange(8);
        me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
        me.setThreshold(20);
        me.setMu1(0.5);
        me.setMu2(0.5);
        me.setSampleStep(4);
        dynamic_cast<vpMbEdgeTracker *>(tracker)->setMovingEdge(me);
        //! [Set moving-edges parameters]
      }

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
      if (opt_tracker == 1 || opt_tracker == 2) {
        //! [Set klt parameters]
        vpKltOpencv klt_settings;
        klt_settings.setMaxFeatures(300);
        klt_settings.setWindowSize(5);
        klt_settings.setQuality(0.015);
        klt_settings.setMinDistance(8);
        klt_settings.setHarrisFreeParameter(0.01);
        klt_settings.setBlockSize(3);
        klt_settings.setPyramidLevels(3);
        dynamic_cast<vpMbKltTracker *>(tracker)->setKltOpencv(klt_settings);
        dynamic_cast<vpMbKltTracker *>(tracker)->setKltMaskBorder(5);
        //! [Set klt parameters]
      }
#endif

      //! [Set camera parameters]
      cam.initPersProjWithoutDistortion(839.21470, 839.44555, 325.66776, 243.69727);
      tracker->setCameraParameters(cam);
      //! [Set camera parameters]

      //! [Set angles]
      tracker->setAngleAppear(vpMath::rad(70));
      tracker->setAngleDisappear(vpMath::rad(80));
      //! [Set angles]
      //! [Set clipping distance]
      tracker->setNearClippingDistance(0.1);
      tracker->setFarClippingDistance(100.0);
      //! [Set clipping distance]
      //! [Set clipping fov]
      tracker->setClipping(tracker->getClipping() | vpMbtPolygon::FOV_CLIPPING);
      //! [Set clipping fov]
      //! [Set parameters]
    }
    //! [Set visibility parameters]
    //! [Set ogre visibility]
    tracker->setOgreVisibilityTest(false);
    tracker->setOgreShowConfigDialog(false);
    //! [Set ogre visibility]
    //! [Set scanline visibility]
    tracker->setScanLineVisibilityTest(true);
    //! [Set scanline visibility]
    //! [Set visibility parameters]

    //! [Load cao]
    if (vpIoTools::checkFilename(objectname + ".cao"))
      tracker->loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Load wrl]
    else if (vpIoTools::checkFilename(objectname + ".wrl"))
      tracker->loadModel(objectname + ".wrl");
    //! [Load wrl]
    //! [Set display]
    tracker->setDisplayFeatures(true);
    //! [Set display]
    //! [Init]
    tracker->initClick(I, objectname + ".init", true);
    //! [Init]

    while (!g.end()) {
      g.acquire(I);
      vpDisplay::display(I);
      //! [Track]
      tracker->track(I);
      //! [Track]
      //! [Get pose]
      tracker->getPose(cMo);
      //! [Get pose]
      //! [Display]
      tracker->getCameraParameters(cam);
      tracker->display(I, cMo, cam, vpColor::red, 2);
      //! [Display]
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);
    //! [Cleanup]
    delete display;
    delete tracker;
    //! [Cleanup]
  }
  catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
#ifdef VISP_HAVE_OGRE
  catch (Ogre::Exception &e) {
    std::cout << "Catch an Ogre exception: " << e.getDescription() << std::endl;
  }
#endif
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
