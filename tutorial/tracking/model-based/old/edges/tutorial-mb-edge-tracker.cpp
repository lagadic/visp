//! \example tutorial-mb-edge-tracker.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
//! [Include]
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
    std::string videoname = "teabox.mp4";

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help] [-h]\n" << std::endl;
        return EXIT_SUCCESS;
      }
    }
    std::string parentname = vpIoTools::getParent(videoname);
    std::string objectname = vpIoTools::getNameWE(videoname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << videoname << std::endl;
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
    g.setFileName(videoname);
    g.open(I);

#if defined(VISP_HAVE_X11)
    vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display;
#elif defined(HAVE_OPENCV_HIGHGUI)
    vpDisplayOpenCV display;
#else
    std::cout << "No image viewer is available..." << std::endl;
    return EXIT_FAILURE;
#endif

    display.init(I, 100, 100, "Model-based edge tracker");

    //! [Constructor]
    vpMbEdgeTracker tracker;
    //! [Constructor]
    bool usexml = false;
    //! [Load xml]
#if defined(VISP_HAVE_PUGIXML)
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      usexml = true;
    }
#endif
//! [Load xml]
    if (!usexml) {
      //! [Set parameters]
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
      me.setThreshold(20);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      tracker.setMovingEdge(me);
      cam.initPersProjWithoutDistortion(839, 839, 325, 243);
      tracker.setCameraParameters(cam);
      //! [Set angles]
      tracker.setAngleAppear(vpMath::rad(70));
      tracker.setAngleDisappear(vpMath::rad(80));
      //! [Set angles]
      //! [Set clipping distance]
      tracker.setNearClippingDistance(0.1);
      tracker.setFarClippingDistance(100.0);
      //! [Set clipping distance]
      //! [Set clipping fov]
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
      //! [Set clipping fov]
      //! [Set parameters]
    }
    //! [Set ogre]
    tracker.setOgreVisibilityTest(false);
    tracker.setOgreShowConfigDialog(false);
    //! [Set ogre]
    //! [Load cao]
    if (vpIoTools::checkFilename(objectname + ".cao"))
      tracker.loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Load wrl]
    else if (vpIoTools::checkFilename(objectname + ".wrl"))
      tracker.loadModel(objectname + ".wrl");
    //! [Load wrl]
    //! [Set display]
    tracker.setDisplayFeatures(true);
    //! [Set display]
    //! [Init]
    tracker.initClick(I, objectname + ".init", true);
    //! [Init]

    while (!g.end()) {
      g.acquire(I);
      vpDisplay::display(I);
      //! [Track]
      tracker.track(I);
      //! [Track]
      //! [Get pose]
      tracker.getPose(cMo);
      //! [Get pose]
      //! [Display]
      tracker.getCameraParameters(cam);
      tracker.display(I, cMo, cam, vpColor::red, 2);
      //! [Display]
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
      vpDisplay::flush(I);
      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);
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
