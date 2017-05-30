/*! \example tutorial-mb-klt-tracker.cpp */
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/mbt/vpMbKltTracker.h>
#include <visp3/io/vpVideoReader.h>

int main(int argc, char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)

  try {
    std::string videoname = "teabox.mpg";

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--name")
        videoname = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--name <video name>] [--help]\n" << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(videoname);
    std::string objectname = vpIoTools::getNameWE(videoname);

    if(! parentname.empty())
       objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname
              << ".[init,"
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

    display.init(I, 100, 100,"Model-based keypoint tracker");

    vpMbKltTracker tracker;
    bool usexml = false;
    //! [Load xml]
#ifdef VISP_HAVE_XML2
    if(vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      usexml = true;
    }
#endif
    //! [Load xml]
    if (! usexml) {
      //! [Set parameters]
      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(5);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      tracker.setKltOpencv(klt_settings);
      tracker.setKltMaskBorder(5);
      cam.initPersProjWithoutDistortion(839, 839, 325, 243);
      tracker.setCameraParameters(cam);
      tracker.setAngleAppear( vpMath::rad(70) );
      tracker.setAngleDisappear( vpMath::rad(80) );
      tracker.setNearClippingDistance(0.1);
      tracker.setFarClippingDistance(100.0);
      tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
      //! [Set parameters]
    }
    tracker.setOgreVisibilityTest(true);
    tracker.setOgreShowConfigDialog(false);
    tracker.loadModel(objectname + "-triangle.cao");
    tracker.setDisplayFeatures(true);
    tracker.initClick(I, objectname + ".init", true);

    while(! g.end()){
      g.acquire(I);
      vpDisplay::display(I);
      tracker.track(I);
      tracker.getPose(cMo);
      tracker.getCameraParameters(cam);
      tracker.display(I, cMo, cam, vpColor::red, 2, true);
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);

#ifdef VISP_HAVE_XML2
    vpXmlParser::cleanup();
#endif
#if defined(VISP_HAVE_COIN3D) && (COIN_MAJOR_VERSION == 3)
    SoDB::finish();
#endif
  }
  catch(vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
#ifdef VISP_HAVE_OGRE
  catch(Ogre::Exception &e) {
    std::cout << "Catch an Ogre exception: " << e.getDescription() << std::endl;
  }
#endif
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
