/*! \example tutorial-mb-klt-tracker.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpMbKltTracker.h>
#include <visp/vpVideoReader.h>

int main(int argc, char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION < 0x030000)

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
              << "cao]" << std::endl;
    std::cout << "Tracker optional config files: " << objectname << ".[ppm]" << std::endl;
    vpImage<unsigned char> I;
    vpCameraParameters cam;
    vpHomogeneousMatrix cMo;

    vpVideoReader g;
    g.setFileName(videoname);
    g.open(I);

#if defined(VISP_HAVE_X11)
    vpDisplayX display(I,100,100,"Model-based keypoints tracker");;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display(I,100,100,"Model-based keypoints tracker");;
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpMbKltTracker tracker;
#ifdef VISP_HAVE_XML2
    tracker.loadConfigFile(objectname + ".xml");
#else
    tracker.setMaskBorder(5);
    vpKltOpencv klt_settings;
    klt_settings.setMaxFeatures(300);
    klt_settings.setWindowSize(5);
    klt_settings.setQuality(0.015);
    klt_settings.setMinDistance(8);
    klt_settings.setHarrisFreeParameter(0.01);
    klt_settings.setBlockSize(3);
    klt_settings.setPyramidLevels(3);
    tracker.setKltOpencv(klt_settings);
    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    tracker.setCameraParameters(cam);
    tracker.setAngleAppear( vpMath::rad(70) );
    tracker.setAngleDisappear( vpMath::rad(80) );
    tracker.setNearClippingDistance(0.1);
    tracker.setFarClippingDistance(100.0);
    tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
#endif
    tracker.setOgreVisibilityTest(true);
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
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false))
        break;
    }

#ifdef VISP_HAVE_XML2
    vpXmlParser::cleanup();
#endif
#if defined(VISP_HAVE_COIN) && (COIN_MAJOR_VERSION == 3)
    SoDB::finish();
#endif
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

#endif
}
