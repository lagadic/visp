/*! \example tutorial-mb-edge-tracker.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpIoTools.h>
#include <visp/vpMbEdgeTracker.h>
#include <visp/vpVideoReader.h>

int main(int argc, char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100) || defined(VISP_HAVE_FFMPEG)
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
    vpDisplayX display(I,100,100,"Model-based edge tracker");;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display(I,100,100,"Model-based edge tracker");;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV display(I,100,100,"Model-based edge tracker");;
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpMbEdgeTracker tracker;
#ifdef VISP_HAVE_XML2
    tracker.loadConfigFile(objectname + ".xml");
#else
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
    tracker.setAngleAppear( vpMath::rad(70) );
    tracker.setAngleDisappear( vpMath::rad(80) );
    tracker.setNearClippingDistance(0.1);
    tracker.setFarClippingDistance(100.0);
    tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
#endif
    tracker.setOgreVisibilityTest(false);
    tracker.loadModel(objectname + ".cao");
    //tracker.loadModel(objectname + ".wrl");
    tracker.setDisplayFeatures(true);
    tracker.initClick(I, objectname + ".init", true);

    while(! g.end()){
      g.acquire(I);
      vpDisplay::display(I);
      tracker.track(I);
      tracker.getPose(cMo);
      tracker.getCameraParameters(cam);
      tracker.display(I, cMo, cam, vpColor::red, 2);
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::flush(I);

      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
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
#else
  std::cout << "Install OpenCV or ffmpeg and rebuild ViSP to use this example." << std::endl;
#endif
}
