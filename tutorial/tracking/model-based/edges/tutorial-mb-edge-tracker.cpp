/*! \example tutorial-mb-edge-tracker.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeTracker.h>

int main()
{
  vpImage<unsigned char> I;
  vpCameraParameters cam;
  vpHomogeneousMatrix cMo;

  vpImageIo::read(I, "teabox.pgm");

#if defined(VISP_HAVE_X11)
  vpDisplayX display(I,100,100,"Model-based edge tracker");;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI display(I,100,100,"Model-based edge tracker");;
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif

  vpMbEdgeTracker tracker;
#ifdef VISP_HAVE_XML2
  tracker.loadConfigFile("teabox.xml");
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
  
  tracker.setClipping(tracker.getClipping() | vpMbtPolygon::FOV_CLIPPING);
//   tracker.setClipping(tracker.getClipping() | vpMbtPolygon::LEFT_CLIPPING | vpMbtPolygon::RIGHT_CLIPPING | vpMbtPolygon::UP_CLIPPING | vpMbtPolygon::DOWN_CLIPPING); // Equivalent to FOV_CLIPPING
#endif
  tracker.setDisplayFeatures(true);
  tracker.setOgreVisibilityTest(false);
  tracker.loadModel("teabox.cao");
  tracker.initClick(I, "teabox.init");

  while(1){
    vpDisplay::display(I);
    tracker.track(I);
    tracker.getPose(cMo);
    tracker.getCameraParameters(cam);
    tracker.display(I, cMo, cam, vpColor::red, 2);
    vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
    vpDisplay::flush(I);

    if (vpDisplay::getClick(I, false))
      break;
    vpTime::wait(40);
  }
#ifdef VISP_HAVE_XML2
  vpXmlParser::cleanup();
#endif
#ifdef VISP_HAVE_COIN
  SoDB::finish();
#endif
}
