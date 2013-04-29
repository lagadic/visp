/*! \example tutorial-mb-hybrid-tracker.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbEdgeKltTracker.h>

int main()
{
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
  vpImage<unsigned char> I;
  vpCameraParameters cam;
  vpHomogeneousMatrix cMo;

  vpImageIo::readPGM(I, "teabox.pgm");

#ifdef UNIX
  vpDisplayX display;
#else
  vpDisplayGDI display;
#endif

  display.init(I,100,100,"Model-based hybrid tracker");

  vpMbEdgeKltTracker tracker;
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
  tracker.setAngleAppear(70);
  tracker.setAngleDisappear(80);
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
#endif
  tracker.setDisplayFeatures(false);
  tracker.setOgreVisibilityTest(true);
  tracker.loadModel("teabox.cao");
  tracker.initClick(I, "teabox.init");

  while(1){
    vpDisplay::display(I);
    tracker.track(I);
    tracker.getPose(cMo);
    tracker.getCameraParameters(cam);
    tracker.display(I, cMo, cam, vpColor::red, 2, true);
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

#endif
}
