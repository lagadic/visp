/*! \example tutorial-mb-klt-tracker.cpp */
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImageIo.h>
#include <visp/vpMbKltTracker.h>

int main()
{
#ifdef VISP_HAVE_OPENCV
  try {
    vpImage<unsigned char> I;
    vpCameraParameters cam;
    vpHomogeneousMatrix cMo;

    vpImageIo::read(I, "teabox.pgm");

#if defined(VISP_HAVE_X11)
    vpDisplayX display(I,100,100,"Model-based keypoints tracker");;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display(I,100,100,"Model-based keypoints tracker");;
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

    vpMbKltTracker tracker;
#ifdef VISP_HAVE_XML2
    tracker.loadConfigFile("teabox.xml");
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
    tracker.loadModel("teabox-triangle.cao");
    tracker.setDisplayFeatures(true);
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
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }

#endif
}
