//! \example tutorial-mb-tracker.cpp
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpIoTools.h>
//! [Include]
#include <visp3/mbt/vpMbEdgeTracker.h>
#include <visp3/mbt/vpMbEdgeKltTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, char** argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)

  try {
    std::string opt_videoname = "teabox.mpg";
    std::string opt_modelname = "teabox";
    int opt_tracker = 0;

    for (int i=0; i<argc; i++) {
      if (std::string(argv[i]) == "--video")
        opt_videoname = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--model")
        opt_modelname = std::string(argv[i+1]);
      else if (std::string(argv[i]) == "--tracker")
        opt_tracker = atoi(argv[i+1]);
      else if (std::string(argv[i]) == "--help") {
        std::cout << "\nUsage: " << argv[0] << " [--video <video name>] [--model <model name>] [--tracker <0=egde|1=keypoint|2=hybrid>] [--help]\n" << std::endl;
        return 0;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_modelname);
    std::string objectname = vpIoTools::getNameWE(opt_modelname);

    if(! parentname.empty())
       objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << opt_videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname
              << ".[init, cao]" << std::endl;
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

    vpDisplay *display = NULL;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display = new vpDisplayGDI;
#else
    display = new vpDisplayOpenCV;
#endif
    display->init(I, 100, 100, "Model-based tracker");

    //! [Constructor]
    vpMbTracker *tracker;
    if (opt_tracker == 0)
     tracker = new vpMbEdgeTracker;
#ifdef VISP_HAVE_MODULE_KLT
    else if (opt_tracker == 1)
      tracker = new vpMbKltTracker;
    else
      tracker = new vpMbEdgeKltTracker;
#else
    else {
      std::cout << "klt and hybrid model-based tracker are not available since visp_klt module is missing" << std::endl;
      return 0;
    }
#endif
    //! [Constructor]

    //! [Set parameters]
    if (opt_tracker == 0 || opt_tracker == 2) {
      vpMe me;
      me.setMaskSize(5);
      me.setMaskNumber(180);
      me.setRange(8);
      me.setThreshold(10000);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setSampleStep(4);
      dynamic_cast<vpMbEdgeTracker*>(tracker)->setMovingEdge(me);
    }

#ifdef VISP_HAVE_MODULE_KLT
    if (opt_tracker == 1 || opt_tracker == 2) {
      vpKltOpencv klt_settings;
      klt_settings.setMaxFeatures(300);
      klt_settings.setWindowSize(5);
      klt_settings.setQuality(0.015);
      klt_settings.setMinDistance(8);
      klt_settings.setHarrisFreeParameter(0.01);
      klt_settings.setBlockSize(3);
      klt_settings.setPyramidLevels(3);
      dynamic_cast<vpMbKltTracker*>(tracker)->setKltOpencv(klt_settings);
      dynamic_cast<vpMbKltTracker*>(tracker)->setKltMaskBorder(5);
    }
#endif

    //! [Set camera parameters]
    cam.initPersProjWithoutDistortion(839, 839, 325, 243);
    //! [Set camera parameters]
    tracker->setCameraParameters(cam);
    //! [Set parameters]

    //! [Load cao]
    tracker->loadModel(objectname + ".cao");
    //! [Load cao]
    //! [Set display]
    tracker->setDisplayFeatures(true);
    //! [Set display]
    //! [Init]
    tracker->initClick(I, objectname + ".init", true);
    //! [Init]

    while(! g.end()){
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
      tracker->display(I, cMo, cam, vpColor::red, 2, true);
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
  catch(vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
}
