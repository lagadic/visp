//! \example tutorial-mb-generic-tracker-full.cpp
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>
//! [Include]
#include <visp3/mbt/vpMbGenericTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100)

  try {
    std::string opt_videoname = "model/teabox/teabox.mp4";
    std::string opt_modelname = "model/teabox/teabox.cao";
    int opt_tracker = 0;
    int opt_video_first_frame = -1;
    int opt_downscale_img = 1;

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--video") {
        opt_videoname = std::string(argv[i + 1]);
        i++;
      } else if (std::string(argv[i]) == "--video-first-frame") {
        opt_video_first_frame = std::atoi(argv[i + 1]);
        i++;
      } else if (std::string(argv[i]) == "--model") {
        opt_modelname = std::string(argv[i + 1]);
        i++;
      } else if (std::string(argv[i]) == "--tracker") {
        opt_tracker = atoi(argv[i + 1]);
        i++;
      } else if (std::string(argv[i]) == "--downscale-img") {
        opt_downscale_img = std::atoi(argv[i + 1]);
        i++;
      } else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0] << " [--video <video name> (default: model/teabox/teabox.mp4)]"
                  << " [--video-first-frame <image index> (default: -1)]"
                  << " [--model <model name> (default: model/teabox/teabox.cao)]"
                  << " [--tracker <0=egde|1=keypoint|2=hybrid> (default: 0)]"
                  << " [--downscale-img <scale factor> (default: 1)]"
                  << " [--help] [-h]\n"
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
    if (opt_downscale_img > 1) {
      std::cout << "Downscale image factor: " << opt_downscale_img << std::endl;
    }

    //! [Image]
    vpImage<unsigned char> Ivideo, I;
    //! [Image]
    //! [cMo]
    vpHomogeneousMatrix cMo;
    //! [cMo]

    vpVideoReader g;
    g.setFileName(opt_videoname);
    if (opt_video_first_frame > 0) {
      g.setFirstFrameIndex(static_cast<unsigned int>(opt_video_first_frame));
    }
    if (opt_downscale_img > 1) {
      g.open(Ivideo);
      Ivideo.subsample(opt_downscale_img, opt_downscale_img, I);
    } else {
      g.open(I);
    }

    std::cout << "DEBUG: frame name: " << g.getFrameName() << std::endl;
    std::cout << "DEBUG: frame first: " << g.getFirstFrameIndex() << std::endl;
    std::cout << "DEBUG: frame last: " << g.getLastFrameIndex() << std::endl;
    std::cout << "DEBUG: frame index: " << g.getFrameIndex() << std::endl;

    vpDisplay *display = NULL;
#if defined(VISP_HAVE_X11)
    display = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
    display = new vpDisplayGDI;
#else
    display = new vpDisplayOpenCV;
#endif
    display->setDownScalingFactor(vpDisplay::SCALE_AUTO);
    display->init(I, 100, 100, "Model-based tracker");

    //! [Constructor]
    vpMbGenericTracker tracker;
    if (opt_tracker == 0)
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
#ifdef VISP_HAVE_MODULE_KLT
    else if (opt_tracker == 1)
      tracker.setTrackerType(vpMbGenericTracker::KLT_TRACKER);
    else
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
#else
    else {
      std::cout << "klt and hybrid model-based tracker are not available since visp_klt module is not available. "
                   "In CMakeGUI turn visp_klt module ON, configure and build ViSP again."
                << std::endl;
      return EXIT_SUCCESS;
    }
#endif
    //! [Constructor]

    bool usexml = false;
    //! [Load xml]
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      tracker.loadConfigFile(objectname + ".xml");
      usexml = true;
    }
    //! [Load xml]

    if (!usexml) {
      //! [Set parameters]
      if (opt_tracker == 0 || opt_tracker == 2) {
        //! [Set moving-edges parameters]
        vpMe me;
        me.setMaskSize(5);
        me.setMaskNumber(180);
        me.setRange(8);
        me.setThreshold(10000);
        me.setMu1(0.5);
        me.setMu2(0.5);
        me.setSampleStep(4);
        tracker.setMovingEdge(me);
        //! [Set moving-edges parameters]
      }

#ifdef VISP_HAVE_MODULE_KLT
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
        tracker.setKltOpencv(klt_settings);
        tracker.setKltMaskBorder(5);
        //! [Set klt parameters]
      }
#endif

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

      //! [Set camera parameters]
      vpCameraParameters cam;
      cam.initPersProjWithoutDistortion(839.21470, 839.44555, 325.66776, 243.69727);
      tracker.setCameraParameters(cam);
      //! [Set camera parameters]
      //! [Set parameters]
    }

    //! [Set visibility parameters]
    //! [Set ogre visibility]
    tracker.setOgreVisibilityTest(false);
    tracker.setOgreShowConfigDialog(false);
    //! [Set ogre visibility]
    //! [Set scanline visibility]
    tracker.setScanLineVisibilityTest(true);
    //! [Set scanline visibility]
    //! [Set visibility parameters]

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

    tracker.setGoodMovingEdgesRatioThreshold(0.2);

    //! [Get camera parameters]
    vpCameraParameters cam;
    tracker.getCameraParameters(cam);
    std::cout << "Camera parameters: \n" << cam << std::endl;
    //! [Get camera parameters]

    std::cout << "Initialize tracker on image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;

    //! [Init]
    tracker.initClick(I, objectname + ".init", true);
    //! [Init]

    while (!g.end()) {
      if (opt_downscale_img > 1) {
        g.acquire(Ivideo);
        Ivideo.subsample(opt_downscale_img, opt_downscale_img, I);
      } else {
        g.acquire(I);
      }

      std::cout << "DEBUG: frame name: " << g.getFrameName() << std::endl;
      std::cout << "DEBUG: frame index: " << g.getFrameIndex() << std::endl;

      vpDisplay::display(I);
      //! [Track]
      tracker.track(I);
      //! [Track]
      //! [Get pose]
      tracker.getPose(cMo);
      //! [Get pose]
      //! [Display]
      tracker.display(I, cMo, cam, vpColor::red, 2);
      //! [Display]
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, 3);
      vpDisplay::displayText(I, 10, 10, "A click to exit...", vpColor::red);
      {
        std::stringstream ss;
        ss << "Features: edges " << tracker.getNbFeaturesEdge() << ", klt " << tracker.getNbFeaturesKlt();
        vpDisplay::displayText(I, 30, 10, ss.str(), vpColor::red);
      }
      vpDisplay::flush(I);

      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);
    //! [Cleanup]
    delete display;
    //! [Cleanup]
  } catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
#ifdef VISP_HAVE_OGRE
  catch (Ogre::Exception &e) {
    std::cout << "Catch an Ogre exception: " << e.getDescription() << std::endl;
    return EXIT_FAILURE;
  }
#endif
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
  return EXIT_SUCCESS;
}
