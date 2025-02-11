//! \example tutorial-mb-tracker.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpImageIo.h>
//! [Include]
#include <visp3/mbt/vpMbEdgeKltTracker.h>
#include <visp3/mbt/vpMbEdgeTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) && defined(VISP_HAVE_DISPLAY)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay();
#else
  vpDisplay *display = vpDisplayFactory::allocateDisplay();
#endif
  vpMbTracker *tracker = nullptr;

  try {
    std::string opt_videoname = "teabox.mp4";
    std::string opt_modelname = "teabox";
    int opt_tracker = 0;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--video" && i + 1 < argc) {
        opt_videoname = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--model" && i + 1 < argc) {
        opt_modelname = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--tracker" && i + 1 < argc) {
        opt_tracker = atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
          << " [--video <video name>]"
          << " [--model <model name>] "
          << " [--tracker <0=egde|1=keypoint|2=hybrid>]"
          << " [--help] [-h]\n"
          << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
        if (display != nullptr) {
          delete display;
        }
#endif
        return EXIT_SUCCESS;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_modelname);
    std::string objectname = vpIoTools::getNameWE(opt_modelname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << "Video name: " << opt_videoname << std::endl;
    std::cout << "Tracker requested config files: " << objectname << ".[init, cao]" << std::endl;
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

    display->init(I, 100, 100, "Model-based tracker");

    //! [Constructor]
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
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
      if (display != nullptr) {
        delete display;
      }
#endif
      return EXIT_FAILURE;
    }
#endif
    //! [Constructor]

    //! [Set parameters]
    if (opt_tracker == 0 || opt_tracker == 2) {
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
    }

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    if (opt_tracker == 1 || opt_tracker == 2) {
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
  }
  catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
    if (display != nullptr) {
      delete display;
    }
#endif
    if (tracker != nullptr) {
      delete tracker;
    }
    return EXIT_FAILURE;
  }

  //! [Cleanup]
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  if (display != nullptr) {
    delete display;
  }
#endif
  if (tracker != nullptr) {
    delete tracker;
  }
    //! [Cleanup]
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
  return EXIT_SUCCESS;
  }
