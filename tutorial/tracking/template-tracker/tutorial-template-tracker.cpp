/*! \example tutorial-template-tracker.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayFactory.h>
#include <visp3/io/vpVideoReader.h>
//! [Include]
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>
//! [Include]

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string opt_videoname = "bruegel.mp4";
  unsigned int opt_subsample = 1;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--videoname" && i + 1 < argc) {
      opt_videoname = std::string(argv[++i]);
    }
    else if (std::string(argv[i]) == "--subsample" && i + 1 < argc) {
      opt_subsample = static_cast<unsigned int>(std::atoi(argv[++i]));
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
        << " [--videoname <video name>]"
        << " [--subsample <scale factor>] [--help] [-h]\n"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  std::cout << "Video name: " << opt_videoname << std::endl;

  vpImage<unsigned char> I, Iacq;

  vpVideoReader g;
  g.setFileName(opt_videoname);
  g.open(Iacq);
  Iacq.subsample(opt_subsample, opt_subsample, I);

#if defined(VISP_HAVE_DISPLAY)
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  std::shared_ptr<vpDisplay> display = vpDisplayFactory::createDisplay(I, 100, 100, "Template tracker", vpDisplay::SCALE_AUTO);
#else
  vpDisplay *display = vpDisplayFactory::allocateDisplay(I, 100, 100, "Template tracker", vpDisplay::SCALE_AUTO);
#endif
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif
  vpDisplay::display(I);
  vpDisplay::flush(I);

  //! [Construction]
  vpTemplateTrackerWarpHomography warp;
  vpTemplateTrackerSSDInverseCompositional tracker(&warp);
  //! [Construction]

  tracker.setSampling(2, 2);
  tracker.setLambda(0.001);
  tracker.setIterationMax(200);
  tracker.setPyramidal(2, 1);

  //! [Init]
  tracker.initClick(I);
  //! [Init]

  while (1) {
    double t = vpTime::measureTimeMs();
    g.acquire(Iacq);
    Iacq.subsample(opt_subsample, opt_subsample, I);
    vpDisplay::display(I);

    //! [Track]
    tracker.track(I);
    //! [Track]

    //! [Homography]
    vpColVector p = tracker.getp();
    vpHomography H = warp.getHomography(p);
    std::cout << "Homography: \n" << H << std::endl;
    //! [Homography]

    //! [Display]
    tracker.display(I, vpColor::red);
    //! [Display]

    vpDisplay::displayText(I, 10 * vpDisplay::getDownScalingFactor(I), 10 * vpDisplay::getDownScalingFactor(I),
                           "Click to quit", vpColor::red);
    if (vpDisplay::getClick(I, false))
      break;

    vpDisplay::flush(I);
    if (!g.isVideoFormat()) {
      vpTime::wait(t, 40);
    }
  }

#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11) && defined(VISP_HAVE_DISPLAY)
  if (display != nullptr) {
    delete display;
  }
#endif
#else
  (void)argc;
  (void)argv;
#endif
}
