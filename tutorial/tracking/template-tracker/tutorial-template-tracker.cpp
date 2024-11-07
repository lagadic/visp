/*! \example tutorial-template-tracker.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
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

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--videoname")
      opt_videoname = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--subsample")
      opt_subsample = static_cast<unsigned int>(std::atoi(argv[i + 1]));
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0] << " [--videoname <video name>] [--subsample <scale factor>] [--help] [-h]\n"
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

#if defined(VISP_HAVE_X11)
  vpDisplayX display;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI display;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV display;
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif
  display.setDownScalingFactor(vpDisplay::SCALE_AUTO);
  display.init(I, 100, 100, "Template tracker");
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
#else
  (void)argc;
  (void)argv;
#endif
}
