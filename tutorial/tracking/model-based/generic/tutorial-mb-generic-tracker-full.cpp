//! \example tutorial-mb-generic-tracker-full.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpPlot.h>
//! [Include]
#include <visp3/mbt/vpMbGenericTracker.h>
//! [Include]
#include <visp3/io/vpVideoReader.h>
#include <visp3/io/vpVideoWriter.h>

#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_VIDEOIO) && defined(HAVE_OPENCV_HIGHGUI)
#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

namespace
{
std::vector<double> poseToVec(const vpHomogeneousMatrix &cMo)
{
  vpThetaUVector tu = cMo.getThetaUVector();
  vpTranslationVector t = cMo.getTranslationVector();
  std::vector<double> vec { t[0], t[1], t[2], tu[0], tu[1], tu[2] };

  return vec;
}
}
#endif

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_VIDEOIO) && defined(HAVE_OPENCV_HIGHGUI)
  std::string opt_videoname = "model/teabox/teabox.mp4";
  std::string opt_modelname = "model/teabox/teabox.cao";
  int opt_tracker = 0;
  int opt_video_first_frame = -1;
  int opt_downscale_img = 1;
  bool opt_verbose = false;
  bool opt_plot = true;
  bool opt_display_scale_auto = false;
  vpColVector opt_dof_to_estimate(6, 1.); // Here we consider 6 dof estimation
  std::string opt_save;
#ifdef VISP_HAVE_MINIZ
  std::string opt_save_results;
#endif
  unsigned int thickness = 2;

  vpImage<unsigned char> I;
  std::shared_ptr<vpDisplay> display;
  std::shared_ptr<vpPlot> plot;
  std::shared_ptr<vpVideoWriter> writer;

  try {
    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--video") {
        opt_videoname = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--video-first-frame") {
        opt_video_first_frame = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--model") {
        opt_modelname = std::string(argv[++i]);
      }
      else if (std::string(argv[i]) == "--tracker") {
        opt_tracker = atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--downscale-img") {
        opt_downscale_img = std::atoi(argv[++i]);
      }
      else if (std::string(argv[i]) == "--save") {
        opt_save = std::string(argv[++i]);
      }
#ifdef VISP_HAVE_MINIZ
      else if (std::string(argv[i]) == "--save-results") {
        opt_save_results = std::string(argv[++i]);
      }
#endif
      else if (std::string(argv[i]) == "--plot") {
        opt_plot = true;
      }
      else if (std::string(argv[i]) == "--dof") {
        for (int j = 0; j < 6; j++) {
          int val = std::atoi(argv[++i]);
          if (val == 0 || val == 1) {
            opt_dof_to_estimate[j] = val;
          }
          else {
            std::cout << "Error: wrong value after --dof option. Authorized values are 0 or 1 for each 6 dof to estimate." << std::endl;
            return EXIT_FAILURE;
          }
        }
      }
      else if (std::string(argv[i]) == "--display-scale-auto") {
        opt_display_scale_auto = true;
      }
      else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
        opt_verbose = true;
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nSYNOPSIS " << std::endl
          << argv[0]
          << " [--video <video name>]"
          << " [--video-first-frame <image index>"
          << " [--model <model name>"
          << " [--tracker <0=egde|1=keypoint|2=hybrid>]"
          << " [--downscale-img <scale factor>]"
          << " [--dof <0/1 0/1 0/1 0/1 0/1 0/1>]"
          << " [--save <e.g. results-%04d.png>]"
#ifdef VISP_HAVE_MINIZ
          << " [--save-results <e.g. tracking_poses.npz>]"
#endif
          << " [--display-scale-auto]"
          << " [--plot]"
          << " [--verbose,-v]"
          << " [--help,-h]"
          << std::endl;
        std::cout << "\nOPTIONS " << std::endl
          << "  --video <video name>" << std::endl
          << "      Input video name." << std::endl
          << "      Default: model/teabox/teabox.mp4" << std::endl
          << std::endl
          << "  --video-first-frame <image index>" << std::endl
          << "      Index of the first image to process." << std::endl
          << "      Set to -1 to process the first image of the video." << std::endl
          << "      Default: -1" << std::endl
          << std::endl
          << "  --model <model name>" << std::endl
          << "      CAD model filename. Supported formats are .cao and .wrl." << std::endl
          << "      To use wrl format, ViSP need to be built with Coin3D third-party." << std::endl
          << "      Default: model/teabox/teabox.cao" << std::endl
          << std::endl
          << "  --tracker <0=egde|1=keypoint|2=hybrid>" << std::endl
          << "      Tracker type:" << std::endl
          << "      - when 0: use only moving-edges" << std::endl
          << "      - when 1: use only KLT keypoints" << std::endl
          << "      - when 2: use hybrid scheme, moving-edges and KLT keypoints." << std::endl
          << "      Default: 0" << std::endl
          << std::endl
          << "  --downscale-img <scale factor>" << std::endl
          << "      Downscale input image width and height by this factor." << std::endl
          << "      When set to 1, image not down scaled. When set to 2, image width" << std::endl
          << "      and height is divided by 2." << std::endl
          << "      Default: 1" << std::endl
          << std::endl
          << "  --dof <0/1 0/1 0/1 0/1 0/1 0/1>" << std::endl
          << "      6-dim vector of 0 and 1 to indicate which dof [tx ty tz rx ry rz]" << std::endl
          << "      has to be estimated." << std::endl
          << "      When set to 1 the dof is estimated. When rather set to 0 the dof" << std::endl
          << "      is not estimated. It's value is the one from the initialisation." << std::endl
          << "      Default: 1 1 1 1 1 1 (to estimate all 6 dof)" << std::endl
          << std::endl
          << "  --save <e.g. results-%04d.png>" << std::endl
          << "      Name of the saved image sequence that contains tracking results in overlay." << std::endl
          << "      When the name contains a folder like in the next example, the folder" << std::endl
          << "      is created if it doesn't exist."
          << "      Example: \"result/image-%04d.png\"." << std::endl
          << std::endl
#ifdef VISP_HAVE_MINIZ
          << "  --save-results <e.g. tracking_results.npz>" << std::endl
          << "      Name of the npz file containing cMo data estimated from MBT." << std::endl
          << "      When the name contains a folder like in the next example, the folder" << std::endl
          << "      is created if it doesn't exist."
          << "      Example: \"result/tracking_results.npz\"." << std::endl
          << std::endl
#endif
          << "  --display-scale-auto" << std::endl
          << "      Enable display window auto scaling to ensure that the image is fully" << std::endl
          << "      visible on the screen. Useful for large images." << std::endl
          << "      Note that this option doesn't affect the size of the processed images." << std::endl
          << std::endl
          << "  --plot" << std::endl
          << "      Open a window that plots the estimated pose evolution." << std::endl
          << std::endl
          << "  --verbose, -v" << std::endl
          << "      Enable verbose mode." << std::endl
          << std::endl
          << "  --help, -h" << std::endl
          << "      Display this helper message." << std::endl
          << std::endl;
        return EXIT_SUCCESS;
      }
    }
    std::string parentname = vpIoTools::getParent(opt_modelname);
    std::string objectname = vpIoTools::getNameWE(opt_modelname);

    if (!parentname.empty())
      objectname = parentname + "/" + objectname;

    std::cout << " *********** Tracker config ************ " << std::endl;
    std::cout << "Video name                 : " << opt_videoname << std::endl;
    std::cout << "Tracker cad model file     : " << objectname << ".[cao or wrl]" << std::endl;
    std::cout << "Tracker init file          : " << objectname << ".init" << std::endl;
    std::cout << "Tracker optional init image: " << objectname << ".[png,ppm,jpg]" << std::endl;
    if (opt_downscale_img > 1) {
      std::cout << "Downscale image factor     : " << opt_downscale_img << std::endl;
    }
    std::cout << "Dof to estimate            : " << opt_dof_to_estimate.t() << std::endl;

    // Create output folder if needed
    if (!opt_save.empty()) {
      std::string parent = vpIoTools::getParent(opt_save);
      if (!parent.empty()) {
        std::cout << "Create output directory: " << parent << std::endl;
        vpIoTools::makeDirectory(parent);
      }
    }
#ifdef VISP_HAVE_MINIZ
    if (!opt_save_results.empty()) {
      std::string parent = vpIoTools::getParent(opt_save_results);
      if (!parent.empty()) {
        std::cout << "Create output directory for the npz file: " << parent << std::endl;
        vpIoTools::makeDirectory(parent);
      }
    }
#endif

//! [Image]
    vpImage<unsigned char> Ivideo;
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
    }
    else {
      g.open(I);
    }

    vpImage<vpRGBa> O;
    if (!opt_save.empty()) {
      writer = std::make_shared<vpVideoWriter>();
      writer->setFileName(opt_save);
      writer->open(O);
    }

#if defined(VISP_HAVE_X11)
    display = std::make_shared<vpDisplayX>();
#elif defined(VISP_HAVE_GDI)
    display = std::make_shared<vpDisplayGDI>();
#elif defined(HAVE_OPENCV_HIGHGUI)
    display = std::make_shared<vpDisplayOpenCV>();
#endif
    if (opt_display_scale_auto) {
      display->setDownScalingFactor(vpDisplay::SCALE_AUTO);
    }
    display->init(I, 100, 100, "Model-based tracker");

    if (opt_plot) {
      plot = std::make_shared<vpPlot>(2, 700, 700, display->getWindowXPosition() + I.getWidth() / display->getDownScalingFactor() + 30,
                                      display->getWindowYPosition(), "Estimated pose");
      plot->initGraph(0, 3); // Translation
      plot->setTitle(0, "Translation [m]");
      plot->setColor(0, 0, vpColor::red);
      plot->setColor(0, 1, vpColor::green);
      plot->setColor(0, 2, vpColor::blue);
      plot->initGraph(1, 3); // Attitude
      plot->setTitle(1, "Attitude thetaU [deg]");
      plot->setColor(1, 0, vpColor::red);
      plot->setColor(1, 1, vpColor::green);
      plot->setColor(1, 2, vpColor::blue);
    }

    //! [Constructor]
    vpMbGenericTracker tracker;
    if (opt_tracker == 0)
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER);
#if defined(VISP_HAVE_MODULE_KLT) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
    else if (opt_tracker == 1)
      tracker.setTrackerType(vpMbGenericTracker::KLT_TRACKER);
    else
      tracker.setTrackerType(vpMbGenericTracker::EDGE_TRACKER | vpMbGenericTracker::KLT_TRACKER);
#else
    else {
      std::cout << "klt and hybrid model-based tracker are not available since visp_klt module is not available. "
        "In CMakeGUI turn visp_klt module ON, configure and build ViSP again."
        << std::endl;
      return EXIT_FAILURE;
    }
#endif
    //! [Constructor]

    bool usexml = false;
    //! [Load xml]
#if defined(VISP_HAVE_PUGIXML)
    if (vpIoTools::checkFilename(objectname + ".xml")) {
      std::cout << "Tracker config file        : " << objectname + ".xml" << std::endl;
      tracker.loadConfigFile(objectname + ".xml");
      usexml = true;
    }
#endif
    //! [Load xml]

    if (!usexml) {
      //! [Set parameters]
      if (opt_tracker == 0 || opt_tracker == 2) {
        //! [Set moving-edges parameters]
        vpMe me;
        me.setMaskSize(5);
        me.setMaskNumber(180);
        me.setRange(8);
        me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
        me.setThreshold(20);
        me.setMu1(0.5);
        me.setMu2(0.5);
        me.setSampleStep(4);
        tracker.setMovingEdge(me);
        //! [Set moving-edges parameters]
      }

#if defined(VISP_HAVE_MODULE_KLT) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
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

    if (opt_dof_to_estimate != 1.) {
      tracker.setEstimatedDoF(opt_dof_to_estimate);
    }

    //! [Get camera parameters]
    vpCameraParameters cam;
    tracker.getCameraParameters(cam);
    std::cout << "Camera parameters: \n" << cam << std::endl;
    //! [Get camera parameters]

    std::cout << "Initialize tracker on image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;

    std::vector<double> vec_poses;
#ifdef VISP_HAVE_MINIZ
    if (!opt_save_results.empty()) {
      const unsigned int height = I.getHeight(), width = I.getWidth();
      visp::cnpy::npz_save(opt_save_results, "height", &height, { 1 }, "w");
      visp::cnpy::npz_save(opt_save_results, "width", &width, { 1 }, "a");

      const double cam_px = cam.get_px(), cam_py = cam.get_py(), cam_u0 = cam.get_u0(), cam_v0 = cam.get_v0();
      visp::cnpy::npz_save(opt_save_results, "cam_px", &cam_px, { 1 }, "a");
      visp::cnpy::npz_save(opt_save_results, "cam_py", &cam_py, { 1 }, "a");
      visp::cnpy::npz_save(opt_save_results, "cam_u0", &cam_u0, { 1 }, "a");
      visp::cnpy::npz_save(opt_save_results, "cam_v0", &cam_v0, { 1 }, "a");
    }
#endif

    //! [Init]
    tracker.initClick(I, objectname + ".init", true);
    //! [Init]

    while (!g.end()) {
      if (opt_downscale_img > 1) {
        g.acquire(Ivideo);
        Ivideo.subsample(opt_downscale_img, opt_downscale_img, I);
      }
      else {
        g.acquire(I);
      }
      std::stringstream ss;
      ss << "Process image " << g.getFrameIndex();
      if (opt_verbose) {
        std::cout << "-- " << ss.str() << std::endl;
      }
      vpDisplay::display(I);
      //! [Track]
      tracker.track(I);
      //! [Track]
      //! [Get pose]
      tracker.getPose(cMo);
      //! [Get pose]
      //! [Display]
      tracker.display(I, cMo, cam, vpColor::red, thickness);
      //! [Display]
      vpDisplay::displayFrame(I, cMo, cam, 0.025, vpColor::none, thickness);
      vpDisplay::displayText(I, 20 * display->getDownScalingFactor(), 10 * display->getDownScalingFactor(), "A click to exit...", vpColor::red);
      vpDisplay::displayText(I, 40 * display->getDownScalingFactor(), 10 * display->getDownScalingFactor(), ss.str(), vpColor::red);
      {
        std::stringstream ss;
        ss << "Features";
        if (tracker.getTrackerType() & vpMbGenericTracker::EDGE_TRACKER) {
          ss << " edge: " << tracker.getNbFeaturesEdge();
        }
#if defined(VISP_HAVE_MODULE_KLT) && defined(HAVE_OPENCV_IMGPROC) && defined(HAVE_OPENCV_VIDEO)
        if (tracker.getTrackerType() & vpMbGenericTracker::KLT_TRACKER) {
          ss << " klt: " << tracker.getNbFeaturesKlt();
        }
#endif
        vpDisplay::displayText(I, 60 * display->getDownScalingFactor(), 10 * display->getDownScalingFactor(), ss.str(), vpColor::red);
        if (opt_verbose) {
          std::cout << ss.str() << std::endl;
          std::cout << "cMo:\n" << cMo << std::endl;
        }
      }
      {
        double proj_error = tracker.computeCurrentProjectionError(I, cMo, cam);
        std::stringstream ss;
        ss << "Projection error: " << std::setprecision(2) << proj_error << " deg";
        vpDisplay::displayText(I, 80 * display->getDownScalingFactor(), 10 * display->getDownScalingFactor(), ss.str(), vpColor::red);
        if (opt_verbose) {
          std::cout << ss.str() << std::endl;
        }
      }
      vpDisplay::flush(I);

      if (opt_plot) {
        vpTranslationVector c_t_o = cMo.getTranslationVector();
        vpThetaUVector c_tu_o = vpThetaUVector(cMo.getRotationMatrix());
        vpColVector c_tu_o_deg = vpMath::deg(c_tu_o);
        plot->plot(0, g.getFrameIndex(), c_t_o);
        plot->plot(1, g.getFrameIndex(), c_tu_o_deg);
      }

      if (!opt_save.empty()) {
        vpDisplay::getImage(I, O);
        writer->saveFrame(O);
      }

#ifdef VISP_HAVE_MINIZ
      if (!opt_save_results.empty()) {
        std::vector<double> vec_pose = poseToVec(cMo);
        vec_poses.insert(vec_poses.end(), vec_pose.begin(), vec_pose.end());
      }
#endif

      if (vpDisplay::getClick(I, false))
        break;
    }
    vpDisplay::getClick(I);

#ifdef VISP_HAVE_MINIZ
    if (!opt_save_results.empty()) {
      visp::cnpy::npz_save(opt_save_results, "vec_poses", vec_poses.data(), { static_cast<size_t>(vec_poses.size()/6), 6 }, "a");
    }
#endif
  }
  catch (const vpException &e) {
    std::cout << "Catch a ViSP exception: " << e << std::endl;
    vpDisplay::getClick(I);
  }
#ifdef VISP_HAVE_OGRE
  catch (Ogre::Exception &e) {
    std::cout << "Catch an Ogre exception: " << e.getDescription() << std::endl;
    vpDisplay::getClick(I);
  }
#endif
#else
  (void)argc;
  (void)argv;
  std::cout << "Install OpenCV and rebuild ViSP to use this example." << std::endl;
#endif
  return EXIT_SUCCESS;
      }
