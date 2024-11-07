/*! \example tutorial-grabber-ids-ueye.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/sensor/vpUeyeGrabber.h>

#define USE_COLOR // Comment to acquire gray level images

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--device <index>]"
    << " [--config-file <filename.ini>]"
    << " [--fps <auto|fps value like 6|15|30|60>]"
    << " [--gain <auto|value in 0 - 100>]"
    << " [--shutter <auto|exposure value in ms>]"
    << " [--subsample <factor>]"
    << " [--white-balance <value>]"
    << " [--color-mode <mode>]"
    << " [--seqname <sequence name>]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--verbose] [-v]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --device <index>" << std::endl
    << "    Camera device index. Set 0 to dial with the first camera," << std::endl
    << "    and 1 to dial with the second camera attached to the computer." << std::endl
    << "    Default: 0" << std::endl
    << std::endl
    << "  --config-file <filename.ini>" << std::endl
    << "    Camera config file." << std::endl
    << "    Default: empty." << std::endl
    << std::endl
    << "  --fps <auto|fps value like 6|15|30|60>" << std::endl
    << "    \"Auto\" or a frames per second value." << std::endl
    << "    Default: current setting." << std::endl
    << std::endl
    << "  --gain <auto|value in 0 - 100>" << std::endl
    << "    \"Auto\" or manual gain with a value in 0 - 100." << std::endl
    << "    Default: current setting." << std::endl
    << std::endl
    << "  --shutter <auto|manu>" << std::endl
    << "    \"Auto\" or manual shutter." << std::endl
    << "    Default: current setting." << std::endl
    << std::endl
    << "  --subsample <factor>" << std::endl
    << "    Subsample factor to reduce image size alog rows and columns." << std::endl
    << "    Default: 1." << std::endl
    << std::endl
    << "  --white-balance <value>" << std::endl
    << "    Possible values are 0 (disabled) or 1 (enabled)." << std::endl
    << "    Default: current setting." << std::endl
    << std::endl
    << "  --color-mode <mode>" << std::endl
    << "    Possible modes are: mono8, rgb24, rgb32, bayer8." << std::endl
    << "    Default: current setting." << std::endl
    << std::endl
    << "  --seqname <sequence name>" << std::endl
    << "    Name of the sequence of image to create (ie: /tmp/image%04d.jpg)." << std::endl
    << "    Default: empty." << std::endl
    << std::endl
    << "  --record <mode>" << std::endl
    << "    Allowed values for mode are:" << std::endl
    << "      0: record all the captures images (continuous mode)," << std::endl
    << "      1: record only images selected by a user click (single shot mode)." << std::endl
    << "    Default mode: 0" << std::endl
    << std::endl
    << "  --no-display" << std::endl
    << "    Disable displaying captured images." << std::endl
    << "    When used and sequence name specified, record mode is internally set to 1 (continuous mode)."
    << std::endl
    << std::endl
    << "  --verbose, -v" << std::endl
    << "    Enable extra printings." << std::endl
    << std::endl
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  std::cout << "USAGE" << std::endl
    << "  Example to visualize images:" << std::endl
    << "    " << argv[0] << std::endl
    << std::endl
    << "  Examples to record a sequence of images:" << std::endl
    << "    " << argv[0] << " --seqname I%04d.png" << std::endl
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 0" << std::endl
    << std::endl
    << "  Examples to record single shot images:\n"
    << "    " << argv[0] << " --seqname I%04d.png --record 1\n"
    << "    " << argv[0] << " --seqname folder/I%04d.png --record 1" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

/*!
  Usage :
    To get the help    : ./tutorial-grabber-ids-ueye --help
    To set the device  : ./tutorial-grabber-ids-ueye
 */
int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_UEYE) && defined(VISP_HAVE_THREADS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    unsigned int opt_device = 0;
    std::string opt_seqname;
    int opt_record_mode = 0;
    std::string opt_config_file = "";
    std::string opt_fps = "";
    std::string opt_gain = "";
    std::string opt_shutter = "";
    std::string opt_color_mode = "";
    int opt_white_balance = -1;
    int opt_subsample = 1;
    bool opt_verbose = false;
    bool opt_display = true;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--device") {
        opt_device = static_cast<unsigned int>(std::atoi(argv[i + 1]));
        i++;
      }
      else if (std::string(argv[i]) == "--config-file") {
        opt_config_file = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--fps") {
        opt_fps = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--gain") {
        opt_gain = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--shutter") {
        opt_shutter = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--subsample") {
        opt_subsample = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--white-balance") {
        opt_white_balance = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--color-mode") {
        opt_color_mode = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--seqname") {
        opt_seqname = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--record") {
        opt_record_mode = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
        opt_verbose = true;
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        usage(argv, 0);
        return EXIT_SUCCESS;
      }
      else {
        usage(argv, i);
        return EXIT_FAILURE;
      }
    }

    if ((!opt_display) && (!opt_seqname.empty())) {
      opt_record_mode = 0;
    }

    //! [Create image]
#ifdef USE_COLOR
    vpImage<vpRGBa> I; // To acquire color images
#else
    vpImage<unsigned char> I; // To acquire gray images
#endif
    //! [Create image]

    //! [List camera info]
    vpUeyeGrabber g;

    // Get info on connected cameras
    std::vector<unsigned int> cam_ids = g.getCameraIDList();
    std::vector<std::string> cam_models = g.getCameraModelList();
    std::vector<std::string> cam_serials = g.getCameraSerialNumberList();

    if (!cam_ids.size()) {
      std::cout << "No camera detected. Plug a camera and try again..." << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "Found " << cam_ids.size() << " cameras :" << std::endl;
    for (unsigned int i = 0; i < cam_ids.size(); i++) {
      std::cout << (opt_device == i ? " * Camera " : "   Camera ") << i << " - ID: " << cam_ids[i]
        << " Model: " << cam_models[i] << " S/N: " << cam_serials[i] << std::endl;
    }
    //! [List camera info]

    //! [Active camera info]
    if (!g.setActiveCamera(opt_device)) {
      std::cout << "Unable to select camera " << opt_device << std::endl;
      return EXIT_FAILURE;
    };
    //! [Active camera info]

    std::cout << "Active camera is Model " << g.getActiveCameraModel()
      << " with S/N: " << g.getActiveCameraSerialNumber() << std::endl;

    //! [Open connection]
    g.open(I);
    //! [Open connection]

    if (!opt_config_file.empty()) {
      //! [Load settings from file]
      g.loadParameters(opt_config_file);
      //! [Load settings from file]
      // Since loaded parameters may affect image size, rescale image in case of
      //! [Update image size]
      I.resize(g.getFrameHeight(), g.getFrameWidth());
      //! [Update image size]
    }

    if (opt_subsample > 1) {
      std::cout << "Subsampling factor: " << opt_subsample << std::endl;
      g.setSubsampling(opt_subsample);
      // Since subsampling may affect image size, rescale image in case of
      I.resize(g.getFrameHeight(), g.getFrameWidth());
    }

    if (!opt_gain.empty()) {
      if (opt_gain == "auto") {
        std::cout << "Auto gain         : " << (g.setGain(true) ? "enabled" : "N/A") << std::endl;
      }
      else {
        std::cout << "Manual gain       : "
          << (g.setGain(false, std::atoi(opt_gain.c_str())) ? (std::string(opt_gain) + " %") : "N/A")
          << std::endl;
      }
    }
    if (!opt_shutter.empty()) {
      if (opt_shutter == "auto") {
        std::cout << "Auto shutter      : " << (g.setExposure(true) ? "enabled" : "N/A") << std::endl;
      }
      else {
        std::cout << "Manual shutter    : "
          << (g.setExposure(false, std::atof(opt_shutter.c_str())) ? (std::string(opt_shutter) + " ms") : "N/A")
          << std::endl;
      }
    }

    if (opt_white_balance > 0) {
      bool wb = (opt_white_balance ? true : false);
      std::cout << "Subsampling factor: " << opt_subsample << std::endl;
      std::cout << "White balance     : " << (wb ? "auto" : "disabled") << std::endl;
      g.setWhiteBalance(wb);
    }

    if (!opt_color_mode.empty()) {
      if (g.setColorMode(opt_color_mode)) {
        std::cout << "Color mode        : " << opt_color_mode << std::endl;
      }
    }

    if (!opt_fps.empty()) {
      if (opt_fps == "auto") {
        std::cout << "Auto framerate    : " << (g.setFrameRate(true) ? "enabled" : "N/A") << std::endl;
      }
      else {
        std::cout << "Manual framerate  : "
          << (g.setFrameRate(false, std::atof(opt_fps.c_str())) ? (std::string(opt_fps) + " Hz") : "N/A")
          << std::endl;
      }
    }

    std::cout << "Recording         : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;
    std::cout << "Display           : " << (opt_display ? "enabled" : "disabled") << std::endl;

    std::string text_record_mode =
      std::string("Record mode       : ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (!opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name       : " << opt_seqname << std::endl;
    }

    std::cout << "Config file       : " << (opt_config_file.empty() ? "empty" : opt_config_file) << std::endl;
    std::cout << "Image size        : " << I.getWidth() << " " << I.getHeight() << std::endl;

    vpDisplay *d = nullptr;
    if (opt_display) {
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
      std::cout << "No image viewer is available..." << std::endl;
      opt_display = false;
#endif
    }
    if (opt_display) {
#ifdef VISP_HAVE_X11
      d = new vpDisplayX;
#elif defined(VISP_HAVE_GDI)
      d = new vpDisplayGDI;
#elif defined(HAVE_OPENCV_HIGHGUI)
      d = new vpDisplayOpenCV;
#endif
      d->setDownScalingFactor(vpDisplay::SCALE_AUTO);
      d->init(I);
    }

#ifdef USE_COLOR
    vpImageQueue<vpRGBa> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<vpRGBa> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<vpRGBa>::run, &image_storage_worker);
#else
    vpImageQueue<unsigned char> image_queue(opt_seqname, opt_record_mode);
    vpImageStorageWorker<unsigned char> image_storage_worker(std::ref(image_queue));
    std::thread image_storage_thread(&vpImageStorageWorker<unsigned char>::run, &image_storage_worker);
#endif

    bool quit = false;
    double timestamp_camera = 0, timestamp_camera_prev = 0;
    std::string timestamp_system;
    while (!quit) {
      g.acquire(I, &timestamp_camera, &timestamp_system);
      double fps = g.getFramerate();

      vpDisplay::display(I);

      quit = image_queue.record(I, &timestamp_system);

      if (opt_verbose) {
        std::cout << "System timestamp: " << timestamp_system << std::endl;
        std::cout << "Camera timestamp diff: " << timestamp_camera - timestamp_camera_prev << std::endl;
        timestamp_camera_prev = timestamp_camera;
      }
      vpDisplay::displayText(I, static_cast<int>(I.getHeight() - 40 * vpDisplay::getDownScalingFactor(I)),
                             static_cast<int>(10 * vpDisplay::getDownScalingFactor(I)), timestamp_system, vpColor::red);
      {
        std::stringstream ss;
        ss << "Camera framerate: " << fps;
        vpDisplay::displayText(I, static_cast<int>(I.getHeight() - 60 * vpDisplay::getDownScalingFactor(I)),
                               static_cast<int>(10 * vpDisplay::getDownScalingFactor(I)), ss.str(), vpColor::red);
      }

      vpDisplay::flush(I);
    }
    image_queue.cancel();
    image_storage_thread.join();

    if (d) {
      delete d;
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
}
#else
  (void)argc;
  (void)argv;
#ifndef VISP_HAVE_UEYE
  std::cout << "Install IDS uEye SDK, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
