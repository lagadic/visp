/*! \example tutorial-grabber-ids-ueye.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpUeyeGrabber.h>
#include <visp3/io/vpImageStorageWorker.h>

#define USE_COLOR // Comment to acquire gray level images

/*!
  Usage :
    To get the help    : ./tutorial-grabber-ids-ueye --help
    To set the device  : ./tutorial-grabber-ids-ueye
 */
int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_UEYE) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  try {
    unsigned int opt_camera = 0;
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

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--camera")
        opt_camera = (unsigned int)atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--config-file")
        opt_config_file = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--fps")
        opt_fps = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--gain")
        opt_gain = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--shutter")
        opt_shutter = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--subsample")
        opt_subsample = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--white-balance")
        opt_white_balance = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--color-mode")
        opt_color_mode = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--seqname")
        opt_seqname = std::string(argv[i + 1]);
      else if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v")
        opt_verbose = true;
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--camera <index> (default: 0)]"
                  << " [--config-file <filename.ini> (default: empty)]"
                  << " [--fps <auto | value> (default: empty)]"
                  << " [--gain <auto | value in 0 - 100> (default: empty)]"
                  << " [--shutter <auto | exposure value in ms> (default: empty)]"
                  << " [--subsample <1,2,3,4,5,6,8,16> (default: 1)]"
                  << " [--white-balance <0: disabled, 1: enabled> (default: -1)]"
                  << " [--color-mode <mono8, rgb24, rgb32> (default: empty)]"
                  << " [--seqname <sequence name> (default: empty)]"
                  << " [--record <0: continuous | 1: single shot> (default: 0)]"
                  << " [--verbose] [-v]"
                  << " [--help] [-h]\n"
                  << "\nExample to visualize images:\n"
                  << "  " << argv[0] << " \n"
                  << "\nExample to visualize images and set camera parameter from a config file:\n"
                  << "  " << argv[0] << " --config-file UI-388xCP-C.ini\n"
                  << "\nExamples to record a sequence:\n"
                  << "  " << argv[0] << " --seqname I%04d.png \n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 0\n"
                  << "\nExamples to record single shot images:\n"
                  << "  " << argv[0] << " --seqname I%04d.png --record 1\n"
                  << "  " << argv[0] << " --seqname folder/I%04d.png --record 1\n"
                  << std::endl;
        return 0;
      }
    }

    //! [Create image]
#ifdef USE_COLOR
    vpImage<vpRGBa> I;        // To acquire color images
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

    if (! cam_ids.size()) {
      std::cout << "No camera detected. Plug a camera and try again..." << std::endl;
      return EXIT_FAILURE;
    }
    std::cout << "Found " << cam_ids.size() << " cameras :"<< std::endl;
    for (unsigned int i = 0; i < cam_ids.size(); i++) {
      std::cout << (opt_camera == i ? " * Camera " : "   Camera ") << i
                << " - ID: " << cam_ids[i]  << " Model: " << cam_models[i] << " S/N: " << cam_serials[i] << std::endl;
    }
    //! [List camera info]

    //! [Active camera info]
    if (! g.setActiveCamera(opt_camera)) {
      std::cout << "Unable to select camera " << opt_camera << std::endl;
      return EXIT_FAILURE;
    };
    //! [Active camera info]

    std::cout << "Active camera is Model " << g.getActiveCameraModel() << " with S/N: " << g.getActiveCameraSerialNumber() << std::endl;

    //! [Open connection]
    g.open(I);
    //! [Open connection]

    if (! opt_config_file.empty()) {
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

    if (! opt_gain.empty()) {
      if (opt_gain == "auto") {
        std::cout << "Auto gain         : " << (g.setGain(true) ? "enabled" : "N/A") << std::endl;
      }
      else {
        std::cout << "Manual gain       : " << (g.setGain(false, std::atoi(opt_gain.c_str())) ? (std::string(opt_gain) + " %") : "N/A") << std::endl;
      }
    }
    if (! opt_shutter.empty()) {
      if (opt_shutter == "auto") {
        std::cout << "Auto shutter      : " << (g.setExposure(true) ? "enabled" : "N/A") << std::endl;
      }
      else {
        std::cout << "Manual shutter    : " << (g.setExposure(false, std::atof(opt_shutter.c_str())) ? (std::string(opt_shutter) + " ms") : "N/A") << std::endl;
      }
    }

    if (opt_white_balance > 0) {
      bool wb = (opt_white_balance ? true : false);
      std::cout << "Subsampling factor: " << opt_subsample << std::endl;
      std::cout << "White balance     : " << (wb ? "auto" : "disabled") << std::endl;
      g.setWhiteBalance(wb);
    }

    if (! opt_color_mode.empty()) {
      if (g.setColorMode(opt_color_mode)) {
        std::cout << "Color mode        : " << opt_color_mode << std::endl;
      }
    }

    if (! opt_fps.empty()) {
      if (opt_fps == "auto") {
        std::cout << "Auto framerate    : " << (g.setFrameRate(true) ? "enabled" : "N/A") << std::endl;
      }
      else {
        std::cout << "Manual framerate  : " << (g.setFrameRate(false, std::atof(opt_fps.c_str())) ? (std::string(opt_fps) + " Hz") : "N/A") << std::endl;
      }
    }

    std::cout << "Recording         : " << (opt_seqname.empty() ? "disabled" : "enabled") << std::endl;

    std::string text_record_mode = std::string("Record mode       : ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    if (! opt_seqname.empty()) {
      std::cout << text_record_mode << std::endl;
      std::cout << "Record name       : " << opt_seqname << std::endl;
    }

    std::cout << "Config file       : " << (opt_config_file.empty() ? "empty" : opt_config_file) << std::endl;
    std::cout << "Image size        : " << I.getWidth() << " " << I.getHeight() << std::endl;

#ifdef VISP_HAVE_X11
    vpDisplayX d;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d;
#else
    std::cout << "No image viewer is available..." << std::endl;
#endif

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
    d.setDownScalingFactor(vpDisplay::SCALE_AUTO);
    d.init(I);
#endif

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
    while (! quit) {
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
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void) argc;
  (void) argv;
#ifndef VISP_HAVE_UEYE
  std::cout << "Install IDS uEye SDK, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This turorial should be built with c++11 support" << std::endl;
#endif
#endif
}
