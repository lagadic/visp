/*! \example tutorial-grabber-structure-core.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageStorageWorker.h>
#include <visp3/sensor/vpOccipitalStructure.h>

void usage(const char *argv[], int error)
{
  std::cout << "SYNOPSIS" << std::endl
    << "  " << argv[0] << " [--depth-fps <6|15|30|60>]"
    << " [--depth-fps <6|15|30|60>]"
    << " [--sxga]"
    << " [--no-frame-sync]"
    << " [--record <mode>]"
    << " [--no-display]"
    << " [--help] [-h]" << std::endl
    << std::endl;
  std::cout << "DESCRIPTION" << std::endl
    << "  --visible-fps <6|15|30|60>" << std::endl
    << "    Visible camera (gray or color) frames per second." << std::endl
    << "    Default: 30." << std::endl
    << std::endl
    << "  --depth-fps <6|15|30|60>" << std::endl
    << "    Depth camera frames per second." << std::endl
    << "    Default: 30." << std::endl
    << std::endl
    << "  --sxga" << std::endl
    << "    If available, output 1280x960 high resolution depth array." << std::endl
    << std::endl
    << "  --no-frame-sync" << std::endl
    << "    If available, disable frame synchronization." << std::endl
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
    << "  --help, -h" << std::endl
    << "    Print this helper message." << std::endl
    << std::endl;
  std::cout << "USAGE" << std::endl
    << "  Example to visualize images:" << std::endl
    << "    " << argv[0] << std::endl
    << std::endl
    << "  Example to record a sequence of images:" << std::endl
    << "    " << argv[0] << " --record 0" << std::endl
    << std::endl
    << "  Example to record a sequence of images at different frame rates:" << std::endl
    << "    " << argv[0] << " --record 0 --depth-fps 15 --visible-fps 10 --no-frame-sync" << std::endl
    << std::endl
    << "  Example to record single shot images:\n"
    << "    " << argv[0] << " --record 1" << std::endl
    << std::endl;

  if (error) {
    std::cout << "Error" << std::endl
      << "  "
      << "Unsupported parameter " << argv[error] << std::endl;
  }
}

/*!
  Grab images from an Occipital Structure Core device.
 */
int main(int argc, const char *argv[])
{
#if defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && defined(VISP_HAVE_THREADS)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    std::string opt_seqname_visible = "visible-%04d.png", opt_seqname_depth = "depth-%04d.png";
    int opt_record_mode = 0;
    int opt_depth_fps = 30, opt_visible_fps = opt_depth_fps; // frame synchronization by default.
    bool opt_sxga = false;                                   // Used for high resolution depth array (true => 1280x960).
    bool opt_frame_sync = true;                              // Used to set/unset frame synchronization (default: true).
    bool opt_display = true;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--depth-fps") {
        opt_depth_fps = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--visible-fps") {
        opt_visible_fps = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--sxga") {
        opt_sxga = true;
      }
      else if (std::string(argv[i]) == "--no-frame-sync") {
        opt_frame_sync = false;
      }
      else if (std::string(argv[i]) == "--record") {
        opt_record_mode = std::atoi(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--no-display") {
        opt_display = false;
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

    if (!opt_display) {
      opt_record_mode = 0;
    }

    std::cout << "Depth framerate  : " << opt_depth_fps << std::endl;
    std::cout << "Visible framerate: " << opt_visible_fps << std::endl;
    std::cout << "Display          : " << (opt_display ? "enabled" : "disabled") << std::endl;

    std::string text_record_mode =
      std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    std::cout << text_record_mode << std::endl;
    std::cout << "Visible record name: " << opt_seqname_visible << std::endl;
    std::cout << "Depth record name: " << opt_seqname_depth << std::endl;

    vpImage<vpRGBa> I_color, I_depth;
    vpImage<float> I_depth_raw;

    vpOccipitalStructure g;

    // There's an issue in the firmware when visible fps is set between 1 Hz and 2 Hz.
    // The visible frame is damaged. The depth frame can be streamed at 1Hz without any problem.
    if (opt_visible_fps < 2) {
      opt_visible_fps = 2;
    }

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.visibleEnabled = true;
    settings.frameSyncEnabled = opt_frame_sync;
    settings.structureCore.depthFramerate = opt_depth_fps;
    settings.structureCore.visibleFramerate = opt_visible_fps;
    if (opt_sxga)
      settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::SXGA;
    settings.applyExpensiveCorrection = true; // Apply a correction and clean filter to the depth before streaming.

    bool is_open = g.open(settings);

    if (is_open) {
      // Wait some time to at least have 1 frame of each enabled stream (worst case scenario fps = 1Hz).
      vpTime::wait(1000);
      I_color = vpImage<vpRGBa>(g.getHeight(vpOccipitalStructure::visible), g.getWidth(vpOccipitalStructure::visible));
      I_depth = vpImage<vpRGBa>(g.getHeight(vpOccipitalStructure::depth), g.getWidth(vpOccipitalStructure::depth));
      I_depth_raw = vpImage<float>(g.getHeight(vpOccipitalStructure::depth), g.getWidth(vpOccipitalStructure::depth));

      vpDisplay *display_visible = nullptr, *display_depth = nullptr;
      if (opt_display) {
#if !(defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV))
        std::cout << "No image viewer is available..." << std::endl;
        opt_display = false;
#endif
      }
      if (opt_display) {
#ifdef VISP_HAVE_X11
        display_visible = new vpDisplayX(I_color, 10, 10, "Visible image");
        display_depth = new vpDisplayX(I_depth, 10 + I_color.getWidth(), 10, "Depth image");
#elif defined(VISP_HAVE_GDI)
        display_visible = new vpDisplayGDI(I_color, 10, 10, "Visible image");
        display_depth = new vpDisplayGDI(I_depth, 10 + I_color.getWidth(), 10, "Depth image");
#elif defined(HAVE_OPENCV_HIGHGUI)
        display_visible = new vpDisplayOpenCV(I_color, 10, 10, "Visible image");
        display_depth = new vpDisplayOpenCV(I_depth, 10 + I_color.getWidth(), 10, "Depth image");
#endif
      }

      vpImageQueue<vpRGBa> image_queue_visible(opt_seqname_visible, opt_record_mode);
      std::thread image_visible_storage_thread;

      vpImageStorageWorker<vpRGBa> image_visible_storage_worker(std::ref(image_queue_visible));
      image_visible_storage_thread = std::thread(&vpImageStorageWorker<vpRGBa>::run, &image_visible_storage_worker);

      vpImageQueue<vpRGBa> image_queue_depth(opt_seqname_depth, opt_record_mode);
      vpImageStorageWorker<vpRGBa> image_depth_storage_worker(std::ref(image_queue_depth));
      std::thread image_depth_storage_thread(&vpImageStorageWorker<vpRGBa>::run, &image_depth_storage_worker);

      bool quit = false;
      double t;
      while (!quit) {
        t = vpTime::measureTimeMs();

        g.acquire((unsigned char *)I_color.bitmap, (unsigned char *)I_depth_raw.bitmap);

        vpDisplay::display(I_color);
        vpImageConvert::createDepthHistogram(I_depth_raw, I_depth);
        vpDisplay::display(I_depth);

        quit = image_queue_visible.record(I_color);
        quit |= image_queue_depth.record(I_depth, nullptr, image_queue_visible.getRecordingTrigger(), true);

        std::stringstream ss;
        ss << "Acquisition time: " << std::setprecision(3) << vpTime::measureTimeMs() - t << " ms";
        vpDisplay::displayText(I_depth, I_depth.getHeight() - 20, 10, ss.str(), vpColor::red);
        vpDisplay::flush(I_color);
        vpDisplay::flush(I_depth);
      }
      image_queue_visible.cancel();
      image_queue_depth.cancel();
      image_visible_storage_thread.join();
      image_depth_storage_thread.join();

      if (display_visible) {
        delete display_visible;
      }
      if (display_depth) {
        delete display_depth;
      }
    }
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void)argc;
  (void)argv;
#if !(defined(VISP_HAVE_OCCIPITAL_STRUCTURE))
  std::cout << "Install libStructure, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
