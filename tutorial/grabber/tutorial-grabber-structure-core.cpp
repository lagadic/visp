/*! \example tutorial-grabber-structure-core.cpp */
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpOccipitalStructure.h>
#include <visp3/io/vpImageStorageWorker.h>

/*!
  Grab images from an Occipital Structure Core device.
 */
int main(int argc, char **argv)
{
#if defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  try {
    std::string opt_seqname_visible = "visible-%04d.png", opt_seqname_depth = "depth-%04d.png";
    int opt_record_mode = 0;
    int opt_depth_fps = 30, opt_visible_fps = opt_depth_fps; // frame synchronization by default.
    bool sxga = false; // Used for high resolution depth array (true => 1280x960).
    bool frame_sync = true; // Used to set/unset frame synchronization (default: true).

    for (int i = 0; i < argc; i++) {
      if (std::string(argv[i]) == "--record")
        opt_record_mode = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--depth_fps")
        opt_depth_fps = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--visible_fps")
        opt_visible_fps = std::atoi(argv[i + 1]);
      else if (std::string(argv[i]) == "--sxga")
        sxga = true;
      else if (std::string(argv[i]) == "--no_frame_sync")
        frame_sync = false;
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "\nUsage: " << argv[0]
                  << " [--record <0: continuous | 1: single shot (default: 0)>]"
                     " [--depth_fps <depth_frames_per_seconds> (default: 30)]"
                     " [--visible_fps <visible_frames_per_seconds> (default: 30)]"
                     " [--sxga (if exists, output 1280x960 depth array)]"
                     " [--no_frame_sync (if exists, disable frame synchronization)]"
                     " [--help] [-h]\n"
                  << "\nExample to visualize images:\n"
                  << "  " << argv[0] << "\n"
                  << "\nExamples to record single shot images:\n"
                  << "  " << argv[0] << " --record 1\n"
                  << "\nExamples to record a sequence of images:\n"
                  << "  " << argv[0] << " --record 0\n"
                  << "\nExamples to record a sequence of images at different frame rates:\n"
                  << "  " << argv[0] << " --record 0 --depth_fps 15 --visible_fps 10 --no_frame_sync\n"
                  << std::endl;
        return 0;
      }
    }

    std::cout << "Depth framerate  : " << opt_depth_fps << std::endl;
    std::cout << "Visible framerate  : " << opt_visible_fps << std::endl;

    std::string text_record_mode = std::string("Record mode: ") + (opt_record_mode ? std::string("single") : std::string("continuous"));

    std::cout << text_record_mode << std::endl;
    std::cout << "Visible record name: " << opt_seqname_visible << std::endl;
    std::cout << "Depth record name: " << opt_seqname_depth << std::endl;

    vpImage<vpRGBa> I_color, I_depth;
    vpImage<float> I_depth_raw;

    vpOccipitalStructure g;

    // There's an issue in the firmware when visible fps is set between 1 Hz and 2 Hz.
    // The visible frame is damaged. The depth frame can be streamed at 1Hz without any problem.
    if(opt_visible_fps < 2)
    {
      opt_visible_fps = 2;
    }

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.visibleEnabled = true;
    settings.frameSyncEnabled = frame_sync;
    settings.structureCore.depthFramerate = opt_depth_fps;
    settings.structureCore.visibleFramerate = opt_visible_fps;
    if(sxga)
      settings.structureCore.depthResolution = ST::StructureCoreDepthResolution::SXGA;
    settings.applyExpensiveCorrection = true; // Apply a correction and clean filter to the depth before streaming.

    bool is_open = g.open(settings);

    if(is_open) {
    // Wait some time to at least have 1 frame of each enabled stream (worst case scenario fps = 1Hz).
    vpTime::wait(1000);
    I_color = vpImage<vpRGBa>(g.getHeight(vpOccipitalStructure::visible), g.getWidth(vpOccipitalStructure::visible));
    I_depth = vpImage<vpRGBa>(g.getHeight(vpOccipitalStructure::depth), g.getWidth(vpOccipitalStructure::depth));
    I_depth_raw = vpImage<float>(g.getHeight(vpOccipitalStructure::depth), g.getWidth(vpOccipitalStructure::depth));

#if defined(VISP_HAVE_X11)
    vpDisplayX display_visible;  // Visible image
    display_visible.init(I_color, 10, 10, "Visible image");
    vpDisplayX display_depth; // Depth image
    display_depth.init(I_depth, 10+I_color.getWidth(), 10, "Depth image");
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display_visible;  // Visible image
    display_visible.init(I_color, 10, 10, "Visible image");
    vpDisplayGDI display_depth; // Depth image
    display_depth.init(I_depth, 10, 10, "Depth image");
#else
    std::cout << "No viewer is installed..." << std::endl;
#endif

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
      quit |= image_queue_depth.record(I_depth, NULL, image_queue_visible.getRecordingTrigger(), true);

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
  }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
#else
  (void) argc;
  (void) argv;
#if !(defined(VISP_HAVE_OCCIPITAL_STRUCTURE))
  std::cout << "Install libStructure, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This turorial should be built with c++11 support" << std::endl;
#endif
#endif
}
