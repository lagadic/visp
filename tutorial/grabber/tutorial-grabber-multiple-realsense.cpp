/*! \example tutorial-grabber-multiple-realsense.cpp */
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpRealSense.h>
#include <visp3/sensor/vpRealSense2.h>

/*!
  Grab images from multiple Intel realsense cameras
 */

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_REALSENSE2) && (RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0)) \
  && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::vector<std::pair<std::string, std::string> > type_serial_nb;
  std::vector<bool> cam_found;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--T265") {
      type_serial_nb.push_back(std::make_pair("T265", std::string(argv[i + 1])));
    }
    else if (std::string(argv[i]) == "--D435") {
      type_serial_nb.push_back(std::make_pair("D435", std::string(argv[i + 1])));
    }
    else if (std::string(argv[i]) == "--SR300") {
      type_serial_nb.push_back(std::make_pair("SR300", std::string(argv[i + 1])));
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
        << " [--T265 <serial number>] [--D435 <serial number>] [--SR300 <serial number>]\n"
        << "\nExample to use 2 T265 cameras:\n"
        << "  " << argv[0] << " --T265 11622110511 --T265 11622110433 \n"
        << "\nExample to use 1 T265 and 1 D435 cameras:\n"
        << "  " << argv[0] << " --T265 11622110511 --D435 752112077408 \n"
        << "\nExample to use 2 T265 and 1 D435 cameras:\n"
        << "  " << argv[0] << " --T265 11622110511 --T265 11622110433 --D435 752112070408 \n"
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  rs2::config T265_cfg, D435_cfg;
  std::vector<vpRealSense2> g(type_serial_nb.size());
  std::vector<vpImage<unsigned char> > I(type_serial_nb.size());

#ifdef VISP_HAVE_X11
  std::vector<vpDisplayX> d(type_serial_nb.size());
#elif defined(VISP_HAVE_GDI)
  std::vector<vpDisplayGDI> d(type_serial_nb.size());
#elif defined(HAVE_OPENCV_HIGHGUI)
  std::vector<vpDisplayOpenCV> d(type_serial_nb.size());
#else
  std::cout << "No image viewer is available..." << std::endl;
#endif

  bool clicked = false;

  for (size_t i = 0; i < type_serial_nb.size(); i++) {
    std::cout << "Opening " << type_serial_nb[i].first << " with ID: " << type_serial_nb[i].second << "." << std::endl;
    if (type_serial_nb[i].first == "T265") { // T265.
      T265_cfg.enable_device(type_serial_nb[i].second);
      T265_cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
      T265_cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);
      cam_found.push_back(g[i].open(T265_cfg));
      if (!cam_found.back()) {
        std::cout << "Device with ID: " << type_serial_nb[i].second << " not found." << std::endl;
      }
    }
    else { // D435 or SR300
      D435_cfg.enable_device(type_serial_nb[i].second);
      D435_cfg.disable_stream(RS2_STREAM_DEPTH);
      D435_cfg.disable_stream(RS2_STREAM_INFRARED);
      D435_cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_RGBA8, 30);
      cam_found.push_back(g[i].open(D435_cfg));
      if (!cam_found.back()) {
        std::cout << "Device with ID: " << type_serial_nb[i].second << " not found." << std::endl;
      }
    }
  }

  while (true) {
    for (size_t i = 0; i < type_serial_nb.size(); i++) {
      if (cam_found[i]) {
        if (type_serial_nb[i].first == "T265") { // T265.
          g[i].acquire(&I[i], nullptr, nullptr);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
          if (!d[i].isInitialised()) {
            d[i].init(I[i], static_cast<int>(100 * i), static_cast<int>(100 * i), "T265 left image");
          }
#endif
        }

        else { // D435.
          g[i].acquire(I[i]);

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV)
          if (!d[i].isInitialised()) {
            d[i].init(I[i], static_cast<int>(100 * i), static_cast<int>(100 * i), type_serial_nb[i].first.c_str());
          }
#endif
        }

        vpDisplay::display(I[i]);
        vpDisplay::flush(I[i]);
        clicked = vpDisplay::getClick(I[i], false);
      }

      if (clicked) {
        break;
      }
    }

    if (clicked) {
      break;
    }
}
#else
  (void)argc;
  (void)argv;
#if !(defined(VISP_HAVE_REALSENSE2))
  std::cout << "Install librealsense version > 2.31.0, configure and build ViSP again to use this example" << std::endl;
#endif
#if !(RS2_API_VERSION > ((2 * 10000) + (31 * 100) + 0))
  std::cout << "librealsense is detected but its version is too old. Install librealsense version > 2.31.0, configure "
    "and build ViSP again to use this example"
    << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
  return EXIT_SUCCESS;
}
