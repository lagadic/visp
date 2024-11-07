/*! \example tutorial-grabber-rgbd-D435-structurecore.cpp */
#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpOccipitalStructure.h>
#include <visp3/sensor/vpRealSense2.h>

/*!
 * Grab color and depth images from Intel RealSense D435 and Occipital Structure Core sensors.
 */
int main(int argc, char **argv)
{
#if defined(VISP_HAVE_REALSENSE2) && defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  // Both cameras can stream color and depth in 640x480 resolution.
  unsigned int width = 640, height = 480;

  // Grabbers.
  vpRealSense2 rs;
  vpOccipitalStructure sc;

  // Required images to acquire and display color and depth.
  vpImage<float> sc_I_depth_raw(height, width);
  vpImage<uint16_t> rs_I_depth_raw(height, width);
  vpImage<vpRGBa> I_depth_sc(height, width), I_depth_rs(height, width);
  vpImage<vpRGBa> I_color_sc(height, width), I_color_rs(height, width);

  vpDisplayX d_rs_depth(I_depth_rs, 10, height + 10, "RealSense Depth"),
    d_sc_depth(I_depth_sc, width + 10, height + 10, "Structure Core Depth");
  vpDisplayX d_color_rs(I_color_rs, 10, 10, "RealSense Color"),
    d_color_sc(I_color_sc, width + 10, 10, "Structure Core Color");

  // Configuring and opening RealSense grabber.
  rs2::config cfg;
  cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGBA8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
  rs.open(cfg);

  // Configuring and opening Structure Core grabber.
  ST::CaptureSessionSettings settings;
  settings.source = ST::CaptureSessionSourceId::StructureCore;
  settings.structureCore.visibleEnabled = true;
  settings.applyExpensiveCorrection = true; // Apply a correction and clean filter to the depth before streaming.
  sc.open(settings);

  // Acquiring images.
  for (;;) {
    rs.acquire(reinterpret_cast<unsigned char *>(I_color_rs.bitmap),
               reinterpret_cast<unsigned char *>(rs_I_depth_raw.bitmap), nullptr, nullptr, nullptr, nullptr, nullptr);
    sc.acquire(reinterpret_cast<unsigned char *>(I_color_sc.bitmap),
               reinterpret_cast<unsigned char *>(sc_I_depth_raw.bitmap));

    // Converting raw depth data to color images.
    vpImageConvert::createDepthHistogram(rs_I_depth_raw, I_depth_rs);
    vpImageConvert::createDepthHistogram(sc_I_depth_raw, I_depth_sc);

    vpDisplay::display(I_color_rs);
    vpDisplay::display(I_color_sc);
    vpDisplay::display(I_depth_rs);
    vpDisplay::display(I_depth_sc);

    vpDisplay::flush(I_color_rs);
    vpDisplay::flush(I_color_sc);
    vpDisplay::flush(I_depth_rs);
    vpDisplay::flush(I_depth_sc);

    if (vpDisplay::getClick(I_color_rs, false) || vpDisplay::getClick(I_color_sc, false) ||
        vpDisplay::getClick(I_depth_rs, false) || vpDisplay::getClick(I_depth_sc, false)) {
      break;
}
}
#else
  (void)argc;
  (void)argv;
#if !(defined(VISP_HAVE_OCCIPITAL_STRUCTURE))
  std::cout << "Install libStructure, configure and build ViSP again to use this example" << std::endl;
#endif
#if !(defined(VISP_HAVE_REALSENSE2))
  std::cout << "Install librealsense, configure and build ViSP again to use this example" << std::endl;
#endif
#if (VISP_CXX_STANDARD < VISP_CXX_STANDARD_11)
  std::cout << "This tutorial should be built with c++11 support" << std::endl;
#endif
#endif
}
