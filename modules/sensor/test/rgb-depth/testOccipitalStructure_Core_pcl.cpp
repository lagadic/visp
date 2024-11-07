/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Point cloud depth visualization with Occipital Structure Core sensor.
 */

/*!
  \example testOccipitalStructure_Core_pcl.cpp
  This example visualize point cloud from a Occipital Structure Core sensor
  with libStructure. If a color sensor is used, RGB pointcloud is visualized.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_OCCIPITAL_STRUCTURE) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11) && (defined(VISP_HAVE_PCL)) && defined(VISP_HAVE_PCL_VISUALIZATION)

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpOccipitalStructure.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  try {
    unsigned int display_scale = 1;
    vpOccipitalStructure sc;

    ST::CaptureSessionSettings settings;
    settings.source = ST::CaptureSessionSourceId::StructureCore;
    settings.structureCore.visibleEnabled = true;
    settings.applyExpensiveCorrection = true; // Apply a correction and clean filter to the depth before streaming.

    sc.open(settings);

    // Calling these 2 functions to set internal variables.
    sc.getCameraParameters(vpOccipitalStructure::visible);
    sc.getCameraParameters(vpOccipitalStructure::depth);

#if defined(VISP_HAVE_X11)
    vpDisplayX display_visible; // Visible image
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI display_visible; // Visible image
#endif
    vpImage<vpRGBa> I_visible(sc.getHeight(vpOccipitalStructure::visible), sc.getWidth(vpOccipitalStructure::visible),
                              0);
    ;
    display_visible.setDownScalingFactor(display_scale);
    display_visible.init(I_visible, static_cast<int>(I_visible.getWidth() / display_scale) + 80, 10, "Color image");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    sc.acquire((unsigned char *)I_visible.bitmap, nullptr, nullptr, pointcloud);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud);

    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0, 0, -0.5, 0, -1, 0);

    while (true) {
      double t = vpTime::measureTimeMs();

      // Acquire depth as point cloud.
      sc.acquire((unsigned char *)I_visible.bitmap, nullptr, nullptr, pointcloud);
      vpDisplay::display(I_visible);
      vpDisplay::displayText(I_visible, 15 * display_scale, 15 * display_scale, "Click to quit", vpColor::red);
      vpDisplay::flush(I_visible);

      if (vpDisplay::getClick(I_visible, false))
        break;

      static bool update = false;
      if (!update) {
        viewer->addPointCloud<pcl::PointXYZRGB>(pointcloud, rgb, "sample cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
        update = true;
      }
      else {
        viewer->updatePointCloud<pcl::PointXYZRGB>(pointcloud, rgb, "sample cloud");
      }

      viewer->spinOnce(30);

      std::cout << "Loop time: " << vpTime::measureTimeMs() - t << std::endl;
    }
  }
  catch (const vpException &e) {
    std::cerr << "Structure SDK error " << e.what() << std::endl;
  }
  catch (const std::exception &e) {
    std::cerr << e.what() << std::endl;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined( VISP_HAVE_OCCIPITAL_STRUCTURE )
  std::cout << "You do not have Occipital Structure SDK functionality enabled..." << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Install libStructure, configure again ViSP using cmake and build again this example" << std::endl;
  return EXIT_SUCCESS;
#elif ( VISP_CXX_STANDARD < VISP_CXX_STANDARD_11 )
  std::cout << "You do not build ViSP with c++11 or higher compiler flag" << std::endl;
  std::cout << "Tip:" << std::endl;
  std::cout << "- Configure ViSP again using cmake -DUSE_CXX_STANDARD=11, and build again this example" << std::endl;
#elif !defined( VISP_HAVE_PCL )
  std::cout << "You do not have PCL 3rd party installed." << std::endl;
#elif !defined( VISP_HAVE_PCL_VISUALIZATION )
  std::cout << "You do not have PCL visualization module." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
