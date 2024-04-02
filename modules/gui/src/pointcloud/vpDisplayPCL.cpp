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
 * Display a point cloud using PCL library.
 */

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_THREADS)

#include <visp3/gui/vpDisplayPCL.h>

vpDisplayPCL::vpDisplayPCL() : m_stop(false), m_flush_viewer(false), m_verbose(false) { }

void vpDisplayPCL::flush()
{
  m_flush_viewer = true;
}

void vpDisplayPCL::run(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

  bool flush_viewer = false;
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setPosition(640 + 80, 480 + 80);
  viewer->setCameraPosition(0, 0, -0.25, 0, -1, 0);
  viewer->setSize(640, 480);

  while (!m_stop) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      flush_viewer = m_flush_viewer;
      m_flush_viewer = false;
      local_pointcloud = pointcloud->makeShared();

    }

    // If updatePointCloud fails, it means that the pcl was not previously known by the viewer
    if (!viewer->updatePointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud")) {
      // Add the pcl to the list of pcl known by the viewer + the according legend
      viewer->addPointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud");
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    }

    viewer->spinOnce(10);
  }

  if (m_verbose) {
    std::cout << "End of point cloud display thread" << std::endl;
  }
}

void vpDisplayPCL::stop()
{
  m_stop = true;
}

void vpDisplayPCL::setVerbose(bool verbose)
{
  m_verbose = verbose;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_gui.a(vpDisplayPCL.cpp.o) has no symbols
void dummy_vpDisplayPCL() { };

#endif
