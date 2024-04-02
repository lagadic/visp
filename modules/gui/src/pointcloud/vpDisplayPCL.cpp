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

/*!
 * Default constructor.
 * By default, viewer size is set to 640 x 480.
 */
vpDisplayPCL::vpDisplayPCL() : m_stop(false), m_verbose(false), m_width(640), m_height(480) { }

/*!
 * Constructor able to initialize the display window size.
 */
vpDisplayPCL::vpDisplayPCL(unsigned int width, unsigned int height) : m_stop(false), m_verbose(false), m_width(width), m_height(height) { }

/*!
 * Destructor that stops and join the viewer thread if not already done.
 */
vpDisplayPCL::~vpDisplayPCL()
{
  stop();
}

/*!
 * Loop that does the display of the point cloud.
 * @param[inout] mutex : Shared mutex.
 * @param[in] pointcloud : Point cloud to display.
 */
void vpDisplayPCL::run(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setPosition(m_width + 80, m_height + 80);
  viewer->setCameraPosition(0, 0, -0.25, 0, -1, 0);
  viewer->setSize(m_width, m_height);

  while (!m_stop) {
    {
      std::lock_guard<std::mutex> lock(mutex);
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

/*!
 * Start the viewer thread able to display a point cloud.
 * @param[inout] mutex : Shared mutex.
 * @param[in] pointcloud : Point cloud to display.
 */
void vpDisplayPCL::startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  m_thread = std::thread(&vpDisplayPCL::run, this, std::ref(mutex), pointcloud);
}

/*!
 * Stop the viewer thread and join.
 */
void vpDisplayPCL::stop()
{
  if (!m_stop) {
    m_stop = true;

    if (m_thread.joinable()) {
      m_thread.join();
    }
  }
}

/*!
 * Enable/disable verbose mode.
 * @param[in] verbose : When true verbose mode is enable.
 */
void vpDisplayPCL::setVerbose(bool verbose)
{
  m_verbose = verbose;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_gui.a(vpDisplayPCL.cpp.o) has no symbols
void dummy_vpDisplayPCL() { };

#endif
