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

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION) && defined(VISP_HAVE_THREADS)

#include <visp3/gui/vpDisplayPCL.h>

BEGIN_VISP_NAMESPACE
/*!
 * Default constructor.
 * By default, viewer size is set to 640 x 480.
 */
vpDisplayPCL::vpDisplayPCL(int posx, int posy, const std::string &window_name)
  : m_stop(false), m_verbose(false), m_width(640), m_height(480), m_posx(posx), m_posy(posy),
  m_window_name(window_name), m_viewer(nullptr)
{ }

/*!
 * Constructor able to initialize the display window size.
 * \param[in] width : Point cloud viewer width.
 * \param[in] height : Point cloud viewer height.
 * \param[in] posx : Position along X-axis.
 * \param[in] posy : Position along Y-axis.
 * \param[in] window_name : Window name.
 */
vpDisplayPCL::vpDisplayPCL(unsigned int width, unsigned int height, int posx, int posy, const std::string &window_name)
  : m_stop(false), m_verbose(false), m_width(width), m_height(height), m_posx(posx), m_posy(posy),
  m_window_name(window_name), m_viewer(nullptr)
{ }

/*!
 * Destructor that stops and join the viewer thread if not already done.
 *
 * \sa stop(), startThread()
 */
vpDisplayPCL::~vpDisplayPCL()
{
  stop();
}

/*!
 * Loop that does the display of the point cloud.
 * @param[inout] pointcloud_mutex : Shared mutex to protect from concurrent access to `pointcloud` object.
 * @param[in] pointcloud : Point cloud to display.
 */
void vpDisplayPCL::run(std::mutex &pointcloud_mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
  m_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer());
  m_viewer->setBackgroundColor(0, 0, 0);
  m_viewer->initCameraParameters();
  m_viewer->setPosition(m_posx, m_posy);
  m_viewer->setCameraPosition(0, 0, -0.25, 0, -1, 0);
  m_viewer->setSize(m_width, m_height);
  m_viewer->setWindowName(m_window_name);
  bool init = true;

  while (!m_stop) {
    {
      std::lock_guard<std::mutex> lock(pointcloud_mutex);
      local_pointcloud = pointcloud->makeShared();
    }

    if (init) {
      m_viewer->addPointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud");
      m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

      init = false;
    }
    else {
      m_viewer->updatePointCloud<pcl::PointXYZ>(local_pointcloud, "sample cloud");
    }

    m_viewer->spinOnce(10);
  }

  if (m_verbose) {
    std::cout << "End of point cloud display thread" << std::endl;
  }
}

/*!
 * Loop that does the display of the textured point cloud.
 * @param[inout] mutex : Shared mutex.
 * @param[in] pointcloud : Textured point cloud to display.
 */
void vpDisplayPCL::run_color(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_color)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pointcloud_color);
  m_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer());
  m_viewer->setBackgroundColor(0, 0, 0);
  m_viewer->initCameraParameters();
  m_viewer->setPosition(m_posx, m_posy);
  m_viewer->setCameraPosition(0, 0, -0.25, 0, -1, 0);
  m_viewer->setSize(m_width, m_height);
  m_viewer->setWindowName(m_window_name);
  bool init = true;

  while (!m_stop) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      local_pointcloud = pointcloud_color->makeShared();
    }

    if (init) {
      m_viewer->addPointCloud<pcl::PointXYZRGB>(local_pointcloud, rgb, "RGB sample cloud");
      m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "RGB sample cloud");

      init = false;
    }
    else {
      m_viewer->updatePointCloud<pcl::PointXYZRGB>(local_pointcloud, rgb, "RGB sample cloud");
    }

    m_viewer->spinOnce(10);
  }

  if (m_verbose) {
    std::cout << "End of point cloud display thread" << std::endl;
  }
}

/*!
 * Start the viewer thread able to display a point cloud.
 * @param[inout] mutex : Shared mutex.
 * @param[in] pointcloud : Point cloud to display.
 *
 * \sa stop()
 */
void vpDisplayPCL::startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud)
{
  m_thread = std::thread(&vpDisplayPCL::run, this, std::ref(mutex), pointcloud);
}

/*!
 * Start the viewer thread able to display a textured point cloud.
 * @param[inout] mutex : Shared mutex.
 * @param[in] pointcloud_color : Textured point cloud to display.
 *
 * \sa stop()
 */
void vpDisplayPCL::startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_color)
{
  m_thread = std::thread(&vpDisplayPCL::run_color, this, std::ref(mutex), pointcloud_color);
}

/*!
 * Set the position of the viewer window.
 * This function has to be called prior startThread().
 *
 * @param[in] posx : Position along x.
 * @param[in] posy : Position along y.
 *
 * \sa setWindowName()
 */
void vpDisplayPCL::setPosition(int posx, int posy)
{
  m_posx = posx;
  m_posy = posy;
}

/*!
 * Set the name of the viewer windows.
 * This function has to be called prior startThread().
 *
 * @param[in] window_name : Window name to set.
 *
 * \sa setPosition()
 */
void vpDisplayPCL::setWindowName(const std::string &window_name)
{
  m_window_name = window_name;
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
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_gui.a(vpDisplayPCL.cpp.o) has no symbols
void dummy_vpDisplayPCL() { };

#endif
