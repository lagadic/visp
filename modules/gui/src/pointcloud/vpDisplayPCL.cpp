/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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

unsigned int vpDisplayPCL::PointCloudHandling::s_nb = 0;

/*!
 * Default constructor.
 * By default, viewer size is set to 640 x 480.
 */
vpDisplayPCL::vpDisplayPCL(int posx, int posy, const std::string &window_name)
  : m_stop(false), m_thread_running(false), m_verbose(false), m_width(640), m_height(480), m_posx(posx), m_posy(posy),
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
  : m_stop(false), m_thread_running(false), m_verbose(false), m_width(width), m_height(height), m_posx(posx), m_posy(posy),
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

/**
 * \brief Create the viewer.
 */
void vpDisplayPCL::createViewer()
{
  m_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(m_window_name));
  m_viewer->setBackgroundColor(0, 0, 0);
  m_viewer->initCameraParameters();
  m_viewer->setPosition(m_posx, m_posy);
  m_viewer->setCameraPosition(0, 0, -0.25, 0, -1, 0);
  m_viewer->setSize(m_width, m_height);
}

/**
 * \brief Insert a legend in the viewer for non-textured point-clouds.
 *
 * \param[in] id The ID of the point-cloud for which we need to add a legend.
 */
void vpDisplayPCL::insertLegend(const size_t &id)
{
  std::string text = this->mv_xyz_pcl[id].second.m_name;
  unsigned int posU = 10;
  unsigned int size = 16;
  unsigned int posV = 10 + id * size;
  double rRatio = this->mv_xyz_pcl[id].second.m_color.R / 255.0;
  double gRatio = this->mv_xyz_pcl[id].second.m_color.G / 255.0;
  double bRatio = this->mv_xyz_pcl[id].second.m_color.B / 255.0;

  this->m_viewer->addText(text, posU, posV, rRatio, gRatio, bRatio);
};

/**
 * \brief Monothread display method. MacOS currently can only use this monothread
 * method, otherwise they get the error `uncaught exception 'NSInternalInconsistencyException', reason: 'NSWindow should only be instantiated on the main thread!'`
 * \warning Because pcl::visualization::PCLVisualizer is not multi-thread friendly,
 * calling this method stops the display thread if it was running.
 */
void vpDisplayPCL::display(const bool &blocking)
{
  if (m_thread_running) {
    stop();

    // Reset the fact that the point-clouds must be first inserted before being updated
    // as the viewer will be an entirely new one
    size_t nb_pcls = mv_xyz_pcl.size();
    for (size_t id = 0; id < nb_pcls; ++id) {
      mv_xyz_pcl[id].second.m_do_init = true;
    }

    nb_pcls = mv_colored_pcl.size();
    for (size_t id = 0; id < nb_pcls; ++id) {
      mv_colored_pcl[id].second.m_do_init = true;
    }
  }

  if (!m_viewer) {
    createViewer();
  }

  size_t nb_pcls = mv_xyz_pcl.size();
  for (size_t id = 0; id < nb_pcls; ++id) {
    if (mv_xyz_pcl[id].second.m_do_init) {
      m_viewer->addPointCloud<pcl::PointXYZ>(mv_xyz_pcl[id].second.m_pcl, mv_xyz_handlers[id], mv_xyz_pcl[id].second.m_name);
      m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, mv_xyz_pcl[id].second.m_name);
      insertLegend(id);
      mv_xyz_pcl[id].second.m_do_init = false;
    }
    else {
      m_viewer->updatePointCloud<pcl::PointXYZ>(mv_xyz_pcl[id].second.m_pcl, mv_xyz_handlers[id], mv_xyz_pcl[id].second.m_name);
    }
  }

  nb_pcls = mv_colored_pcl.size();
  for (size_t id = 0; id < nb_pcls; ++id) {
    if (mv_colored_pcl[id].second.m_do_init) {
      m_viewer->addPointCloud<pcl::PointXYZRGB>(mv_colored_pcl[id].second.m_pcl, mv_color_handlers[id], mv_colored_pcl[id].second.m_name);
      m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, mv_colored_pcl[id].second.m_name);

      mv_colored_pcl[id].second.m_do_init = false;
    }
    else {
      m_viewer->updatePointCloud<pcl::PointXYZRGB>(mv_colored_pcl[id].second.m_pcl, mv_color_handlers[id], mv_colored_pcl[id].second.m_name);
    }
  }

  if (blocking) {
    m_viewer->spin();
  }
  else {
    m_viewer->spinOnce(10);
  }
}

/*!
 * Loop that does the display of the point cloud.
 */
void vpDisplayPCL::run()
{
  // Creating the window in this method for 2 reasons:
  // 1) It is the same code whichever startThread method is called
  // 2) According to the pcl documentation, a PCLVisualizer `can NOT be used across multiple threads. Only call functions of objects of this class from the same thread that they were created in!`
  if (m_viewer) {
    // m_viewer->close();
    m_viewer.reset();

    // Reset the fact that the point-clouds must be first inserted before being updated
    // as the viewer will be an entirely new one
    size_t nb_pcls;
    {
      std::lock_guard<std::mutex> lock(m_mutex_vector);
      nb_pcls = mv_xyz_pcl.size();
    }
    for (size_t id = 0; id < nb_pcls; ++id) {
      std::lock_guard<std::mutex> lock(mv_xyz_pcl[id].first);
      mv_xyz_pcl[id].second.m_do_init = true;
    }

    {
      std::lock_guard<std::mutex> lock(m_mutex_vector);
      nb_pcls = mv_colored_pcl.size();
    }

    for (size_t id = 0; id < nb_pcls; ++id) {
      std::lock_guard<std::mutex> lock(mv_colored_pcl[id].first);
      mv_colored_pcl[id].second.m_do_init = true;
    }
  }

  createViewer();
  pcl::PointCloud<pcl::PointXYZ>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());

  size_t nb_pcls;
  while (!m_stop) {
    {
      std::lock_guard<std::mutex> lock(m_mutex_vector);
      nb_pcls = mv_xyz_pcl.size();
    }
    for (size_t id = 0; id < nb_pcls; ++id) {
      {
        std::lock_guard<std::mutex> lock(mv_xyz_pcl[id].first);
        local_pointcloud = mv_xyz_pcl[id].second.m_pcl->makeShared();
      }

      if (mv_xyz_pcl[id].second.m_do_init) {
        m_viewer->addPointCloud<pcl::PointXYZ>(local_pointcloud, mv_xyz_handlers[id], mv_xyz_pcl[id].second.m_name);
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, mv_xyz_pcl[id].second.m_name);
        insertLegend(id);
        mv_xyz_pcl[id].second.m_do_init = false;
      }
      else {
        m_viewer->updatePointCloud<pcl::PointXYZ>(local_pointcloud, mv_xyz_handlers[id], mv_xyz_pcl[id].second.m_name);
      }
    }

    m_viewer->spinOnce(10);
  }

  if (m_verbose) {
    std::cout << "End of point cloud display thread" << std::endl;
  }

  // m_viewer->close();
  m_viewer.reset(); // Mandatory because a viewer can only be used in the thread it was created in
}

/*!
 * Loop that does the display of the textured point clouds.
 */
void vpDisplayPCL::runColor()
{
  // Creating the window in this method for 2 reasons:
  // 1) It is the same code for the two "run" methods
  // 2) To avoid MacOS error "NSInternalInconsistencyException', reason: 'NSWindow should only be instantiated on the main thread!'"
  if (m_viewer) {
    // m_viewer->close();
    m_viewer.reset();

    // Reset the fact that the point-clouds must be first inserted before being updated
    // as the viewer will be an entirely new one
    size_t nb_pcls;
    {
      std::lock_guard<std::mutex> lock(m_mutex_vector);
      nb_pcls = mv_xyz_pcl.size();
    }
    for (size_t id = 0; id < nb_pcls; ++id) {
      std::lock_guard<std::mutex> lock(mv_xyz_pcl[id].first);
      mv_xyz_pcl[id].second.m_do_init = true;
    }

    {
      std::lock_guard<std::mutex> lock(m_mutex_vector);
      nb_pcls = mv_colored_pcl.size();
    }

    for (size_t id = 0; id < nb_pcls; ++id) {
      std::lock_guard<std::mutex> lock(mv_colored_pcl[id].first);
      mv_colored_pcl[id].second.m_do_init = true;
    }
  }
  createViewer();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr local_pointcloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  size_t nb_pcls;
  while (!m_stop) {
    {
      std::lock_guard<std::mutex> lock(m_mutex_vector);
      nb_pcls = mv_colored_pcl.size();
    }
    for (size_t id = 0; id < nb_pcls; ++id) {
      {
        std::lock_guard<std::mutex> lock(mv_colored_pcl[id].first);
        local_pointcloud = mv_colored_pcl[id].second.m_pcl->makeShared();
      }

      if (mv_colored_pcl[id].second.m_do_init) {
        std::lock_guard<std::mutex> lock(mv_colored_pcl[id].first);
        m_viewer->addPointCloud<pcl::PointXYZRGB>(local_pointcloud, mv_color_handlers[id], mv_colored_pcl[id].second.m_name);
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, mv_colored_pcl[id].second.m_name);

        mv_colored_pcl[id].second.m_do_init = false;
      }
      else {
        std::lock_guard<std::mutex> lock(mv_colored_pcl[id].first);
        m_viewer->updatePointCloud<pcl::PointXYZRGB>(local_pointcloud, mv_color_handlers[id], mv_colored_pcl[id].second.m_name);
      }
    }

    m_viewer->spinOnce(10);
  }

  if (m_verbose) {
    std::cout << "End of point cloud display thread" << std::endl;
  }

  // m_viewer->close();
  m_viewer.reset(); // Mandatory because a viewer can only be used in the thread it was created in
}

/*!
 * Start the viewer thread able to display a point cloud.
 */
void vpDisplayPCL::startThread(const bool &colorThread)
{
  if (!m_thread_running) {
    m_stop = false;
    if (colorThread) {
      m_thread = std::thread(&vpDisplayPCL::runColor, this);
    }
    else {
      m_thread = std::thread(&vpDisplayPCL::run, this);
    }
    m_thread_running = true;
  }
  else {
    throw(vpException(vpException::fatalError, "A viewer thread is already running."));
  }
}

/*!
 * Start the viewer thread able to display a point cloud.
 * @param[inout] mutex : Shared mutex.
 * @param[in] pointcloud : Point cloud to display.
 * \sa stop()
 */
void vpDisplayPCL::startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const std::string &name, const vpColor &color)
{
  if (!m_thread_running) {
    m_stop = false;
    mv_xyz_pcl.emplace_back(std::pair<std::mutex &, XYZPointCloudHandling>(std::ref(mutex), XYZPointCloudHandling(pointcloud, name, color)));
    mv_xyz_handlers.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pointcloud, color.R, color.G, color.B));
    m_thread = std::thread(&vpDisplayPCL::run, this);
    m_thread_running = true;
  }
  else {
    throw(vpException(vpException::fatalError, "A viewer thread is already running."));
  }
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
  if (!m_thread_running) {
    m_stop = false;
    mv_colored_pcl.emplace_back(std::pair<std::mutex &, ColoredPointCloudHandling>(std::ref(mutex), ColoredPointCloudHandling(pointcloud_color)));
    mv_color_handlers.push_back(pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(pointcloud_color));
    m_thread = std::thread(&vpDisplayPCL::runColor, this);
    m_thread_running = true;
  }
  else {
    throw(vpException(vpException::fatalError, "A viewer thread is already running."));
  }
}

/*!
 * @brief Insert a point cloud to display.
 *
 * @param[in] mutex Mutex that protects the point-cloud.
 * @param[in] pointcloud The point-cloud to displaY
 */
void vpDisplayPCL::addPointCloud(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const std::string &name, const vpColor &color)
{
  std::lock_guard lg(m_mutex_vector);
  mv_xyz_pcl.emplace_back(std::pair<std::mutex &, XYZPointCloudHandling>(std::ref(mutex), XYZPointCloudHandling(pointcloud, name, color)));
  mv_xyz_handlers.push_back(pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(pointcloud, color.R, color.G, color.B));
}

/*!
 * @brief Insert a colored point cloud to display.
 *
 * @param[in] mutex Mutex that protects the point-cloud.
 * @param[in] pointcloud The point-cloud to display.
 */
void vpDisplayPCL::addPointCloud(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud)
{
  std::lock_guard lg(m_mutex_vector);
  mv_colored_pcl.emplace_back(std::pair<std::mutex &, ColoredPointCloudHandling>(std::ref(mutex), ColoredPointCloudHandling(pointcloud)));
  mv_color_handlers.push_back(pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(pointcloud));
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
  if (m_thread_running) {
    if (!m_stop) {
      m_stop = true;

      if (m_thread.joinable()) {
        m_thread.join();
      }
    }
    m_thread_running = false;
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
void dummy_vpDisplayPCL() { }

#endif
