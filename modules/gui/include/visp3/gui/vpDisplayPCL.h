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

#ifndef _vpDisplayPCL_h_
#define _vpDisplayPCL_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION) && defined(VISP_HAVE_THREADS)

#include <mutex>
#include <thread>
#include <string>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpDisplayPCL
  \ingroup group_gui_plotter
  This class enables real time plotting of 3D point clouds. It relies on the PCL library.
  To see how to install PCL library, please refer to the \ref soft_tool_pcl section.
*/
class VISP_EXPORT vpDisplayPCL
{
public:
  vpDisplayPCL(int posx = 0, int posy = 0, const std::string &window_name = "");
  vpDisplayPCL(unsigned int width, unsigned int height, int posx = 0, int posy = 0, const std::string &window_name = "");
  ~vpDisplayPCL();

  void setVerbose(bool verbose);
  void startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
  void startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);
  void setPosition(int posx, int posy);
  void setWindowName(const std::string &window_name);
  void stop();

private:
  void run(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud);
  void run_color(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);

  bool m_stop;
  bool m_verbose;
  std::thread m_thread; //!< Non-blocking display thread.
  std::mutex m_mutex;
  unsigned int m_width;
  unsigned int m_height;
  int m_posx;
  int m_posy;
  std::string m_window_name;
  pcl::visualization::PCLVisualizer::Ptr m_viewer;
};
END_VISP_NAMESPACE
#endif

#endif
