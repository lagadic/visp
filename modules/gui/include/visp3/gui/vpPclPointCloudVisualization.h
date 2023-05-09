/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * 
 *
 * Description:
 * Real-time 3D point clouds plotter based on the PCL library.
 *
 * Authors:
 * Romain LAGNEAU
 *
 *****************************************************************************/

#ifndef POINTCLOUDVISUALIZATION_H
#define POINTCLOUDVISUALIZATION_H

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL)
// System
#include <thread>
#include <mutex>

// ViSP
#include <visp3/core/vpColVector.h>

// PCL
#include <pcl/visualization/pcl_visualizer.h>

class VISP_EXPORT vpPclPointCloudVisualization
{
public:
  typedef typename pcl::PointXYZRGB pclPoint;
  typedef typename pcl::PointCloud<pclPoint> pclPointCloud;
  typedef typename pclPointCloud::Ptr pclPointCloudPtr;

  typedef struct legendParams{
    std::string _text;
    unsigned int _posX;
    unsigned int _posY;
    unsigned int _size;
    double _rRatio;
    double _gRatio;
    double _bRatio;
  }legendParams;

  vpPclPointCloudVisualization(const std::string &title , const int &width = 640, const int &height = 480, const int &px = 720, const int &py = 560, const std::string &outFolder = std::string(), const double &ignoreThreshold = 0.95);
  ~vpPclPointCloudVisualization();

  void set_nameWindow(const std::string &nameWindow);

  void set_outFolder(const std::string &outputFolder);

  void set_ignoreThreshold(const double &thresh);

  unsigned int addSurface(const pclPointCloud::Ptr &surface, const std::string &name = "", const std::vector<unsigned char> &v_color = std::vector<unsigned char>());

  unsigned int addSurface(const pclPointCloud::Ptr &surface, const vpColVector &weights, const std::string &name = "", const std::vector<unsigned char> &v_color = std::vector<unsigned char>());

  void updateSurface(const pclPointCloud::Ptr &surface, unsigned int id);

  void updateSurface(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector& weights);

  void display();

  void refresh();

  void launchThread();

  void stopThread();

  static void runThread(vpPclPointCloudVisualization* p_visualizer);

  static void copyLegendParams(const legendParams& from, legendParams& to);

  void loopThread();

  void threadUpdateSurface(const pclPointCloud::Ptr &surface, unsigned int id);

  void threadUpdateSurfaceOriginalColor(const pclPointCloud::Ptr &surface, unsigned int id);

  void threadUpdateSurface(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector& weights);

  void threadUpdateSurfaceOriginalColor(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector& weights);

protected:
  std::vector<pclPointCloud::Ptr> _vPointClouds;
  static pcl::visualization::PCLVisualizer::Ptr gp_viewer;
  static std::vector<std::vector<double>> g_vhandler;
  static int s_width;
  static int s_height;
  static int s_px;
  static int s_py;
  static double s_ignoreThresh;
  std::vector<std::string> _vmeshid;
  std::vector<legendParams> _vlegends;
  std::vector<std::mutex*> _vpmutex;
  vpColVector _weights;
  std::mutex _mutexWeights;
  std::thread _threadDisplay;
  bool _hasToRun;
  std::string _title;
  bool _hasToSavePCDs;
  std::string _outFolder;
};
#endif // #if defined(VISP_HAVE_PCL)
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
#endif // PCLVISUALIZATION_H
