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
 * Description:
 * Real-time 3D point clouds plotter based on the PCL library.
 *
*****************************************************************************/

#ifndef _vpPclViewer_h_
#define _vpPclViewer_h_

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION) && defined(VISP_HAVE_THREADS)
// System
#include <thread>
#include <mutex>

// ViSP
#include <visp3/core/vpColVector.h>

// PCL
#include <pcl/visualization/pcl_visualizer.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpPclViewer
  \ingroup group_gui_plotter
  This class enables real time plotting of 3D point clouds. It relies on the PCL library. To see how to install PCL library,
  please refer to the \ref soft_tool_pcl section.
  You can either plot in a non-blocking threaded manner or in a blocking manner.
  The point clouds can be displayed using their original colors, or using uniform colors to match
  a legend.
  Additionally, it is possible to use confidence weights to hide points that are not trusted. These
  weights can result from a robust estimation using for instance the vpRobust class.

  \warning If you face some runtime errors with the following class, please refer to the \ref pcl_viewer_known_issues
  section to see if it is a known problem.

  \sa \ref tutorial-pcl-viewer
*/
class VISP_EXPORT vpPclViewer
{
public:
  typedef typename pcl::PointXYZRGB pclPointXYZRGB;
  typedef typename pcl::PointCloud<pclPointXYZRGB> pclPointCloudPointXYZRGB;
  typedef typename pclPointCloudPointXYZRGB::Ptr pclPointCloudPointXYZRGBPtr;

  /**
   * @brief Structure that contains all the required parameters to display a legend on the viewer.
   */
  typedef struct legendParams
  {
    std::string m_text; /*!< The text of the legend.*/
    unsigned int m_posU; /*!< The position of the legend on the u-axis of the screen.*/
    unsigned int m_posV; /*!< The position of the legend on the v-axis of the screen.*/
    unsigned int m_size; /*!< The size of the legend.*/
    double m_rRatio; /*!< The red ratio of the legend.*/
    double m_gRatio; /*!< The green ratio of the legend.*/
    double m_bRatio; /*!< The blue ratio of the legend.*/
  }legendParams;

  /**
   * @brief Construct a new vpPclViewer object.
   *
   * @param title The title of the window of the viewer.
   * @param width The width of the window of the viewer.
   * @param height The height of the window of the viewer.
   * @param posU The position on the horizontal axis of the screen of the window of the viewer.
   * @param posV The position on the vertical axis of the screen of the window of the viewer.
   * @param outFolder If different from the empty string, the point clouds will be saved in this folder.
   * @param ignoreThreshold A point for which the weight is below this threshold will be displayed in black.
   */
  vpPclViewer(const std::string &title, const int &width = 640, const int &height = 480, const int &posU = 720, const int &posV = 560, const std::string &outFolder = std::string(), const double &ignoreThreshold = 0.95);
  ~vpPclViewer();

  /**
   * @brief Set the name of the PCL viewer window.
   *
   * @param nameWindow The name of the PCL viewer window.
   */
  void setNameWindow(const std::string &nameWindow);

  /**
   * @brief Set the path to the output folder.
   * If different from the empty string, the point clouds will be saved in this folder.
   * @param outputFolder The path towards to the output folder, or empty if the point clouds must not be saved.
   */
  void setOutFolder(const std::string &outputFolder);

  /**
   * @brief Set the threshold below which a point must be displayed in black.
   *
   * @param thresh The threshold below which a point must be displayed in black.
   */
  void setIgnoreThreshold(const double &thresh);

  /**
   * @brief Add a surface to the list of point clouds known by the viewer.
   *
   * @param surface The surface that must be knwon to be displayed by the PCL viewer.
   * @param name The name of the surface that will be displayed in the legend. If empty, it will be automatically generated.
   * @param v_color A vector containing the 3 RGB values.
   * @return unsigned int The ID by which is known the point cloud by the viewer.
   */
  unsigned int addSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const std::string &name = "", const std::vector<unsigned char> &v_color = std::vector<unsigned char>());

  /**
   * @brief Add a surface to the list of point clouds known by the viewer.
   * The points whose weights are below the \b vpPclViewer::s_ignoreThresh wil be displayed in black.
   *
   * @param surface The surface that must be knwon to be displayed by the PCL viewer.
   * @param weights The confidence weights of each points. Must be between 0 and 1.
   * @param name The name of the surface that will be displayed in the legend. If empty, it will be automatically generated.
   * @param v_color A vector containing the 3 RGB values.
   * @return unsigned int The ID by which is known the point cloud by the viewer.
   */
  unsigned int addSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const vpColVector &weights, const std::string &name = "", const std::vector<unsigned char> &v_color = std::vector<unsigned char>());

  /**
   * @brief Update the surface known by \b id by the viewer.
   *
   * @param surface The updated surface.
   * @param id The ID of the surface that must be updated.
   * @param hasToKeepColor If true, will be displayed in its original color. Otherwise, will be displayed in its default color.
   */
  void updateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id, const bool &hasToKeepColor = false);

  /**
   * @brief Update the surface known by \b id by the viewer.
   *
   * @param surface The updated surface.
   * @param id The ID of the surface that must be updated.
   * @param weights The confidence weights of each points.
   * @param hasToKeepColor If true, will be displayed in its original color. Otherwise, will be displayed in its default color.
   */
  void updateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id, const vpColVector &weights, const bool &hasToKeepColor = false);

  /**
   * @brief Blocking-mode display of the viewer.
   */
  void display();

  /**
   * @brief Refresh the display.
   *
   * @param timeout Duration allowed for the drawing.
   * @param waitForDrawing If true, will wait until the drawing is done. Otherwise, might return without doing nothing.
   */
  void refresh(const int &timeout = 100, const bool &waitForDrawing = true);

  /**
   * @brief Start the drawing thread that permits to have a non-blocking display.
   */
  void launchThread();

  /**
   * @brief Stop the drawing thread that permits to have a non-blocking display.
   */
  void stopThread();

protected:
  /**
   * @brief Method to update a point cloud known by the viewer when the drawing thread is running.
   * The updated surface will be drawn with the default color that was affected to it.
   *
   * @param surface The updated surface.
   * @param id The ID of the point cloud that must be updated.
   */
  void threadUpdateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id);

  /**
   * @brief Method to update a point cloud known by the viewer when the drawing thread is running.
   * The updated surface will be drawn with the color it contains.
   *
   * @param surface The updated surface.
   * @param id The ID of the point cloud that must be updated.
   */
  void threadUpdateSurfaceOriginalColor(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id);

  /**
   * @brief Method to update a point cloud known by the viewer when the drawing thread is running.
   * The updated surface will be drawn with the default color that was affected to it.
   *
   * @param surface The updated surface.
   * @param id The ID of the point cloud that must be updated.
   * @param weights The confidence weights of each point. Must be between 0 and 1.
   */
  void threadUpdateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id, const vpColVector &weights);

  /**
   * @brief Method to update a point cloud known by the viewer when the drawing thread is running.
   * The updated surface will be drawn with the color it contains.
   *
   * @param surface The updated surface.
   * @param id The ID of the point cloud that must be updated.
   * @param weights The confidence weights of each point. Must be between 0 and 1.
   */
  void threadUpdateSurfaceOriginalColor(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id, const vpColVector &weights);


  /**
   * @brief Internal method that is called by \b vpPclViewer::launchThread to launch the
   * drawing thread.
   *
   * @param p_viewer The pointer of the \b vpPclViewer object that will run the thread (is equal to \b this ).
   */
  static void runThread(vpPclViewer *p_viewer);

  /**
   * @brief The internal loop of the non-blocking drawing thread.
   *
   */
  void loopThread();

  std::vector<pclPointCloudPointXYZRGB::Ptr> m_vPointClouds; /*!< The list of point clouds known by the viewer.*/
  static pcl::visualization::PCLVisualizer::Ptr sp_viewer; /*!< The PCL viewer permitting the display.*/
  static std::vector<std::vector<double>> s_vhandler; /*!< The list of color handlers.*/
  static int s_width; /*!< The width of the window.*/
  static int s_height; /*!< The height of the window.*/
  static int s_posU; /*!< The position along the horizontal axis of the screen of the window.*/
  static int s_posV; /*!< The position along the vertical axis of the screen of the window.*/
  static double s_ignoreThresh; /*!< The minimum value of the confidence weight of a point to allow it to be displayed.*/
  std::vector<std::string> m_vmeshid; /*!< The list of the point cloud names, for the legend.*/
  std::vector<legendParams> m_vlegends; /*!< The list of the legend items.*/
  std::vector<std::mutex *> m_vpmutex; /*!< The list of mutexes protecting the point clouds from data race when using the drawing thread.*/
  std::vector<vpColVector> m_vweights; /*!< The list of confidence weights of each point cloud.*/
  std::thread m_threadDisplay; /*!< The non-blocking drawing thread.*/
  bool m_hasToRun; /*!< If true, the drawing thread is running. Otherwise, it is stopped.*/
  std::string m_title; /*!< The title of the viewer window.*/
  bool m_hasToSavePCDs; /*!< If true, the point clouds will be saved at each iteration of the drawing thread.*/
  std::string m_outFolder; /*!< If non empty, the path to the folders where the point clouds will be saved.*/
};
END_VISP_NAMESPACE
#endif // #if defined(VISP_HAVE_PCL)
#endif // _vpPclVisualizer_h_
