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

#ifndef VP_DISPLAY_PCL_H
#define VP_DISPLAY_PCL_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_VISUALIZATION) && defined(VISP_HAVE_THREADS)

#include <mutex>
#include <thread>
#include <string>

#include <visp3/core/vpColor.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpDisplayPCL
  \ingroup group_gui_plotter
  This class enables real time plotting of 3D point clouds. It relies on the PCL library.
  To see how to install PCL library, please refer to the \ref soft_tool_pcl section.

  \warning MacOS currently can only use the monothread method `display`, otherwise they get
  the error `uncaught exception 'NSInternalInconsistencyException', reason: 'NSWindow should only be instantiated on the main thread!'`

  <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>

  <b>Tutorials</b><br>
  <span style="margin-left:2em"> If you want detailed explanation on how to use this class, you may have a look at:</span><br>

  - \ref tutorial-display-pcl
  - \ref tutorial-hsv-segmentation-pcl
*/
class VISP_EXPORT vpDisplayPCL
{
public:
  vpDisplayPCL(int posx = 0, int posy = 0, const std::string &window_name = "");
  vpDisplayPCL(unsigned int width, unsigned int height, int posx = 0, int posy = 0, const std::string &window_name = "");
  ~vpDisplayPCL();

  void setVerbose(bool verbose);
  void display();
  void startThread(const bool &colorThread = false);
  void startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const std::string &name = "", const vpColor &color = vpColor::red);
  void startThread(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);
  void addPointCloud(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, const std::string &name = "", const vpColor &color = vpColor::red);
  void addPointCloud(std::mutex &mutex, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud);
  void setPosition(int posx, int posy);
  void setWindowName(const std::string &window_name);
  void stop();
private:
  void run();
  void runColor();
  void createViewer();
  void insertLegend(const size_t &id);

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/**
 * \brief Basic structure to handle the point clouds to display.
 */
  typedef struct PointCloudHandling
  {
    bool m_do_init; //!< Managed internally. True to insert for the first time the point cloud in the viewer, false to update the point cloud instead.
    std::string m_name; //!< The name of the handled point cloud. It is used both by the viewer as unique identifier and in the legend.
    static unsigned int s_nb; //!< The number of point clouds that are handled.

    /**
     * \brief Create a unique name for the point cloud.
     *
     * \return std::string The unique name.
     */
    std::string createName()
    {
      std::stringstream ss;
      ss << "Point cloud " << s_nb;
      return ss.str();
    }

    /**
     * \brief Initializes the point cloud handler
     *
     * \param[in] name Its name, used as ID and potentially in the legend.
     * If empty, it will be automatically generated.
     */
    void init(const std::string &name)
    {
      if (name.empty()) {
        m_name = createName();
      }
      else {
        m_name = name;
      }
      m_do_init = true;
      ++s_nb;
    }

    /**
     * \brief Initialize the point cloud handler.
     * Its name, used as ID and potentially in the legend, will be automatically generated.
     */
    void init()
    {
      init(createName());
    }

    /**
     * \brief Construct a new Point Cloud Handling object.
     * Its name, used as ID and potentially in the legend, will be automatically generated.
     */
    PointCloudHandling()
    {
      init();
    }

    /**
     * \brief Construct a new Point Cloud Handling object
     *
     * \param[in] name Name used as ID and potentially in the legend.
     */
    PointCloudHandling(const std::string &name)
    {
      init(name);
    }
  } PointCloudHandling;

  typedef struct XYZPointCloudHandling : public PointCloudHandling
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pcl; //!< Pointer towards the textureless point cloud.
    vpColor m_color; //!< Monochromic color to render the point cloud.

    /**
     * \brief Construct a new XYZPointCloudHandling object
     *
     * \param[in] pcl Pointer towards the textureless point cloud.
     * \param[in] color Monochromic color to render the point cloud.
     */
    XYZPointCloudHandling(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, const vpColor &color)
      : PointCloudHandling()
      , m_pcl(pcl)
      , m_color(color)
    { }

    /**
     * \brief Construct a new XYZPointCloudHandling object
     *
     * \param[in] pcl Pointer towards the textureless point cloud.
     * \param[in] name Name of the point cloud, used as ID by the viewer and inserted in the legend.
     * \param[in] color Monochromic color to render the point cloud.
     */
    XYZPointCloudHandling(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl, const std::string &name, const vpColor &color)
      : PointCloudHandling(name)
      , m_pcl(pcl)
      , m_color(color)
    { }
  } XYZPointCloudHandling;

  /**
   * \brief Structure that handles textured point cloud.
   */
  typedef struct ColoredPointCloudHandling : public PointCloudHandling
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_pcl; //!< Pointer towards the textured point cloud

    /**
     * \brief Construct a new Colored Point Cloud Handling object
     *
     * \param[in] pcl Pointer towards the textured point cloud
     */
    ColoredPointCloudHandling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
      : PointCloudHandling()
      , m_pcl(pcl)
    { }

    /**
     * \brief Construct a new Colored Point Cloud Handling object
     *
     * \param[in] pcl Pointer towards the textured point cloud
     * \param[in] name Name of the point cloud, used as ID by the viewer.
     */
    ColoredPointCloudHandling(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, const std::string &name)
      : PointCloudHandling(name)
      , m_pcl(pcl)
    { }
  } ColoredPointCloudHandling;
#endif

  bool m_stop;
  bool m_thread_running; //!< Set to true once the startThread method is called.
  bool m_verbose;
  std::thread m_thread; //!< Non-blocking display thread.
  std::mutex m_mutex_vector; //!< Mutex to protect the vectors of point clouds
  std::vector<std::pair<std::mutex &, XYZPointCloudHandling>> mv_xyz_pcl; //!< Storage for textureless point clouds + associated mutex
  std::vector<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>> mv_xyz_handlers; //!< Mono-color handlers for textureless point clouds
  std::vector<std::pair<std::mutex &, ColoredPointCloudHandling>> mv_colored_pcl; //!< Storage for textured point clouds + associated mutex
  std::vector<pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>> mv_color_handlers; //!< Color handlers for textured point clouds
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
