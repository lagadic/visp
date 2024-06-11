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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpConfig.h>
#if defined(VISP_HAVE_PCL) && defined(VISP_HAVE_PCL_IO) && defined(VISP_HAVE_THREADS)
// PCL
#include<pcl/io/pcd_io.h>

// ViSP
#include <visp3/gui/vpPclViewer.h>
#include <visp3/gui/vpColorBlindFriendlyPalette.h>
#include <visp3/core/vpIoTools.h>

BEGIN_VISP_NAMESPACE
const std::vector<vpColorBlindFriendlyPalette::Palette> gcolor = { vpColorBlindFriendlyPalette::Palette::Green, vpColorBlindFriendlyPalette::Palette::Vermillon,vpColorBlindFriendlyPalette::Palette::Blue,
                                     vpColorBlindFriendlyPalette::Palette::Black, vpColorBlindFriendlyPalette::Palette::Orange, vpColorBlindFriendlyPalette::Palette::Purple,
                                     vpColorBlindFriendlyPalette::Palette::Yellow };

const unsigned int gc_nbColorMax = 7;

pcl::visualization::PCLVisualizer::Ptr vpPclViewer::sp_viewer(nullptr);

std::vector<std::vector<double>> vpPclViewer::s_vhandler;

int vpPclViewer::s_width = 640;
int vpPclViewer::s_height = 480;
int vpPclViewer::s_posU = 40;
int vpPclViewer::s_posV = 40;
double vpPclViewer::s_ignoreThresh = 0.95;

vpPclViewer::vpPclViewer(const std::string &title, const int &width, const int &height
  , const int &posU, const int &posV
  , const std::string &outFolder, const double &ignoreThreshold)
  : m_hasToRun(false)
  , m_title(title)
  , m_hasToSavePCDs(false)
  , m_outFolder(outFolder)
{
  setOutFolder(outFolder);
  setIgnoreThreshold(ignoreThreshold);
  s_width = width;
  s_height = height;
  s_posU = posU;
  s_posV = posV;
}

vpPclViewer ::~vpPclViewer()
{
  // Asking to stop thread
  stopThread();

  // Deleting point clouds
  for (unsigned int i = 0; i < m_vPointClouds.size(); i++) {
    m_vPointClouds[i].reset();
  }
  m_vPointClouds.clear();

  // Deleting mutexes
  for (unsigned int id = 0; id < m_vpmutex.size(); id++) {
    delete m_vpmutex[id];
  }
  m_vpmutex.clear();

  // Deleting the viewer
  if (sp_viewer) {
    sp_viewer.reset();
  }
}

void vpPclViewer::setNameWindow(const std::string &nameWindow)
{
  sp_viewer->setWindowName(nameWindow);
}

void vpPclViewer::setOutFolder(const std::string &outFolder)
{
  m_outFolder = outFolder;
  if (!m_outFolder.empty()) {
    // An output folder has been set by the user
    m_hasToSavePCDs = true;
    if (!vpIoTools::checkDirectory(m_outFolder)) {
      // The output folder does not exist => creating it
      vpIoTools::makeDirectory(m_outFolder);
    }
  }
  else {
    // No output folder was assigned to the visualizer
    m_hasToSavePCDs = false;
  }
}

void vpPclViewer::setIgnoreThreshold(const double &ignoreThreshold)
{
  if (ignoreThreshold < 0. || ignoreThreshold > 1.) {
    throw(vpException(vpException::badValue, "[vpPclViewer::setIgnoreThreshold] Fatal error: threshold must be in range [0. ; 1.]"));
  }
  s_ignoreThresh = ignoreThreshold;
}

void vpPclViewer::updateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id, const bool &hasToKeepColor)
{
  if (m_hasToRun) {
    // Threaded mode
    if (hasToKeepColor) {
      // The point cloud will be displayed with its original colors
      threadUpdateSurfaceOriginalColor(surface, id);
    }
    else {
      // The point cloud will be displayed with its default colors
      threadUpdateSurface(surface, id);
    }
  }
  else {
    // Blocking mode
    vpColVector fakeWeights; // Fake weights that are all equal to 1, to keep all the points
    updateSurface(surface, id, fakeWeights, hasToKeepColor);
  }
}

void vpPclViewer::updateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id,
  const vpColVector &weights, const bool &hasToKeepColor)
{
  if (m_hasToRun) {
    // Threaded mode
    if (hasToKeepColor) {
      // The point cloud will be displayed with its original colors
      threadUpdateSurfaceOriginalColor(surface, id, weights);
    }
    else {
      // The point cloud will be displayed with its default colors
      threadUpdateSurface(surface, id, weights);
    }
  }
  else {
    // Blocking mode
    // If the saved pcl corresponding to \b id was not initialized, initialize it
    if (!m_vPointClouds[id]) {
      m_vPointClouds[id].reset(new pclPointCloudPointXYZRGB());
    }

    // Resize if needed the saved pcl corresponding to \b id
    unsigned int nbPoints = surface->size();
    m_vPointClouds[id]->resize(nbPoints);

    // Save the new weights and check if they must be used
    m_vweights[id] = weights;
    bool use_weights = (weights.size() > 0);

    // Keep only the points that are above \b s_ignoreThresh
    for (unsigned int index = 0; index < nbPoints; index++) {
      bool addPoint = false;
      if (use_weights) {
        addPoint = (weights[index] > s_ignoreThresh);
      }
      else {
        addPoint = true;
      }

      pclPointXYZRGB pt = surface->at(index);
      if (addPoint) {
        m_vPointClouds[id]->at(index).x = pt.x;
        m_vPointClouds[id]->at(index).y = pt.y;
        m_vPointClouds[id]->at(index).z = pt.z;

        m_vPointClouds[id]->at(index).r = s_vhandler[id][0];
        m_vPointClouds[id]->at(index).g = s_vhandler[id][1];
        m_vPointClouds[id]->at(index).b = s_vhandler[id][2];
      }
    }

    // Try to update the pcl corresponding to the ID
    // Throw a \b vpException::notInitialized exception if the pcl was not added to the list of known pcl first
    bool status = sp_viewer->updatePointCloud(m_vPointClouds[id], m_vmeshid[id]);
    if (!status) {
      std::stringstream err_msg;
      err_msg << "[vpPclViewer ::updateSurface] ID " << m_vmeshid[id] << " not found !" << std::endl;
      throw(vpException(vpException::notInitialized, err_msg.str()));
    }
  }
}

unsigned int vpPclViewer::addSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const std::string &name, const std::vector<unsigned char> &v_color)
{
  vpColVector emptyWeights; // Fake weights that are all equal to 1, to keep all the points
  return addSurface(surface, emptyWeights, name, v_color);
}

unsigned int vpPclViewer::addSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const vpColVector &weights, const std::string &name, const std::vector<unsigned char> &v_color)
{
  static unsigned int nbSurfaces = 0;
  unsigned int id = m_vPointClouds.size();

  // Creating a new pcl and saving it in the container
  pclPointCloudPointXYZRGB::Ptr p_pointCloud(new pclPointCloudPointXYZRGB());
  m_vPointClouds.push_back(p_pointCloud);

  // Sizing it accordingly to the input pcl
  unsigned int nbPoints = surface->size();
  m_vPointClouds[id]->resize(nbPoints);

  // Affecting a color to the point cloud and its legend
  std::vector<unsigned char> v_RGB;
  if (v_color.size() < 3) {
    // Affecting a default color to the pcl and its legend, if the user does not want to use its original color
    // when displaying it
    vpColorBlindFriendlyPalette color(gcolor[nbSurfaces]);
    v_RGB = color.to_RGB();
  }
  else {
    // Keeping the colors decided by the user
    v_RGB = v_color;
  }

  std::vector<double> v_RGBdouble = {
    (double)v_RGB[0],
    (double)v_RGB[1],
    (double)v_RGB[2]
  };
  s_vhandler.push_back(v_RGBdouble);

  // Storing the weights attached to the surface
  m_vweights.push_back(weights);
  bool use_weigths = weights.size() > 0;

  // Copying the coordinates of each point of the original pcl,
  // while affecting them the default color.
  // Points that have a weight below \b s_ignoreThresh are ignored
  for (unsigned int index = 0; index < nbPoints; index++) {
    bool shouldPointBeVisible = false;
    if (use_weigths) {
      // If weights are defined, the point should be visible only
      // if the weight is greater than the ignore threshold.
      shouldPointBeVisible = weights[index] > s_ignoreThresh;
    }
    else {
      // No weights are used => every points must be considered
      shouldPointBeVisible = true;
    }

    pclPointXYZRGB pt = surface->at(index);
    m_vPointClouds[id]->at(index).x = pt.x;
    m_vPointClouds[id]->at(index).y = pt.y;
    m_vPointClouds[id]->at(index).z = pt.z;

    if (shouldPointBeVisible) {
      m_vPointClouds[id]->at(index).r = s_vhandler[id][0];
      m_vPointClouds[id]->at(index).g = s_vhandler[id][1];
      m_vPointClouds[id]->at(index).b = s_vhandler[id][2];
    }
    else {
      m_vPointClouds[id]->at(index).r = 0.;
      m_vPointClouds[id]->at(index).g = 0.;
      m_vPointClouds[id]->at(index).b = 0.;
    }

  }

  //  std::cout << "Point cloud: width=" << m_vPointClouds[id]->width << ", height= " << m_vPointClouds[id]->height << std::endl;

  // Checking if the user specified a name for the pcl
  if (!name.empty()) {
    // Yes => we save it (will be used for the legend, and for
    // the viewer to know which pcl we are changing when updating pcls)
    m_vmeshid.push_back(name);
  }
  else {
    // No => we create one, for the very same reasons
    m_vmeshid.push_back("point_cloud" + std::to_string(id));
  }
  //  std::cout << "[vpPclViewer ::addSurface] Added ID " << m_vmeshid[id] << " to the list of known point clouds" << std::endl;
  if (sp_viewer) {
    // The viewer is already on, we can add the pcl to its known list
    sp_viewer->addPointCloud(m_vPointClouds[id], m_vmeshid[id]);
  }

  // Updating the number of known pcls
  nbSurfaces = (nbSurfaces + 1) % gc_nbColorMax;

  // Adding a mutex to protect the new pcl when using \b threadUpdateSurface
  m_vpmutex.push_back(new std::mutex());

  // Adding a legend for this pcl
  legendParams legend;
  m_vlegends.push_back(legend);
  legendParams &newLegend = m_vlegends[id];
  newLegend.m_text = m_vmeshid[id];
  newLegend.m_posU = 10;
  newLegend.m_posV = 10;
  newLegend.m_size = 16;
  if (id > 0) {
    newLegend.m_posV = m_vlegends[id - 1].m_posV + newLegend.m_size;
  }
  newLegend.m_rRatio = s_vhandler[id][0] / 255.0;
  newLegend.m_gRatio = s_vhandler[id][1] / 255.0;
  newLegend.m_bRatio = s_vhandler[id][2] / 255.0;

  if (sp_viewer) {
    // The viewer is on => we add the legend on the screen
    sp_viewer->addText(newLegend.m_text, newLegend.m_posU, newLegend.m_posV, newLegend.m_rRatio, newLegend.m_gRatio, newLegend.m_bRatio);
  }

  return id;
}



void vpPclViewer::display()
{
  stopThread(); // We have to stop the thread to manipulate the viewer with a blocking waiting
  if (!sp_viewer) {
    // The viewer was not created => creating a new one
    sp_viewer.reset(new pcl::visualization::PCLVisualizer(m_title));
    sp_viewer->addCoordinateSystem(0.5); // Display a coordinate system whose axis are 0.5m long
    sp_viewer->initCameraParameters(); // Initialize the viewer with default camera settings
    sp_viewer->setSize(s_width, s_height); // Setting the size of the viewer window
    sp_viewer->setPosition(s_posU, s_posV); // Setting the position of the viewer window on the screen
    sp_viewer->resetCamera();

    for (unsigned int id = 0; id < m_vPointClouds.size(); id++) {
      sp_viewer->addPointCloud(m_vPointClouds[id], m_vmeshid[id]);
      sp_viewer->addText(m_vlegends[id].m_text, m_vlegends[id].m_posU, m_vlegends[id].m_posV, m_vlegends[id].m_rRatio, m_vlegends[id].m_gRatio, m_vlegends[id].m_bRatio);
    }
  }
  sp_viewer->spin();
}

void vpPclViewer::refresh(const int &timeout, const bool &waitForDrawing)
{
  sp_viewer->spinOnce(timeout, waitForDrawing);
}

void vpPclViewer::launchThread()
{
  // Check if the visualization thread is already started
  if (!m_hasToRun) {
    // Thread not started => starting it now
    m_hasToRun = true;
    m_threadDisplay = std::thread(vpPclViewer::runThread, this);
  }
}

void vpPclViewer::stopThread()
{
  // Check if the visualization thread is running
  if (m_hasToRun) {
    // Thread is running, asking it to stop
    m_hasToRun = false;
    m_threadDisplay.join();
  }
}

void vpPclViewer::runThread(vpPclViewer *p_visualizer)
{
  p_visualizer->loopThread();
}

void vpPclViewer::loopThread()
{
  bool useWeights; /*!< Will be used to know if the points of the pcl have weights. If so, will display only the ones whose weight exceed a threshold.*/
  sp_viewer.reset(new pcl::visualization::PCLVisualizer(m_title)); // Allocating a new viewer or resetting the old one.
  sp_viewer->addCoordinateSystem(0.5); // Display a coordinate system whose axis are 0.5m long
  sp_viewer->initCameraParameters(); // Initialize the viewer with default camera settings
  sp_viewer->setSize(s_width, s_height); // Setting the size of the viewer window
  sp_viewer->setPosition(s_posU, s_posV); // Setting the position of the viewer window on the screen
  sp_viewer->resetCamera();
  unsigned int iter = 0;

  // Running the main loop of the thread
  while (m_hasToRun) {
    for (unsigned int id = 0; id < m_vmeshid.size(); id++) {
      m_vpmutex[id]->lock();
      unsigned int nbPoints = m_vPointClouds[id]->size();
      // Checking if the pcl[id] has weights attached to its points
      unsigned int nbWeights = m_vweights[id].size();
      useWeights = (nbWeights > 0);
      if (useWeights) {
        // Setting points for which the weights are lower than \b s_ignoreThresh to be of color \b s_colorRejectedPoints
        for (unsigned int index = 0; index < nbPoints; index++) {
          if (m_vweights[id][index] < s_ignoreThresh) {
            m_vPointClouds[id]->at(index).r = 0.0;
            m_vPointClouds[id]->at(index).g = 0.0;
            m_vPointClouds[id]->at(index).b = 0.0;
          }

        }
      }

      // If updatePointCloud fails, it means that the pcl was not previously known by the viewer
      if (!sp_viewer->updatePointCloud(m_vPointClouds[id], m_vmeshid[id])) {
        // Add the pcl to the list of pcl known by the viewer + the according legend
        sp_viewer->addPointCloud(m_vPointClouds[id], m_vmeshid[id]);
        sp_viewer->addText(m_vlegends[id].m_text, m_vlegends[id].m_posU, m_vlegends[id].m_posV, m_vlegends[id].m_rRatio, m_vlegends[id].m_gRatio, m_vlegends[id].m_bRatio);
      }

      // If the pcl is not empty and the \b vpPclViewer is asked to save the pcls,
      // create a new file name and save the pcl
      if (m_vPointClouds[id]->size() > 0 && m_hasToSavePCDs) {
        std::string filename = vpIoTools::createFilePath(m_outFolder, m_vmeshid[id] + std::to_string(iter) + ".pcd");
        pcl::io::savePCDFile(filename, *m_vPointClouds[id]);
      }

      m_vpmutex[id]->unlock();
    }
    sp_viewer->spinOnce(100, true);

    iter++;
  }
  // When leaving the thread, resetting the viewer
  sp_viewer.reset();
}

void vpPclViewer::threadUpdateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id)
{
  threadUpdateSurface(surface, id, vpColVector());
}

void vpPclViewer::threadUpdateSurfaceOriginalColor(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id)
{
  threadUpdateSurfaceOriginalColor(surface, id, vpColVector());
}

void vpPclViewer::threadUpdateSurface(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id, const vpColVector &weights)
{
  m_vpmutex[id]->lock();
  m_vweights[id] = weights; // Saving the weights affected to each point of the pcl
  unsigned int nbPoints = surface->size();
  m_vPointClouds[id]->resize(nbPoints); // Resizing internal point cloud to the size of the input surface

  // Iterating on each point of the pcl to change the color of the points
  // for the default value affected to this pcl
  for (unsigned int index = 0; index < nbPoints; index++) {
    pclPointXYZRGB pt = surface->at(index);
    m_vPointClouds[id]->at(index).x = pt.x;
    m_vPointClouds[id]->at(index).y = pt.y;
    m_vPointClouds[id]->at(index).z = pt.z;

    m_vPointClouds[id]->at(index).r = s_vhandler[id][0];
    m_vPointClouds[id]->at(index).g = s_vhandler[id][1];
    m_vPointClouds[id]->at(index).b = s_vhandler[id][2];
  }
  m_vpmutex[id]->unlock();
}

void vpPclViewer::threadUpdateSurfaceOriginalColor(const pclPointCloudPointXYZRGB::Ptr &surface, const unsigned int &id, const vpColVector &weights)
{
  m_vpmutex[id]->lock();
  m_vweights[id] = weights; // Saving the weights affected to each point of the pcl
  unsigned int nbPoints = surface->size();
  m_vPointClouds[id]->resize(nbPoints); // Resizing internal point cloud to the size of the input surface

  // As we keep the color of the original pcl, a plain copy will do the job
  pcl::copyPointCloud(*surface, *m_vPointClouds[id]);

  m_vpmutex[id]->unlock();
}
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpD3DRenderer.cpp.o) has no symbols
void dummy_vpPCLPointCLoudVisualization() { };
#endif
#endif
