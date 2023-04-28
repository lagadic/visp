/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Description:
 * D3D renderer for windows 32 display
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/
#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpConfig.h>
#if (defined(VISP_HAVE_PCL))
// PCL
#include<pcl/io/pcd_io.h>

// ViSP
#include <visp3/gui/vpPclPointCloudVisualization.h>
#include <visp3/gui/vpColorBlindFriendlyPalette.h>
#include <visp3/core/vpIoTools.h>

const std::vector<vpColorBlindFriendlyPalette::Palette> gcolor= {vpColorBlindFriendlyPalette::Palette::Green, vpColorBlindFriendlyPalette::Palette::Vermillon,vpColorBlindFriendlyPalette::Palette::Blue,
                                     vpColorBlindFriendlyPalette::Palette::Black, vpColorBlindFriendlyPalette::Palette::Orange, vpColorBlindFriendlyPalette::Palette::Purple,
                                     vpColorBlindFriendlyPalette::Palette::Yellow};

const unsigned int gc_nbColorMax = 7;

pcl::visualization::PCLVisualizer::Ptr vpPclPointCloudVisualization ::gp_viewer(nullptr);

std::vector<std::vector<double>> vpPclPointCloudVisualization ::g_vhandler;

int vpPclPointCloudVisualization::s_width           =  640;
int vpPclPointCloudVisualization::s_height          =  480;
int vpPclPointCloudVisualization::s_px              =   40;
int vpPclPointCloudVisualization::s_py              =   40;
double vpPclPointCloudVisualization::s_ignoreThresh = 0.95;

vpPclPointCloudVisualization ::vpPclPointCloudVisualization ( const std::string &title , const int &width, const int &height
                                                            , const int &px, const int &py
                                                            , const std::string &outFolder, const double &ignoreThreshold)
  : _hasToRun(false)
  , _title(title)
  , _hasToSavePCDs(false)
  , _outFolder(outFolder)
{
  set_outFolder(outFolder);
  set_ignoreThreshold(ignoreThreshold);
  s_width  = width ;
  s_height = height;
  s_px     = px    ;
  s_py     = py    ;
}

vpPclPointCloudVisualization ::~vpPclPointCloudVisualization ()
{
  std::cout << "[vpPclPointCloudVisualization ::~pclPointCloudVisualization] Asking to stop thread" << std::endl << std::flush;
  stopThread();
  std::cout << "[vpPclPointCloudVisualization ::~vpPclPointCloudVisualization] Thread stopped" << std::endl << std::flush;
  for(unsigned int i = 0; i < _vPointClouds.size(); i++){
    _vPointClouds[i].reset();
  }
  _vPointClouds.clear();
  std::cout << "[vpPclPointCloudVisualization ::~vpPclPointCloudVisualization] Point cloud reseted" << std::endl << std::flush;
  for(unsigned int id =0; id < _vpmutex.size(); id++){
    delete _vpmutex[id];
  }
  _vpmutex.clear();
  std::cout << "[vpPclPointCloudVisualization ::~vpPclPointCloudVisualization] Mutex deleted" << std::endl << std::flush;
  if(gp_viewer){
    gp_viewer.reset();
  }
}

void vpPclPointCloudVisualization ::set_nameWindow(const std::string &nameWindow)
{
  gp_viewer->setWindowName(nameWindow);
}

void vpPclPointCloudVisualization::set_outFolder(const std::string &outFolder)
{
  _outFolder = outFolder;
  if(!_outFolder.empty()){
    // An output folder has been set by the user
    _hasToSavePCDs = true;
    if(!vpIoTools::checkDirectory(_outFolder)){
      // The output folder does not exist => creating it
      vpIoTools::makeDirectory(_outFolder);
    }
  }else{
    // No output folder was assigned to the visualizer
    _hasToSavePCDs = false;
  }
}

void vpPclPointCloudVisualization::set_ignoreThreshold(const double &ignoreThreshold)
{
  if(ignoreThreshold < 0. || ignoreThreshold > 1.)
  {
    throw(vpException(vpException::badValue, "[vpPclPointCloudVisualization::set_ignoreThreshold] Fatal error: threshold must be in range [0. ; 1.]"));
  }
  s_ignoreThresh = ignoreThreshold;
}

void vpPclPointCloudVisualization ::updateSurface(const pclPointCloud::Ptr &surface, unsigned int id)
{
  unsigned int nbPoints = surface->size();
  vpColVector fakeWeights(nbPoints, 1.); // Fake weights that are all equal to 1, to keep all the points
  updateSurface(surface, id, fakeWeights);
}

void vpPclPointCloudVisualization ::updateSurface(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector &weights)
{
  // If the saved pcl corresponding to \b id was not initialized, initialize it
  if(!_vPointClouds[id]){
    _vPointClouds[id].reset(new pclPointCloud());
  }

  // Resize if needed the saved pcl corresponding to \b id
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  // Keep only the points that are above \b s_ignoreThresh
  for(unsigned int index = 0; index < nbPoints; index++){
    if(weights[index] > s_ignoreThresh){
      pclPoint pt = surface->at(index);
      _vPointClouds[id]->at(index).x = pt.x;
      _vPointClouds[id]->at(index).y = pt.y;
      _vPointClouds[id]->at(index).z = pt.z;

      _vPointClouds[id]->at(index).r = g_vhandler[id][0];
      _vPointClouds[id]->at(index).g = g_vhandler[id][1];
      _vPointClouds[id]->at(index).b = g_vhandler[id][2];
    }
  }

  // Try to update the pcl corresponding to the ID
  // Throw a \b vpException::notInitialized exception if the pcl was not added to the list of known pcl first
  bool status = gp_viewer->updatePointCloud(_vPointClouds[id], _vmeshid[id]);
  if(!status){
    std::stringstream err_msg;
    err_msg << "[vpPclPointCloudVisualization ::updateSurface] ID " << _vmeshid[id] << " not found !" << std::endl;
    throw(vpException(vpException::notInitialized, err_msg.str()));
  }
}

unsigned int vpPclPointCloudVisualization ::addSurface(const pclPointCloud::Ptr &surface, std::string name)
{
  unsigned int nbPoints = surface->size();
  vpColVector fakeWeigths(nbPoints, 1.); // Fake weights that are all equal to 1, to keep all the points
  return addSurface(surface, fakeWeigths, name);
}

unsigned int vpPclPointCloudVisualization ::addSurface(const pclPointCloud::Ptr &surface, const vpColVector &weights, std::string name)
{
  static unsigned int nbSurfaces = 0;
  unsigned int id = _vPointClouds.size();

  // Creating a new pcl and saving it in the container
  pclPointCloud::Ptr p_pointCloud(new pclPointCloud());
  _vPointClouds.push_back(p_pointCloud);

  // Sizing it accordingly to the input pcl
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  // Affecting a default color to the pcl, if the user does not want to use its original color
  // when displaying it
  vpColorBlindFriendlyPalette color(gcolor[nbSurfaces]);
  std::vector<unsigned char> v_RGB = color.to_RGB();
  std::vector<double> v_RGBdouble = {
    (double)v_RGB[0],
    (double)v_RGB[1],
    (double)v_RGB[2]
  };
  g_vhandler.push_back(v_RGBdouble);

  // Copying the coordinates of each point of the original pcl,
  // while affecting them the default color.
  // Points that have a weight below \b s_ignoreThresh are ignored
  for(unsigned int index = 0; index < nbPoints; index++){
    if(weights[index] > s_ignoreThresh){
      pclPoint pt = surface->at(index);
      _vPointClouds[id]->at(index).x = pt.x;
      _vPointClouds[id]->at(index).y = pt.y;
      _vPointClouds[id]->at(index).z = pt.z;

      _vPointClouds[id]->at(index).r = g_vhandler[id][0];
      _vPointClouds[id]->at(index).g = g_vhandler[id][1];
      _vPointClouds[id]->at(index).b = g_vhandler[id][2];
    }

  }

  //  std::cout << "Point cloud: width=" << _vPointClouds[id]->width << ", height= " << _vPointClouds[id]->height << std::endl;

  // Checking if the user specified a name for the pcl
  if(!name.empty()){
    // Yes => we save it (will be used for the legend, and for 
    // the viewer to know which pcl we are changing when updating pcls)
    _vmeshid.push_back(name);
  }else{
    // No => we create one, for the very same reasons
    _vmeshid.push_back("point_cloud" + std::to_string(id));
  }
//  std::cout << "[vpPclPointCloudVisualization ::addSurface] Added ID " << _vmeshid[id] << " to the list of known point clouds" << std::endl;
  if(gp_viewer){
    // The viewer is already on, we can add the pcl to its known list
    gp_viewer->addPointCloud(_vPointClouds[id], _vmeshid[id]);
  }

  // Updating the number of known pcls 
  nbSurfaces = (nbSurfaces + 1) % gc_nbColorMax;

  // Adding a mutex to protect the new pcl when using \b threadUpdateSurface
  _vpmutex.push_back(new std::mutex());

  // Adding a legend for this pcl
  legendParams legend;
  _vlegends.push_back(legend);
  legendParams& newLegend = _vlegends[id];
  newLegend._text = _vmeshid[id];
  newLegend._posX = 10;
  newLegend._posY = 10;
  newLegend._size = 16;
  if(id > 0){
    newLegend._posY = _vlegends[id - 1]._posY + newLegend._size;
  }
  newLegend._rRatio = g_vhandler[id][0] / 255.0;
  newLegend._gRatio = g_vhandler[id][1] / 255.0;
  newLegend._bRatio = g_vhandler[id][2] / 255.0;

  if(gp_viewer){
    // The viewer is on => we add the legend on the screen
    gp_viewer->addText(newLegend._text, newLegend._posX, newLegend._posY, newLegend._rRatio, newLegend._gRatio, newLegend._bRatio );
  }

  return id;
}



void vpPclPointCloudVisualization ::display()
{
  stopThread(); // We have to stop the thread to manipulate the viewer with a blocking waiting
  if(!gp_viewer){
    // The viewer was not created => creating a new one
    gp_viewer.reset(new pcl::visualization::PCLVisualizer(_title));
    gp_viewer->setSize(800,600);
    gp_viewer->addCoordinateSystem (0.5);
  }
  gp_viewer->spin();
}

void vpPclPointCloudVisualization ::refresh()
{
  gp_viewer->spinOnce(100,true);
}

void vpPclPointCloudVisualization ::launchThread()
{
  // Check if the visualization thread is already started
  if(!_hasToRun){
    // Thread not started => starting it now
    _hasToRun = true;
    _threadDisplay = std::thread(vpPclPointCloudVisualization ::runThread, this);
  }
}

void vpPclPointCloudVisualization ::stopThread()
{
  // Check if the visualization thread is running
  if(_hasToRun){
    // Thread is running, asking it to stop
    _hasToRun = false;
    _threadDisplay.join();
  }
}

void vpPclPointCloudVisualization ::runThread(vpPclPointCloudVisualization  *p_visualizer)
{
  p_visualizer->loopThread();
}

void vpPclPointCloudVisualization::copyLegendParams(const vpPclPointCloudVisualization::legendParams &from, vpPclPointCloudVisualization::legendParams &to)
{
  to._text   = from._text  ;
  to._posX   = from._posX  ;
  to._posY   = from._posY  ;
  to._size   = from._size  ;
  to._rRatio = from._rRatio;
  to._gRatio = from._gRatio;
  to._bRatio = from._bRatio;
}

void vpPclPointCloudVisualization ::loopThread()
{
  bool useWeights; /*!< Will be used to know if the points of the pcl have weights. If so, will display only the ones whose weight exceed a threshold.*/
  gp_viewer.reset(new pcl::visualization::PCLVisualizer(_title)); // Allocating a new viewer or resetting the old one.
  gp_viewer->addCoordinateSystem (0.5); // Display a coordinate system whose axis are 0.5m long
  gp_viewer->initCameraParameters(); // Initialize the viewer with default camera settings
  gp_viewer->setSize(s_width,s_height); // Setting the size of the viewer window
  gp_viewer->setPosition(s_px, s_py); // Setting the position of the viewer window on the screen
  unsigned int iter;

  // Running the main loop of the thread
  while(_hasToRun){
    for(unsigned int id=0; id< _vmeshid.size(); id++){
      // Checking if the pcl[id] has weights attached to its points
      unsigned int nbWeights = _weights.size(); 
      useWeights = (nbWeights > 0);
      unsigned int nbPoints = _vPointClouds[id]->size();

      _vpmutex[id]->lock();
      if(useWeights){
        _mutexWeights.lock();
        // Setting points for which the weights are lower than \b s_ignoreThresh to be of color \b s_colorRejectedPoints
        for(unsigned int index = 0; index < nbPoints; index++){
            if(_weights[index] < s_ignoreThresh){
              _vPointClouds[id]->at(index).r = 0.0;
              _vPointClouds[id]->at(index).g = 0.0;
              _vPointClouds[id]->at(index).b = 0.0;
            }

        }
        _mutexWeights.unlock();
      }

      // If updatePointCloud fails, it means that the pcl was not previously known by the viewer
      if(!gp_viewer->updatePointCloud(_vPointClouds[id], _vmeshid[id])){
        // Add the pcl to the list of pcl known by the viewer + the according legend
        gp_viewer->addPointCloud(_vPointClouds[id], _vmeshid[id]);
        gp_viewer->addText(_vlegends[id]._text, _vlegends[id]._posX, _vlegends[id]._posY, _vlegends[id]._rRatio, _vlegends[id]._gRatio, _vlegends[id]._bRatio );
      }

      // If the pcl is not empty and the \b vpPclPointCloudVisualization is asked to save the pcls,
      // create a new file name and save the pcl
      if(_vPointClouds[id]->size() > 0 && _hasToSavePCDs){
        std::string filename = vpIoTools::createFilePath(_outFolder, _vmeshid[id] + std::to_string(iter) + ".pcd" );
        pcl::io::savePCDFile(filename, *_vPointClouds[id]);
      }

      _vpmutex[id]->unlock();
    }
    gp_viewer->spinOnce(100,true);

    iter++;
  }
  // When leaving the thread, resetting the viewer
  gp_viewer.reset();
}

void vpPclPointCloudVisualization ::threadUpdateSurface(const pclPointCloud::Ptr &surface, unsigned int id)
{
  _vpmutex[id]->lock();
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  // Iterating on each point of the pcl to change the color of the points
  // for the default value affected to this pcl
  for(unsigned int index = 0; index < nbPoints; index++){
      pclPoint pt = surface->at(index);
      _vPointClouds[id]->at(index).x = pt.x;
      _vPointClouds[id]->at(index).y = pt.y;
      _vPointClouds[id]->at(index).z = pt.z;

      _vPointClouds[id]->at(index).r = g_vhandler[id][0];
      _vPointClouds[id]->at(index).g = g_vhandler[id][1];
      _vPointClouds[id]->at(index).b = g_vhandler[id][2];
  }
  _vpmutex[id]->unlock();
}

void vpPclPointCloudVisualization ::threadUpdateSurfaceOriginalColor(const pclPointCloud::Ptr &surface, unsigned int id)
{
  _vpmutex[id]->lock();
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  // As we keep the color of the original pcl, a plain copy will do the job
  pcl::copyPointCloud(*surface, *_vPointClouds[id]);

  _vpmutex[id]->unlock();
}

void vpPclPointCloudVisualization ::threadUpdateSurface(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector &weights)
{
  _vpmutex[id]->lock();
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  // Iterating on each point of the pcl to change the color of the points
  // for the default value affected to this pcl
  for(unsigned int index = 0; index < nbPoints; index++){
      pclPoint pt = surface->at(index);
      _vPointClouds[id]->at(index).x = pt.x;
      _vPointClouds[id]->at(index).y = pt.y;
      _vPointClouds[id]->at(index).z = pt.z;

      _vPointClouds[id]->at(index).r = g_vhandler[id][0];
      _vPointClouds[id]->at(index).g = g_vhandler[id][1];
      _vPointClouds[id]->at(index).b = g_vhandler[id][2];
  }
  _vpmutex[id]->unlock();

  // Saving the weights affected to each point of the pcl
  _mutexWeights.lock();
  _weights = weights;
  _mutexWeights.unlock();
}

void vpPclPointCloudVisualization ::threadUpdateSurfaceOriginalColor(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector &weights)
{
  _vpmutex[id]->lock();
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  // As we keep the color of the original pcl, a plain copy will do the job
  pcl::copyPointCloud(*surface, *_vPointClouds[id]);

  _vpmutex[id]->unlock();

  // Saving the weights affected to each point of the pcl
  _mutexWeights.lock();
  _weights = weights;
  _mutexWeights.unlock();
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpD3DRenderer.cpp.o) has no
// symbols
void dummy_vpPCLPointCLoudVisualization(){};
#endif
#endif