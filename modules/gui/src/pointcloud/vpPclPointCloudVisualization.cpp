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

int vpPclPointCloudVisualization::s_width  = 640;
int vpPclPointCloudVisualization::s_height = 480;
int vpPclPointCloudVisualization::s_px     =  40;
int vpPclPointCloudVisualization::s_py     =  40;

vpPclPointCloudVisualization ::vpPclPointCloudVisualization (const std::string &title , const int &width, const int &height, const int &px, const int &py, const std::string &outFolder)
  : _hasToRun(false)
  , _title(title)
  , _hasToSavePCDs(false)
  , _outFolder(outFolder)
{
  if(!_outFolder.empty()){
    _hasToSavePCDs = true;
    if(!vpIoTools::checkDirectory(_outFolder)){
      vpIoTools::makeDirectory(_outFolder);
    }
  }
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

void vpPclPointCloudVisualization ::set_nameWindow(std::string nameWindow)
{
  gp_viewer->setWindowName(nameWindow);
}

void vpPclPointCloudVisualization::set_outFolder(std::string outFolder)
{
  _outFolder = outFolder;
  if(!_outFolder.empty()){
    _hasToSavePCDs = true;
    if(!vpIoTools::checkDirectory(_outFolder)){
      vpIoTools::makeDirectory(_outFolder);
    }
  }else{
    _hasToSavePCDs = false;
  }
}

void vpPclPointCloudVisualization ::updateSurface(const pclPointCloud::Ptr &surface, unsigned int id)
{
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  for(unsigned int index = 0; index < nbPoints; index++){
      pclPoint pt = surface->at(index);
      _vPointClouds[id]->at(index).x = pt.x;
      _vPointClouds[id]->at(index).y = pt.y;
      _vPointClouds[id]->at(index).z = pt.z;

      _vPointClouds[id]->at(index).r = g_vhandler[id][0];
      _vPointClouds[id]->at(index).g = g_vhandler[id][1];
      _vPointClouds[id]->at(index).b = g_vhandler[id][2];

  }

  bool status = gp_viewer->updatePointCloud(_vPointClouds[id], _vmeshid[id]);
  if(!status){
    std::cerr << "[vpPclPointCloudVisualization ::updateSurface] ID " << _vmeshid[id] << " not found !" << std::endl;
  }

}

void vpPclPointCloudVisualization ::updateSurface(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector &weights)
{
  if(!_vPointClouds[id]){
    _vPointClouds[id].reset(new pclPointCloud());
  }

  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  for(unsigned int index = 0; index < nbPoints; index++){
    if(weights[index] > 0.95){
      pclPoint pt = surface->at(index);
      _vPointClouds[id]->at(index).x = pt.x;
      _vPointClouds[id]->at(index).y = pt.y;
      _vPointClouds[id]->at(index).z = pt.z;

      _vPointClouds[id]->at(index).r = g_vhandler[id][0];
      _vPointClouds[id]->at(index).g = g_vhandler[id][1];
      _vPointClouds[id]->at(index).b = g_vhandler[id][2];
    }

  }

  bool status = gp_viewer->updatePointCloud(_vPointClouds[id], _vmeshid[id]);
  if(!status){
    std::cerr << "[vpPclPointCloudVisualization ::updateSurface] ID " << _vmeshid[id] << " not found !" << std::endl;
  }

}

unsigned int vpPclPointCloudVisualization ::addSurface(const pclPointCloud::Ptr &surface, std::string name)
{

  static unsigned int nbSurfaces = 0;
  unsigned int id = _vPointClouds.size();


  pclPointCloud::Ptr p_pointCloud(new pclPointCloud());
  _vPointClouds.push_back(p_pointCloud);

  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

//  std::cout << "Point cloud[" << id << "]: size=" << nbPoints << std::endl << std::flush;
  vpColorBlindFriendlyPalette color(gcolor[nbSurfaces]);
  std::vector<unsigned char> v_RGB = color.to_RGB();
  std::vector<double> v_RGBdouble = {
    (double)v_RGB[0],
    (double)v_RGB[1],
    (double)v_RGB[2]
  };
  g_vhandler.push_back(v_RGBdouble);

  for(unsigned int index = 0; index < nbPoints; index++){
      pclPoint pt = surface->at(index);
      _vPointClouds[id]->at(index).x = pt.x;
      _vPointClouds[id]->at(index).y = pt.y;
      _vPointClouds[id]->at(index).z = pt.z;

      _vPointClouds[id]->at(index).r = g_vhandler[id][0];
      _vPointClouds[id]->at(index).g = g_vhandler[id][1];
      _vPointClouds[id]->at(index).b = g_vhandler[id][2];

  }

  if(!name.empty()){
    _vmeshid.push_back(name);
  }else{
    _vmeshid.push_back("point_cloud" + std::to_string(id));
  }
//  std::cout << "[vpPclPointCloudVisualization ::addSurface] Added ID " << _vmeshid[id] << " to the list of known point clouds" << std::endl << std::flush;
  if(gp_viewer){
    gp_viewer->addPointCloud(_vPointClouds[id], _vmeshid[id]);
  }

  nbSurfaces = (nbSurfaces + 1) % gc_nbColorMax;

  _vpmutex.push_back(new std::mutex());
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
    gp_viewer->addText(newLegend._text, newLegend._posX, newLegend._posY, newLegend._rRatio, newLegend._gRatio, newLegend._bRatio );
  }

  return id;
}

unsigned int vpPclPointCloudVisualization ::addSurface(const pclPointCloud::Ptr &surface, const vpColVector &weights, std::string name)
{
  static unsigned int nbSurfaces = 0;
  unsigned int id = _vPointClouds.size();

  pclPointCloud::Ptr p_pointCloud(new pclPointCloud());
  _vPointClouds.push_back(p_pointCloud);

  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

  vpColorBlindFriendlyPalette color(gcolor[nbSurfaces]);
  std::vector<unsigned char> v_RGB = color.to_RGB();
  std::vector<double> v_RGBdouble = {
    (double)v_RGB[0],
    (double)v_RGB[1],
    (double)v_RGB[2]
  };
  g_vhandler.push_back(v_RGBdouble);

  for(unsigned int index = 0; index < nbPoints; index++){
    if(weights[index] > 0.95){
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

  if(!name.empty()){
    _vmeshid.push_back(name);
  }else{
    _vmeshid.push_back("point_cloud" + std::to_string(id));
  }
//  std::cout << "[vpPclPointCloudVisualization ::addSurface] Added ID " << _vmeshid[id] << " to the list of known point clouds" << std::endl;
  if(gp_viewer){
    gp_viewer->addPointCloud(_vPointClouds[id], _vmeshid[id]);
  }

  nbSurfaces = (nbSurfaces + 1) % gc_nbColorMax;

  _vpmutex.push_back(new std::mutex());

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
    gp_viewer->addText(newLegend._text, newLegend._posX, newLegend._posY, newLegend._rRatio, newLegend._gRatio, newLegend._bRatio );
  }

  return id;
}



void vpPclPointCloudVisualization ::display()
{
  stopThread();
  if(!gp_viewer){
    gp_viewer.reset(new pcl::visualization::PCLVisualizer(_title));
    gp_viewer->setSize(800,600);
    gp_viewer->addCoordinateSystem (0.5);
  }
  gp_viewer->spin();
}

void vpPclPointCloudVisualization ::refresh()
{
  gp_viewer->spinOnce(100,true);
  // _viewer.updateCamera();
}

void vpPclPointCloudVisualization ::launchThread()
{
  if(!_hasToRun){
    _hasToRun = true;
    _threadDisplay = std::thread(vpPclPointCloudVisualization ::runThread, this);
  }
}

void vpPclPointCloudVisualization ::stopThread()
{
  if(_hasToRun){
    _hasToRun = false;
    _threadDisplay.join();
  }
}

void vpPclPointCloudVisualization ::runThread(vpPclPointCloudVisualization  *p_visualizer)
{
  p_visualizer->loopThread();
}

void vpPclPointCloudVisualization::coyLegendParams(const vpPclPointCloudVisualization::legendParams &from, vpPclPointCloudVisualization::legendParams &to)
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
  bool useWeights;
  gp_viewer.reset(new pcl::visualization::PCLVisualizer(_title));
  gp_viewer->addCoordinateSystem (0.5);
  gp_viewer->initCameraParameters();
  gp_viewer->setSize(s_width,s_height);
  gp_viewer->setPosition(s_px, s_py);
  unsigned int iter;

  while(_hasToRun){
    for(unsigned int id=0; id< _vmeshid.size(); id++){
      unsigned int nbWeights = _weights.size();
      useWeights = (nbWeights > 0);
      unsigned int nbPoints = _vPointClouds[id]->size();

      _vpmutex[id]->lock();
      if(useWeights){
        _mutexWeights.lock();
        for(unsigned int index = 0; index < nbPoints; index++){
            if(_weights[index] < 0.95){
              _vPointClouds[id]->at(index).r = 0.0;
              _vPointClouds[id]->at(index).g = 0.0;
              _vPointClouds[id]->at(index).b = 0.0;
            }

        }
        _mutexWeights.unlock();
      }

      if(!gp_viewer->updatePointCloud(_vPointClouds[id], _vmeshid[id])){
        gp_viewer->addPointCloud(_vPointClouds[id], _vmeshid[id]);
        gp_viewer->addText(_vlegends[id]._text, _vlegends[id]._posX, _vlegends[id]._posY, _vlegends[id]._rRatio, _vlegends[id]._gRatio, _vlegends[id]._bRatio );
      }

      if(_vPointClouds[id]->size() > 0 && _hasToSavePCDs){
        std::string filename = vpIoTools::createFilePath(_outFolder, _vmeshid[id] + std::to_string(iter) + ".pcd" );
        pcl::io::savePCDFile(filename, *_vPointClouds[id]);
      }

      _vpmutex[id]->unlock();
    }
    gp_viewer->spinOnce(100,true);

    iter++;
  }
  gp_viewer.reset();
}

void vpPclPointCloudVisualization ::threadUpdateSurface(const pclPointCloud::Ptr &surface, unsigned int id)
{
  _vpmutex[id]->lock();
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

//  std::cout << "[vpPclPointCloudVisualization ::threadUpdateSurface] color = (" << g_vhandler[id][0] << ", " << g_vhandler[id][1] << ", "<< g_vhandler[id][2] << ")" << std::endl << std::flush;

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

  pcl::copyPointCloud(*surface, *_vPointClouds[id]);

  _vpmutex[id]->unlock();
}

void vpPclPointCloudVisualization ::threadUpdateSurface(const pclPointCloud::Ptr &surface, unsigned int id, const vpColVector &weights)
{
  _vpmutex[id]->lock();
  unsigned int nbPoints = surface->size();
  _vPointClouds[id]->resize(nbPoints);

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