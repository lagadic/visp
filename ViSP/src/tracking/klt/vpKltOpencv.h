/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker implemented
 * with opencv.
 *
 * Authors:
 * Fabien Servant
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \file vpKltOpencv.h

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with opencv.
*/

#ifndef vpKltOpencv_h
#define vpKltOpencv_h

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_OPENCV

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <ctype.h>
#endif

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplay.h>
#include <visp/vpDebug.h>
#include <visp/vpException.h>
#include <visp/vpTrackingException.h>

typedef int (*funccheck)(int,double,double);
typedef void (*funcinfo)(int,int,int,double,double);
typedef void (*funcevent)(int);

/*!
  \class vpKltOpencv

  \ingroup TrackingImagePoint

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with OpenCV.

*/
class VISP_EXPORT vpKltOpencv
{
 private:
  int initialized; //Is the tracker ready ?

  int maxFeatures; //Maximum number of features to track (Default 50)
  int countFeatures; //Currently tracked features
  int countPrevFeatures; //Previously tracked features
  int globalcountFeatures; //Keep over time for ID

  int flags; //Flags for tracking (internal)

  int win_size; //Size of search window for tracker (default 10)
  double quality; //Multiplier for the maxmin eigenvalue; specifies minimal accepted quality of image corners (default 0.01)
  double min_distance; //Limit, specifying minimum possible distance between returned corners; Euclidian distance is used. (default 10)
  double harris_free_parameter; //Harris detector free parameter. (default 0.04)
  int block_size; //Size of the averaging block used by the corner detector (default 3)
  int use_harris; //0 use a simple Minimum EigenValue Detector, != 0  use Harris (default 1)
  int pyramid_level; //Number of level for the tracker's gaussian pyramid data (default 3)
  int _tid; //tracker id for multiple trackers

  IplImage *image; //Image buffer
  IplImage *prev_image; //Image buffer for the previous iteration
  IplImage *pyramid; //Gaussian pyramid data
  IplImage *prev_pyramid; //Gaussian pyramid data for the previous iteration
  IplImage *swap_temp; //Internal

  CvPoint2D32f *features; //List of features
  CvPoint2D32f *prev_features; //List of features for previous iteration
  
  long *featuresid; //Array of ids for features
  long *prev_featuresid; //Array of ids for previous features
  bool *lostDuringTrack; // Result of the tracker for every feature : 1 = lost, 0 = still present
  char *status; //Result of the tracker for every features : 0 = lost, 1 = found



  //EVENT FUNCTION POINTERS
  funcevent OnInitialize;
  funcinfo OnFeatureLost;
  funcinfo OnNewFeature;
  funcinfo OnMeasureFeature;
  funccheck IsFeatureValid;

 private:

  //Internal
  void clean();
  void cleanAll();
  void reset();

 public:
  vpKltOpencv();
  vpKltOpencv(const vpKltOpencv& copy);
  virtual ~vpKltOpencv();

  //Detect corners in the image. Initialize the tracker
  void initTracking(const IplImage *I, const IplImage *mask = NULL);

  //Track !
  void track(const IplImage *I);

  //Draw the tracked features on the given image
  void display(const vpImage<unsigned char> &I,
	       vpColor color = vpColor::red);

  //Seters
  /* Should be used only before initTracking */
  void setMaxFeatures(unsigned int input);

  void setWindowSize(unsigned int input) {initialized = 0; win_size=input;}
  void setQuality(double input) {initialized = 0; quality=input;}
  void setMinDistance(double input) {initialized = 0; min_distance=input;}
  void setHarrisFreeParameter(double input) {initialized = 0; harris_free_parameter=input;}
  void setBlockSize(unsigned int input) {initialized = 0; block_size=input;}
  void setUseHarris(unsigned int input) {initialized = 0; use_harris=input;}
  void setPyramidLevels(unsigned int input) {initialized = 0; pyramid_level=input;}
  void setTrackerId(int tid) {_tid = tid;}

  //Functors

  //Event when tracker is initialized -> event(id_tracker)
  void setOnInitialize(funcevent input) {OnInitialize = input;}
  //Event when a feature is lost -> event(id_tracker, index, uid, x, y)
  void setOnFeatureLost(funcinfo input) {OnFeatureLost = input;}
  //Event when a new feature is found -> event(id_tracker, index, uid, x, y)
  void setOnNewFeature(funcinfo input) {OnNewFeature = input;}
  //Event when a feature is found while tracking -> event(id_tracker, index, uid, x, y)
  void setOnMeasureFeature(funcinfo input) {OnMeasureFeature = input;}
  //Is a feature valid (e.g. : test if not too close to borders) -> event(id_tracker, x, y)
  void setIsFeatureValid(funccheck input) {IsFeatureValid = input;}

  //! Get the list of lost feature
  bool *getListOfLostFeature(){ return lostDuringTrack; }
  //! Get the current number of features
  int getNbFeatures() { return countFeatures; }
  //! Get the previous number of features
  int getNbPrevFeatures() { return countPrevFeatures; }
  //! Get Max number of features
  int getMaxFeatures() {return maxFeatures;}

  void getFeature(int index, int &id, float &x, float &y);
  void getPrevFeature(int index, int &id, float &x, float &y);
  void addFeature(const int &id, const float &x, const float &y);
  void suppressFeature(int index);
};

#endif
#endif
