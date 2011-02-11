/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
  \file vpKltOpencv.cpp

  \brief Wrapper for the KLT (Kanade-Lucas-Tomasi) feature tracker
  implemented with opencv.
*/

#include "visp/vpConfig.h"

#ifdef VISP_HAVE_OPENCV

#include <string>

#include "vpKltOpencv.h"

void vpKltOpencv::clean()
{
  if (image) cvReleaseImage(&image);
  if (prev_image) cvReleaseImage(&prev_image);
  if (pyramid) cvReleaseImage(&pyramid);
  if (prev_pyramid) cvReleaseImage(&prev_pyramid);
  
  image = 0;
  prev_image = 0;
  pyramid = 0;
  prev_pyramid = 0;
  
  swap_temp = 0;
  countFeatures = 0;
  countPrevFeatures = 0;
  flags = 0;
  initialized = 0;
  globalcountFeatures = 0;
}

void vpKltOpencv::cleanAll()
{
  clean();

  if (features) cvFree(&features);
  if (prev_features) cvFree(&prev_features);
  if (status) cvFree(&status);
  if (lostDuringTrack) cvFree(&lostDuringTrack);
  if (featuresid) cvFree(&featuresid);
  if (prev_featuresid) cvFree(&prev_featuresid);

  features = 0;
  prev_features = 0;
  status = 0;
  lostDuringTrack = 0;
  featuresid = 0;
  prev_featuresid = 0;
}

void vpKltOpencv::reset()
{
  clean();

}

vpKltOpencv::vpKltOpencv()
{
  //Valeurs par d�faut pour le KLT
  initialized = 0;
  maxFeatures = 50;
  countFeatures = 0;
  countPrevFeatures = 0;
  globalcountFeatures = 0;
  win_size = 10;
  quality = 0.01;
  min_distance = 10;
  block_size = 3;
  use_harris = 1;
  pyramid_level = 3;
  harris_free_parameter = 0.04;

  //Zeroing pointers
  image = 0;
  prev_image = 0;
  pyramid = 0;
  prev_pyramid = 0;
  swap_temp = 0;
  features = 0;
  prev_features = 0;
  flags = 0;
  status = 0;
  lostDuringTrack = 0;
  featuresid = 0;
  prev_featuresid = 0;
  OnInitialize = 0;
  OnFeatureLost = 0;
  OnNewFeature = 0;
  OnMeasureFeature = 0;
  IsFeatureValid = 0;

  features = (CvPoint2D32f*)cvAlloc((unsigned int)maxFeatures*sizeof(features[0]));
  prev_features = (CvPoint2D32f*)cvAlloc((unsigned int)maxFeatures*sizeof(prev_features[0]));
  status = (char*)cvAlloc((size_t)maxFeatures);
  lostDuringTrack = (bool*)cvAlloc((size_t)maxFeatures);
  featuresid = (long*)cvAlloc((unsigned int)maxFeatures*sizeof(long));
  prev_featuresid = (long*)cvAlloc((unsigned int)maxFeatures*sizeof(long));


  _tid = -1;
}

vpKltOpencv::vpKltOpencv(const vpKltOpencv& copy)
{
  //Shallow copy of primitives
  initialized = copy.initialized;
  maxFeatures = copy.maxFeatures;
  countFeatures = copy.countFeatures;
  countPrevFeatures = copy.countPrevFeatures;
  globalcountFeatures = copy.globalcountFeatures;
  flags = copy.flags;
  win_size = copy.win_size;
  quality = copy.quality;
  min_distance = copy.min_distance;
  harris_free_parameter = copy.harris_free_parameter;
  block_size = copy.block_size;
  use_harris = copy.use_harris;
  pyramid_level = copy.pyramid_level;
  _tid = copy._tid;

  OnInitialize = copy.OnInitialize;
  OnFeatureLost = copy.OnFeatureLost;
  OnNewFeature = copy.OnNewFeature;
  OnMeasureFeature = copy.OnMeasureFeature;
  IsFeatureValid = copy.IsFeatureValid;

  if (!initialized) {
    image = 0;
    prev_image = 0;
    pyramid = 0;
    prev_pyramid = 0;
    features = 0;
    prev_features = 0;
    status = 0;
    lostDuringTrack = 0;
    featuresid = 0;
    prev_featuresid = 0;
    swap_temp = 0;
    countFeatures = 0;
    countPrevFeatures = 0;
    flags = 0;
    initialized = 0;
    globalcountFeatures = 0;
  }

  	if (copy.image)
  	{
           image =  cvCreateImage(cvGetSize(copy.image), 8, 1);
	   //		/*IplImage **/cvCopyImage(copy.image,image);
	   cvCopy(copy.image, image, 0);
  	}

  	if (copy.prev_image)
  	{
          prev_image = cvCreateImage(cvGetSize(copy.prev_image), IPL_DEPTH_8U, 1);
	  //	/*IplImage **/ cvCopyImage(copy.prev_image,prev_image);
	  cvCopy(copy.prev_image, prev_image, 0);
  	}

  	if (copy.pyramid)
  	{
          pyramid = cvCreateImage(cvGetSize(copy.pyramid), IPL_DEPTH_8U, 1);
	  // /*IplImage **/cvCopyImage(copy.pyramid,pyramid);
	  cvCopy(copy.pyramid, pyramid, 0);
  	}

  	if (copy.prev_pyramid)
  	{
          prev_pyramid = cvCreateImage(cvGetSize(copy.prev_pyramid), IPL_DEPTH_8U, 1);
	  //	/*IplImage **/cvCopyImage(copy.prev_pyramid,prev_pyramid);
	  cvCopy(copy.prev_pyramid, prev_pyramid, 0);
  	}

  //Deep copy of arrays
  if (copy.features) {
      /*CvPoint2D32f **/features =
	(CvPoint2D32f*)cvAlloc((unsigned int)copy.maxFeatures*sizeof(CvPoint2D32f));
      for (int i = 0; i < copy.maxFeatures; i++)
	features[i] = copy.features[i];
    }

  if (copy.prev_features) {
      /*CvPoint2D32f **/prev_features =
	(CvPoint2D32f*)cvAlloc((unsigned int)copy.maxFeatures*sizeof(CvPoint2D32f));
      for (int i = 0; i < copy.maxFeatures; i++)
	prev_features[i] = copy.prev_features[i];
    }

  if (copy.featuresid) {
      /*long **/featuresid = (long*)cvAlloc((unsigned int)copy.maxFeatures*sizeof(long));
      for (int i = 0; i < copy.maxFeatures; i++)
	featuresid[i] = copy.featuresid[i];
    }

  if (copy.prev_featuresid) {
      /*long **/prev_featuresid = (long*)cvAlloc((unsigned int)copy.maxFeatures*sizeof(long));
      for (int i = 0; i < copy.maxFeatures; i++)
	prev_featuresid[i] = copy.prev_featuresid[i];
    }

  if (copy.status) {
      /*char **/status = (char*)cvAlloc((unsigned int)copy.maxFeatures*sizeof(char));
      for (int i = 0; i < copy.maxFeatures; i++)
	status[i] = copy.status[i];
    }

  if (copy.lostDuringTrack) {
    /*bool **/lostDuringTrack = (bool*)cvAlloc((unsigned int)copy.maxFeatures*sizeof(bool));
    for (int i = 0; i < copy.maxFeatures; i++)
      lostDuringTrack[i] = copy.lostDuringTrack[i];
  }
}

vpKltOpencv::~vpKltOpencv()
{
  cleanAll();

}

void vpKltOpencv::setMaxFeatures(const int input) {
  initialized = 0; maxFeatures=input;

  if (features) cvFree(&features);
  if (prev_features) cvFree(&prev_features);
  if (status) cvFree(&status);
  if (lostDuringTrack) cvFree(&lostDuringTrack);
  if (featuresid) cvFree(&featuresid);
  if (prev_featuresid) cvFree(&prev_featuresid);
 

  features = (CvPoint2D32f*)cvAlloc((unsigned int)maxFeatures*sizeof(CvPoint2D32f));
  prev_features = (CvPoint2D32f*)cvAlloc((unsigned int)maxFeatures*sizeof(CvPoint2D32f));
  status = (char*)cvAlloc((unsigned int)maxFeatures*sizeof(char));
  lostDuringTrack = (bool*)cvAlloc((unsigned int)maxFeatures*sizeof(bool));
  featuresid = (long*)cvAlloc((unsigned int)maxFeatures*sizeof(long));
  prev_featuresid = (long*)cvAlloc((unsigned int)maxFeatures*sizeof(long));

  
}

void vpKltOpencv::initTracking(const IplImage *I, const IplImage *masque)
{
  if (!I) {
    throw(vpException(vpTrackingException::initializationError,  "Image Not initialized")) ;
  }

  if (I->depth != IPL_DEPTH_8U || I->nChannels != 1)  {
    throw(vpException(vpTrackingException::initializationError,  "Bad Image format")) ;
  }

  if (masque) {
    if (masque->depth != IPL_DEPTH_8U || I->nChannels != 1) 	{
      throw(vpException(vpTrackingException::initializationError,  "Bad Image format")) ;
    }
  }


  //Creation des buffers
  CvSize Sizeim, SizeI;
  SizeI = cvGetSize(I);
  if(image != NULL)Sizeim = cvGetSize(image);
  if(image == NULL ||prev_image == NULL || pyramid==NULL || prev_pyramid ==NULL ||
     SizeI.width != Sizeim.width || SizeI.height != Sizeim.height){
    reset();
    image = cvCreateImage(cvGetSize(I), 8, 1);image->origin = I->origin;
    prev_image = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
    pyramid = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
    prev_pyramid = cvCreateImage(cvGetSize(I), IPL_DEPTH_8U, 1);
  }
  else{
    swap_temp = 0;
    countFeatures = 0;
    countPrevFeatures = 0;
    flags = 0;
    initialized = 0;
    globalcountFeatures = 0;   
  }

  initialized = 1;

  //Import
  cvCopy(I, image, 0);

  //Recherche de points d'int�rets
  countFeatures = maxFeatures;
  countPrevFeatures = 0;
  IplImage* eig = cvCreateImage(cvGetSize(image), 32, 1);
  IplImage* temp = cvCreateImage(cvGetSize(image), 32, 1);
  cvGoodFeaturesToTrack(image, eig, temp, features,
			&countFeatures, quality, min_distance,
			masque, block_size, use_harris, harris_free_parameter);
  cvFindCornerSubPix(image, features, countFeatures, cvSize(win_size, win_size),
		     cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER|
						  CV_TERMCRIT_EPS,20,0.03));
  cvReleaseImage(&eig);
  cvReleaseImage(&temp);

  if (OnInitialize)
    OnInitialize(_tid);

  //printf("Number of features at init: %d\n", countFeatures);
  for (int boucle=0; boucle<countFeatures;boucle++)  {
    featuresid[boucle] = globalcountFeatures;
    globalcountFeatures++;
    
    if (OnNewFeature){
      OnNewFeature(_tid, boucle, featuresid[boucle], features[boucle].x,
		   features[boucle].y);
    }
  }
}
 

void vpKltOpencv::track(const IplImage *I)
{
  if (!initialized) {
    vpERROR_TRACE("KLT Not initialized") ;
    throw(vpException(vpTrackingException::initializationError,
		      "KLT Not initialized")) ;
  }

  if (!I) {
    throw(vpException(vpTrackingException::initializationError,
		      "Image Not initialized")) ;
  }

  if (I->depth != IPL_DEPTH_8U || I->nChannels != 1)  {
    throw(vpException(vpTrackingException::initializationError,
		      "Bad Image format")) ;
  }

  // Save current features as previous features
  countPrevFeatures = countFeatures;
  for (int boucle=0; boucle<countFeatures;boucle++)  {
    prev_featuresid[boucle] = featuresid[boucle];
  }

  CvPoint2D32f *swap_features = 0;

  CV_SWAP(prev_image, image, swap_temp);
  CV_SWAP(prev_pyramid, pyramid, swap_temp);
  CV_SWAP(prev_features, features, swap_features);

  cvCopy(I, image, 0);

  if (countFeatures <= 0) return;


  cvCalcOpticalFlowPyrLK( prev_image, image, prev_pyramid, pyramid,
			  prev_features, features, countFeatures,
			  cvSize(win_size, win_size), pyramid_level,
			  status, 0, cvTermCriteria(CV_TERMCRIT_ITER
						    |CV_TERMCRIT_EPS,20,0.03),
			  flags );
  flags |= CV_LKFLOW_PYR_A_READY;

  int i,k;
  for (i = k = 0; i < countFeatures ; i++)  {
    if (!status[i]) 	{
      lostDuringTrack[i] = 1;
      if (OnFeatureLost)
	OnFeatureLost(_tid, i, featuresid[i], features[i].x,
		      features[i].y);
      continue;
    }
    
    if (IsFeatureValid)	{
      if (!IsFeatureValid(_tid, features[i].x, features[i].y))   {
	lostDuringTrack[i] = 1;
	if (OnFeatureLost)
	  OnFeatureLost(_tid, i, featuresid[i], features[i].x, features[i].y);
	continue;
      }
    }
    features[k] = features[i];
    featuresid[k] = featuresid[i];

    if (OnMeasureFeature) OnMeasureFeature(_tid, k, featuresid[k], features[k].x, features[k].y);
    
    lostDuringTrack[i] = 0;
    k++;
  }
  countFeatures = k;
}

void vpKltOpencv::display(const vpImage<unsigned char> &I,
			  vpColor color)
{
  if ((features == 0) || (I.bitmap==0) || (!initialized))
    {
      vpERROR_TRACE(" Memory problem ");
      throw(vpException(vpException::memoryAllocationError," Memory problem"));
    }

  vpImagePoint ip;
  for (int i = 0 ; i < countFeatures ; i++)
    {
      ip.set_u( vpMath::round(features[i].x ) );
      ip.set_v( vpMath::round(features[i].y ) );
      vpDisplay::displayCross(I, ip, 10, color) ;
      char id[10];
      sprintf(id, "%ld", featuresid[i]);
      
      ip.set_u( vpMath::round( features[i].x + 5 ) );
      vpDisplay::displayCharString(I, ip, id, vpColor::red);
    }
}

/*!

  Get the 'index'th feature image coordinates.  Beware that
  getFeature(i,...) may not represent the same feature before and
  after a tracking iteration (if a feature is lost, features are
  shifted in the array).

*/
void vpKltOpencv::getFeature(int index, int &id, float &x, float &y) const
{
  if (index >= countFeatures)
    {
      vpERROR_TRACE(" Memory problem ");
      throw(vpException(vpException::memoryAllocationError," Memory problem"));
    }

  x = features[index].x;
  y = features[index].y;
  id = featuresid[index];
}

/*!

  Get the 'index'th previous feature image coordinates.  Beware that
  getPrevFeature(i,...) may not represent the same feature before and
  after a tracking iteration (if a feature is lost, features are
  shifted in the array).

*/
void vpKltOpencv::getPrevFeature(int index, int &id, float &x, float &y) const
{
  if (index >= countPrevFeatures)
    {
      vpERROR_TRACE(" Memory problem ");
      throw(vpException(vpException::memoryAllocationError," Memory problem"));
    }

  x = prev_features[index].x;
  y = prev_features[index].y;
  id = prev_featuresid[index];
}

/*!

Add at the end of the feauture list.

If there is no space left, the feature is not added (just return)
*/
void vpKltOpencv::addFeature(const int &id,
			     const float &x, const float &y)
{
  if (maxFeatures == countFeatures)
    {
      vpERROR_TRACE(" Cannot add the feature ");
      return;
    }

  CvPoint2D32f f;
  f.x = x;
  f.y = y;
   features[countFeatures] = f;
  featuresid[countFeatures] = id;
  countFeatures ++;
}

void vpKltOpencv::suppressFeature(int index)
{
  if (index >= countFeatures)
    {
      vpERROR_TRACE(" Memory problem ");
      throw(vpException(vpException::memoryAllocationError," Memory problem"));
    }

  countFeatures --;

  for (int i=index ; i < countFeatures; i ++) {
    features[i] = features[i+1];
    featuresid[i] = featuresid[i+1];
  }
}
#endif
