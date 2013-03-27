/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 * Aurelien Yol
 *
 *****************************************************************************/

#include <limits.h>

#include <visp/vpConfig.h>
/*!
 \file vpMbtPolygon.cpp
 \brief Implements a polygon of the model used by the model-based tracker.
*/

#include <visp/vpMbtPolygon.h>

/*!
  Basic constructor.
*/
vpMbtPolygon::vpMbtPolygon()
{
  nbpt = 0 ;
  p = NULL ;
  isappearing = false;
  isvisible = false;
}

/*!
  Basic destructor.
*/
vpMbtPolygon::~vpMbtPolygon()
{
  if (p !=NULL)
  {
    delete[] p;
    p = NULL;
  }
}

/*!
  Get a reference to a corner.

  \throw vpException::dimensionError if the _index is out of range.

  \param _index : the index of the corner
*/
vpPoint &
vpMbtPolygon::getPoint(const unsigned int _index)
{
  if(_index >= nbpt){
    throw vpException(vpException::dimensionError, "index out of range");
  }
  return p[_index];
}

/*!
  Set the number of points which are the corners of the polygon.
  
  \param nb : The number of corners.
*/
void
vpMbtPolygon::setNbPoint(const unsigned int nb)
{
  nbpt = nb ;
  if (p != NULL)
    delete[] p;
  p = new vpPoint[nb] ;
}

/*!
  Add a corner point to the list of polygon's corners.
  
  \param n : The index of the corner.
  \param P : The point to add.
*/
void
vpMbtPolygon::addPoint(const unsigned int n, const vpPoint &P)
{
  //if( p!NULL && n < nbpt )
    p[n] = P ;
}

/*!
  Project the 3D corner points into the image thanks to the pose of the camera.
  
  \param cMo : The pose of the camera.
*/
void
vpMbtPolygon::changeFrame(const vpHomogeneousMatrix &cMo)
{
  for (unsigned int i = 0 ; i < nbpt ; i++)
  {
    p[i].changeFrame(cMo) ;
    p[i].projection() ;
  }
}

/*!
  Check if the polygon is visible in the image. To do that, the polygon is projected into the image thanks to the camera pose.
  
  \param cMo : The pose of the camera.
  \param depthTest : True if a face has to be entirely visible (in front of the camera). False if it can be partially visible.
  
  \return Return true if the polygon is visible.
*/
bool
vpMbtPolygon::isVisible(const vpHomogeneousMatrix &cMo, const bool &depthTest)
{
  changeFrame(cMo) ;
  
  if(depthTest)
    for (unsigned int i = 0 ; i < nbpt ; i++){
      if(p[i].get_Z() < 0){
        isappearing = false;
        isvisible = false ;
        return false ;
      }
    }
  
  return isVisible(cMo, vpMath::rad(89));
}

/*!
  Check if the polygon is visible in the image and if the angle between the normal 
  to the face and the line vector going from the optical center to the cog of the face is below
  the given threshold.
  To do that, the polygon is projected into the image thanks to the camera pose.
  
  \param cMo : The pose of the camera.
  \param alpha : Maximum angle to detect if the face is visible (in rad).
  \param modulo : Indicates if the test should also consider faces that are not oriented
  counter clockwise. If true, the orientation of the face is without importance.
  
  \return Return true if the polygon is visible.
*/
bool 
vpMbtPolygon::isVisible(const vpHomogeneousMatrix &cMo, const double alpha, const bool &modulo)
{
  //   std::cout << "Computing angle from MBT Face (cMo, alpha)" << std::endl;
  if(nbpt <= 2){
    /* a line is allways visible */
    isvisible = true;
    isappearing = false;
    return  true ;
  }

  changeFrame(cMo);

  vpColVector e1(3) ;
  vpColVector e2(3) ;
  vpColVector facenormal(3) ;

  e1[0] = p[1].get_X() - p[0].get_X() ;
  e1[1] = p[1].get_Y() - p[0].get_Y() ;
  e1[2] = p[1].get_Z() - p[0].get_Z() ;

  e2[0] = p[2].get_X() - p[1].get_X() ;
  e2[1] = p[2].get_Y() - p[1].get_Y() ;
  e2[2] = p[2].get_Z() - p[1].get_Z() ;
 
  e1.normalize();
  e2.normalize();
  
  facenormal = vpColVector::crossProd(e1,e2) ;
  facenormal.normalize();
 
  vpColVector e4(3) ;
  vpPoint pt;
  for (unsigned int i = 0; i < nbpt; i += 1){
    pt.set_X(pt.get_X() + p[i].get_X());
    pt.set_Y(pt.get_Y() + p[i].get_Y());
    pt.set_Z(pt.get_Z() + p[i].get_Z());
  }
  e4[0] = -pt.get_X()/(double)nbpt; e4[1] = -pt.get_Y()/(double)nbpt; e4[2] = -pt.get_Z()/(double)nbpt; 
  e4.normalize();
  
  double cos_angle = vpColVector::dotProd (e4, facenormal);
  double angle = acos(cos_angle);
  
//   std::cout << cos_angle << "/" << vpMath::deg(angle) << std::endl;
  
  if( angle < alpha ){
    isvisible = true;
    isappearing = false;
    return true;
  }

  if(modulo && (M_PI - angle) < alpha){
    isvisible = true;
    isappearing = false;
    return true;
  }

  if (angle < alpha+vpMath::rad(1) ){
    isappearing = true;
  }
  else if (modulo && (M_PI - angle) < alpha+vpMath::rad(1) ){
    isappearing = true;
  }
  else {
    isappearing = false;
  }
  isvisible = false ;
  return false ;
}

std::vector<vpImagePoint> 
vpMbtPolygon::getRoi(const vpCameraParameters &_cam)
{
  std::vector<vpImagePoint> roi;
  for (unsigned int i = 0; i < nbpt; i ++){
    vpImagePoint ip;
    vpMeterPixelConversion::convertPoint(_cam, p[i].get_x(), p[i].get_y(), ip);
    roi.push_back(ip);
  }
  return roi;
}

/*!
  Static method to check the number of points of a region defined by the vector of image point that are inside the image.

  \param I : The image used for its size.
  \param cam : The camera parameters.
*/
unsigned int 
vpMbtPolygon::getNbCornerInsideImage(const vpImage<unsigned char>& I, const vpCameraParameters &cam)
{
  unsigned int nbPolyIn = 0;
  for (unsigned int i = 0; i < nbpt; i ++){
    if(p[i].get_Z() > 0){
      vpImagePoint ip;
      vpMeterPixelConversion::convertPoint(cam, p[i].get_x(), p[i].get_y(), ip);
      if((ip.get_i() >= 0) && (ip.get_j() >= 0) && (ip.get_i() < I.getHeight()) && (ip.get_j() < I.getWidth()))
        nbPolyIn++;
    }
  }
  
  return nbPolyIn;
}

//###################################
//      Static functions
//###################################

void                
vpMbtPolygon::getMinMaxRoi(const std::vector<vpImagePoint> &roi, int & i_min, int &i_max, int &j_min, int &j_max)
{
  // i_min_d = std::numeric_limits<double>::max(); // create an error under Windows. To fix it we have to add #undef max
  double i_min_d = (double) INT_MAX;
  double i_max_d = 0;
  double j_min_d = (double) INT_MAX;
  double j_max_d = 0;

  for (unsigned int i = 0; i < roi.size(); i += 1){
    if(i_min_d > roi[i].get_i())
      i_min_d = roi[i].get_i();
    
    if(roi[i].get_i() < 0)
      i_min_d = 1;
    
    if((roi[i].get_i() > 0) && (i_max_d < roi[i].get_i()))
      i_max_d = roi[i].get_i();
    
    if(j_min_d > roi[i].get_j())
      j_min_d = roi[i].get_j();
    
    if(roi[i].get_j() < 0)
      j_min_d = 1;//border
      
    if((roi[i].get_j() > 0) && j_max_d < roi[i].get_j())
      j_max_d = roi[i].get_j();
  }
  i_min = static_cast<int> (i_min_d);
  i_max = static_cast<int> (i_max_d);
  j_min = static_cast<int> (j_min_d);
  j_max = static_cast<int> (j_max_d);
}

/*!
  Static method to check whether the region defined by the vector of image point
  is contained entirely in the image.

  \param I : The image used for its size.
  \param corners : The vector of points defining a region
*/
bool
vpMbtPolygon::roiInsideImage(const vpImage<unsigned char>& I, const std::vector<vpImagePoint>& corners)
{
  double nbPolyIn = 0;
  for(unsigned int i=0; i<corners.size(); ++i){
    if((corners[i].get_i() >= 0) && (corners[i].get_j() >= 0) &&
       (corners[i].get_i() < I.getHeight()) && (corners[i].get_j() < I.getWidth())){
      nbPolyIn++;
    }
  }
  
  if(nbPolyIn < 3 && nbPolyIn < 0.7 * corners.size())
    return false;
  
  return true;
}


