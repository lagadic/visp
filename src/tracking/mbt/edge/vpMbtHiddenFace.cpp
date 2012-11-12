/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
#include <visp/vpConfig.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*!
 \file vpMbtHiddenFace.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/

#include <visp/vpMbtHiddenFace.h>

/*!
  Basic constructor.
*/
vpMbtPolygon::vpMbtPolygon()
{
  nbpt = 0 ;
  p = NULL ;
  isappearing = false;
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
  
  \return Return true if the polygon is visible.
*/
bool
vpMbtPolygon::isVisible(const vpHomogeneousMatrix &cMo)
{
  changeFrame(cMo) ;
  
  for (unsigned int i = 0 ; i < nbpt ; i++){
    if(p[i].get_Z() < 0){
      isappearing = false;
      isvisible = false ;
      return false ;
    }
  }
  
  if(nbpt <= 2){
    /* a line is allways visible */
    isvisible = true;
    isappearing = false;
    return  true ;
  }

  vpColVector e1(3) ;
  vpColVector e2(3) ;
  vpColVector facenormal(3) ;

  e1[0] = p[1].get_X() - p[0].get_X() ;
  e1[1] = p[1].get_Y() - p[0].get_Y() ;
  e1[2] = p[1].get_Z() - p[0].get_Z() ;

  e2[0] = p[2].get_X() - p[1].get_X() ;
  e2[1] = p[2].get_Y() - p[1].get_Y() ;
  e2[2] = p[2].get_Z() - p[1].get_Z() ;

  facenormal = vpColVector::crossProd(e1,e2) ;

  double angle = p[0].get_X()*facenormal[0] +  p[0].get_Y()*facenormal[1]  +  p[0].get_Z()*facenormal[2]  ;
  
  if(angle < -0.0001 )
  {
    isvisible = true;
    isappearing = false;
    
    return  true ;
  }
  else
  {
    if (angle < 0.0000001 ){
      isappearing = true;
    }
    else {
      isappearing = false;
    }
    isvisible = false ;
    return false ;
  }
}

/*!
  Check if the polygon is visible in the image and if the angle between the normal 
  to the face and the normal to the camera plane is below the given threshold. 
  To do that, the polygon is projected into the image thanks to the camera pose.
  
  \param cMo : The pose of the camera.
  \param alpha : maximum angle to detect if the face is visible (in rad)
  
  \return Return true if the polygon is visible.
*/
bool 
vpMbtPolygon::isVisible(const vpHomogeneousMatrix &cMo, const double alpha)
{
  //   std::cout << "Computing angle from MBT Face (cMo, alpha)" << std::endl;
  if(nbpt <= 2){
    /* a line is allways visible */
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
  
  double angle2 = vpColVector::dotProd (e4, facenormal);
  double my_angle = acos(angle2);
  
//   std::cout << angle2 << "/" << vpMath::deg(my_angle) << std::endl;
  
  if( my_angle < alpha ){
    isvisible = true;
    isappearing = false;
    return true;
  }
  
  isvisible = false;
  isappearing = false;
  return false;
}


/*!
  Basic constructor.
*/
vpMbtHiddenFaces::vpMbtHiddenFaces()
{}


/*!
  Basic destructor.
*/
vpMbtHiddenFaces::~vpMbtHiddenFaces()
{
//  cout << "Deleting Hidden Face struxture "<<endl  ;
  vpMbtPolygon *p ;

  for(std::list<vpMbtPolygon*>::const_iterator it=Lpol.begin(); it!=Lpol.end(); ++it){
    p = *it;
    if (p!=NULL){
      delete p ;
    }
    p = NULL ;
  }

  Lpol.clear() ;
}

/*!
  Add a polygon to the list of polygons.
  
  \param p : The polygon to add.
*/
void
vpMbtHiddenFaces::addPolygon(vpMbtPolygon *p)
{
  vpMbtPolygon *p_new = new vpMbtPolygon;
  p_new->index = p->index;
  p_new->setNbPoint(p->nbpt);
  p_new->isvisible = p->isvisible;
  for(unsigned int i = 0; i < p->nbpt; i++)
    p_new->p[i]= p->p[i];
  Lpol.push_back(p_new);
}


/*!
  Compute the number of visible polygons.
  
  \param cMo : The pose of the camera
  
  \return Return the number of visible polygons
*/
unsigned int
vpMbtHiddenFaces::setVisible(const vpHomogeneousMatrix &cMo)
{
  unsigned int nbvisiblepolygone = 0 ;
  vpMbtPolygon *p ;

  unsigned int indice = 0;
  for(std::list<vpMbtPolygon*>::const_iterator it=Lpol.begin(); it!=Lpol.end(); ++it){
    p = *it;
    if (p->isVisible(cMo)){
      nbvisiblepolygone++;
    }
    
    indice++;
  }
  return nbvisiblepolygone ;
}

/*!
  Check if the polygon with the index \f$ index \f$ is visible or not.
  
  \param index : The index of one polygon in the list of polygons.
  
  \return Return true if the polygon is visible.
*/
bool
vpMbtHiddenFaces::isVisible(const int index)
{
  vpMbtPolygon *p ;
  for(std::list<vpMbtPolygon*>::const_iterator it=Lpol.begin(); it!=Lpol.end(); ++it){
    p = *it;
    if (p->getIndex() == index) return p->isVisible() ;
  }
  return false ;
}

/*!
  Check if the polygon with the index \f$ index \f$ will appear soon.
  
  \param index : The index of one polygon in the list of polygons.
  
  \return Return true if the polygon will appear soon.
*/
bool
vpMbtHiddenFaces::isAppearing(const int index)
{
  vpMbtPolygon *p ;
  for(std::list<vpMbtPolygon*>::const_iterator it=Lpol.begin(); it!=Lpol.end(); ++it){
    p = *it;
    if (p->getIndex() == index) return p->isAppearing() ;
  }
  return false ;
}

/*!
  Reset the Hidden faces (remove the list of vpMbtPolygon)
*/
void
vpMbtHiddenFaces::reset()
{
  vpMbtPolygon *p ;

  for(std::list<vpMbtPolygon*>::const_iterator it=Lpol.begin(); it!=Lpol.end(); ++it){
    p = *it;
    if (p!=NULL){
      delete p ;
    }
    p = NULL ;
  }

  Lpol.clear();
}

#endif

