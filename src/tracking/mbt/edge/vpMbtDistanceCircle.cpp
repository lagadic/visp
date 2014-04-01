/****************************************************************************
 *
 * $Id: vpMbtDistanceCircle.cpp 4649 2014-02-07 14:57:11Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Description:
 * Make the complete tracking of an object by using its CAD model. Circle
 * tracking.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

/*!
 \file vpMbtDistanceCircle.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/

#include <stdlib.h>
#include <algorithm>

#include <visp/vpMbtDistanceCircle.h>
#include <visp/vpPlane.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureEllipse.h>
#include <visp/vpPose.h>

/*!
  Basic constructor
*/
vpMbtDistanceCircle::vpMbtDistanceCircle()
  : name(), index(0), cam(), me(NULL), wmean(1),
    featureEllipse(), meEllipse(NULL),
    circle(NULL), L(), error(), nbFeature(0), Reinit(false)
{
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbtDistanceCircle::~vpMbtDistanceCircle()
{
  if (meEllipse != NULL) delete meEllipse;
  if (circle != NULL) delete circle;
  if (p1 != NULL) delete p1;
  if (p2 != NULL) delete p2;
  if (p3 != NULL) delete p3;
}

/*!
  Project the circle into the image.
  
  \param cMo : The pose of the camera used to project the circle into the image.
*/
void
vpMbtDistanceCircle::project(const vpHomogeneousMatrix &cMo)
{
  circle->project(cMo) ;
}


/*!
  Build a vpMbtDistanceCircle thanks to its center, 3 points (including the center) with
  coordinates expressed in the object frame and defining the plane that contain
  the circle and its radius.

  \param _p1 : Center of the circle.
  \param _p2,_p3 : Two points on the plane containing the circle. With the center of the circle we have 3 points
  defining the plane that contains the circle.
  \param r : Radius of the circle.
*/
void
vpMbtDistanceCircle::buildFrom(const vpPoint &_p1, const vpPoint &_p2, const vpPoint &_p3, const double r)
{
  circle = new vpCircle ;
  p1 = new vpPoint ;
  p2 = new vpPoint ;
  p3 = new vpPoint ;

  // Get the points
  *p1 = _p1;
  *p2 = _p2;
  *p3 = _p3;

  // Get the radius
  radius = r;

  vpPlane plane(*p1, *p2, *p3, vpPlane::object_frame);

  // Build our circle
  circle->setWorldCoordinates(plane.getA(), plane.getB(), plane.getC(), _p1.get_oX(), _p1.get_oY(), _p1.get_oZ(), r);
}


/*! 
  Set the moving edge parameters.
  
  \param _me : an instance of vpMe containing all the desired parameters
*/
void
vpMbtDistanceCircle::setMovingEdge(vpMe *_me)
{
  me = _me ;
  if (meEllipse != NULL)
  {
    meEllipse->setMe(me) ;
  }
}

/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the circle.
  
  \param I : The image.
  \param cMo : The pose of the camera used to initialize the moving edges.
*/
void
vpMbtDistanceCircle::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
	// Perspective projection
  circle->changeFrame(cMo);

  try{
    circle->projection();
  }
  catch(...){std::cout<<"Problem when projecting circle\n";}

  // Create the moving edges containers
  meEllipse = new vpMeEllipse;
  meEllipse->setMe(me) ;

  meEllipse->setDisplay(vpMeSite::RANGE_RESULT) ; // TODO only for debug
  meEllipse->setInitRange(me->getRange()); // TODO: check because set to zero for lines

  vpImagePoint ic; // center in pixels
  double a_p, b_p, e_p;
  vpMeterPixelConversion::convertEllipse(cam, *circle, ic, a_p, b_p, e_p);

  try
  {
    if (0) { // TODO: Fix Ne fonctionne pas !!!!!
      meEllipse->initTracking(I, ic, a_p, b_p, e_p, vpMath::rad(0), vpMath::rad(360));
    }
    else {
      double j1, i1;
      vpImagePoint iP11;

      j1 = i1 = 0 ;

      double incr = vpMath::rad(2) ; // angle increment

      std::vector<vpImagePoint> v_iP;
      double k = 0 ;
      while (k<vpMath::rad(360))
      {
        j1 = a_p *cos(k) ; // equation of an ellipse
        i1 = b_p *sin(k) ; // equation of an ellipse

        // (i1,j1) are the coordinates on the origin centered ellipse ;
        // a rotation by "e" and a translation by (xci,jc) are done
        // to get the coordinates of the point on the shifted ellipse
        iP11.set_j ( ic.get_j() + cos(e_p) *j1 - sin(e_p) *i1 );
        iP11.set_i ( ic.get_i() + sin(e_p) *j1 + cos(e_p) *i1 );

        v_iP.push_back(iP11);
        k += incr ;
      }
      vpImagePoint *tab_iP = new vpImagePoint[v_iP.size()];
      for (unsigned int i=0; i< v_iP.size(); i++) {
        tab_iP[i] = v_iP[i];
        vpDisplay::displayCross(I, v_iP[i], 5, vpColor::darkBlue); // TODO remove
      }
      vpDisplay::flush(I); // TODO remove
      meEllipse->initTracking(I, (unsigned int)v_iP.size(), tab_iP);
      delete [] tab_iP;

    }
  }
  catch(...)
  {
    vpTRACE("the circle can't be initialized");
  }
}

/*!
  Track the moving edges in the image.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceCircle::trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix & /*cMo*/)
{
  try 
  {
    meEllipse->track(I) ;
  }
  catch(...)
  {
    std::cout << "Track meEllipse failed" << std::endl;
    Reinit = true;
  }

  // Update the number of features
  nbFeature = (unsigned int)meEllipse->getMeList().size();
}


/*!
  Update the moving edges internal parameters.

  \warning : Not implemented.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceCircle::updateMovingEdge(const vpImage<unsigned char> &/*I*/, const vpHomogeneousMatrix &/*cMo*/)
{
  // FS: I don't see any good reason to implement this function.
  // In the case of a line, it was useful because we need to update the line extrimities in
  // the case of a motion along the line to avoid a shift. But here it seems not useful !
//  vpTRACE("not implemented");
}


/*!
  Reinitialize the circle if it is required.
  
  A circle is reinitialized if the ellipse do not match enough with the projected 3D circle.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceCircle::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
  if(meEllipse!= NULL)
    delete meEllipse;
  
  meEllipse = NULL;

  initMovingEdge(I,cMo);

  Reinit = false;
}


/*!
  Display the circle. The 3D circle is projected into the image as an ellipse.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
*/
void
vpMbtDistanceCircle::display(const vpImage<unsigned char>&I, const vpHomogeneousMatrix &cMo,
                             const vpCameraParameters &camera, const vpColor col, const unsigned int thickness)
{
  // Perspective projection
  circle->changeFrame(cMo);

  try{
    circle->projection();
  }
  catch(...){std::cout<<"Cannot project the circle";}

  vpImagePoint center;
  double a, b, e;
  vpMeterPixelConversion::convertEllipse(camera, *circle, center, a, b, e);

  // Display
  vpDisplay::displayEllipse(I, center, a, b, e, 0, vpMath::rad(360), col, thickness);
}

/*!
  Display the cylinder. The 3D cylinder is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
*/
void
vpMbtDistanceCircle::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                             const vpCameraParameters &camera, const vpColor col,
                             const unsigned int thickness)
{
  // Perspective projection
  circle->changeFrame(cMo);

  try{
    circle->projection();
  }
  catch(...){std::cout<<"Cannot project the circle";}

  vpImagePoint center;
  double a, b, e;
  vpMeterPixelConversion::convertEllipse(camera, *circle, center, a, b, e);

  // Display
  vpDisplay::displayEllipse(I, center, a, b, e, 0, vpMath::rad(360), col, thickness);
}


/*!
    Enable to display the points along the ellipse with a color corresponding to their state.
    
    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If blue : The point is removed because of the robust method in the virtual visual servoing.
    
    \param I : The image.
*/
void
vpMbtDistanceCircle::displayMovingEdges(const vpImage<unsigned char> &I)
{
  if (meEllipse != NULL)
  {
    //meEllipse->display(I, vpColor::green); // display the ellipse
    meEllipse->display(I); // display the me
  }
}

/*!
  Initialize the size of the interaction matrix and the error vector.
*/
void
vpMbtDistanceCircle::initInteractionMatrixError()
{
    nbFeature = (unsigned int)meEllipse->getMeList().size();
    L.resize(nbFeature, 6);
    error.resize(nbFeature);
}

/*!
  Compute the interaction matrix and the error vector corresponding to the circle.
*/
void
vpMbtDistanceCircle::computeInteractionMatrixError(const vpHomogeneousMatrix &cMo)
{
	// Perspective projection
  circle->changeFrame(cMo) ;
  try{
    circle->projection();
  }
  catch(...){std::cout<<"Problem projection circle\n";}

  vpFeatureBuilder::create(featureEllipse, cam, *meEllipse);

  featureEllipse.setABC(circle->getA(), circle->getB(), circle->getC());
  vpMatrix H1 = featureEllipse.interaction();

  vpRowVector H(5);
  double x=0, y=0;

  // Get the parameters of the ellipse in the image plane
  double xg = circle->p[0];
  double yg = circle->p[1];
  double mu20 = circle->p[2];
  double mu11 = circle->p[3];
  double mu02 = circle->p[4];

  unsigned int j = 0;

  for(std::list<vpMeSite>::const_iterator it=meEllipse->getMeList().begin(); it!=meEllipse->getMeList().end(); ++it){
    vpPixelMeterConversion::convertPoint(cam, it->j, it->i, x, y);
    H[0] = 2*(mu11*(y-yg)+mu02*(xg-x));
    H[1] = 2*(mu20*(yg-y)+mu11*(x-xg));
    H[2] = vpMath::sqr(y-yg)-mu02;
    //H[3] = 2*(yg*(x-xg)+y*xg+mu11-x*y);
    H[3] = 2*(yg*(x+xg)+y*xg+mu11);
    H[4] = vpMath::sqr(x-xg)-mu20;

    for (unsigned int k=0; k<6; k++)
      L[j][k] = H[0]*H1[0][k] + H[1]*H1[1][k] + H[2]*H1[2][k] + H[3]*H1[3][k]  + H[4]*H1[4][k];

    error[j] = mu02*vpMath::sqr(x) + mu20*vpMath::sqr(y) - 2*mu11*x*y
        + 2*(mu11*yg-mu02*xg)*x + 2*(mu11*xg-mu20*yg)*y
        + mu02*vpMath::sqr(xg) + mu20*vpMath::sqr(yg) - 2*mu11*xg*yg
        + vpMath::sqr(mu11) - mu20*mu02;

    j++;
  }
}

