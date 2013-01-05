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
 * Description:
 * Make the complete tracking of an object by using its CAD model. Cylinder
 * tracking.
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 * Bertrand Delabarre
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

/*!
 \file vpMbtDistanceCylinder.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/

#include <visp/vpMbtDistanceCylinder.h>
#include <visp/vpPlane.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureEllipse.h>
#include <stdlib.h>
#include <algorithm>

#include <visp/vpPose.h>

/*!
  Basic constructor
*/
vpMbtDistanceCylinder::vpMbtDistanceCylinder()
{
  name = "";
  p1 = NULL ;
  p2 = NULL ;
  c = NULL ;
  meline1 = NULL ;
  meline2 = NULL ;
  wmean1 = 1 ;
  wmean2 = 1 ;
  nbFeaturel1 =0 ;
  nbFeaturel2 =0 ;
  nbFeature =0 ;
  Reinit = false;

  cercle1 = NULL;
  cercle2 = NULL;
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbtDistanceCylinder::~vpMbtDistanceCylinder()
{
//	cout << "Deleting cylinder " << index << endl ;

  if (p1 != NULL) delete p1 ;
  if (p2 != NULL) delete p2 ;
  if (c != NULL) delete c ;
  if (meline1 != NULL) delete meline1 ;
  if (meline2 != NULL) delete meline2 ;
  if (cercle1 != NULL) delete cercle1 ;
  if (cercle2 != NULL) delete cercle2 ;
}

/*!
  Project the cylinder and the two points corresponding to its extremities into the image.
  
  \param cMo : The pose of the camera used to project the cylinder into the image.
*/
void
vpMbtDistanceCylinder::project(const vpHomogeneousMatrix &cMo)
{
  c->project(cMo) ;
  p1->project(cMo) ;
  p2->project(cMo) ;
  cercle1->project(cMo) ;
  cercle2->project(cMo) ;
}


/*!
  Build a vpMbtDistanceCylinder thanks to two points corresponding to the extremities of its axis and its radius.
  
  \param _p1 : The first extremity on the axis.
  \param _p2 : The second extremity on the axis.
  \param r : Radius of the cylinder.
*/
void
vpMbtDistanceCylinder::buildFrom(const vpPoint &_p1, const vpPoint &_p2, const double r)
{
  c = new vpCylinder ;
  p1 = new vpPoint ;
  p2 = new vpPoint ;
  cercle1 = new vpCircle;
  cercle2 = new vpCircle;

  // Get the points
  *p1 = _p1;
  *p2 = _p2;

  // Get the radius
  radius = r;

  vpColVector ABC(3);
  vpColVector V1(3);
  vpColVector V2(3);

  V1[0] = _p1.get_oX();
  V1[1] = _p1.get_oY();
  V1[2] = _p1.get_oZ();
  V2[0] = _p2.get_oX();
  V2[1] = _p2.get_oY();
  V2[2] = _p2.get_oZ();

  // Get the axis of the cylinder
  ABC = V1-V2;

  // Build our extremity circles
  cercle1->setWorldCoordinates(ABC[0],ABC[1],ABC[2],_p1.get_oX(),_p1.get_oY(),_p1.get_oZ(),r);
  cercle2->setWorldCoordinates(ABC[0],ABC[1],ABC[2],_p2.get_oX(),_p2.get_oY(),_p2.get_oZ(),r);

  // Build our cylinder
  c->setWorldCoordinates(ABC[0],ABC[1],ABC[2],(_p1.get_oX()+_p2.get_oX())/2.0,(_p1.get_oY()+_p2.get_oY())/2.0,(_p1.get_oZ()+_p2.get_oZ())/2.0,r);
}


/*! 
  Set the moving edge parameters.
  
  \param _me : an instance of vpMe containing all the desired parameters
*/
void
vpMbtDistanceCylinder::setMovingEdge(vpMe *_me)
{
  me = _me ;
  if (meline1 != NULL)
  {
    meline1->setMe(me) ;
  }
  if (meline2 != NULL)
  {
    meline2->setMe(me) ;
  }
}

/*!
  Computes the coordinates of the point corresponding to the intersection
  between a circle and a line from a cylinder's limbo plane.
  
  \param i : A reference to the i coordinate of the point.
  \param j : A reference to the j coordinate of the point.
  \param rho : The rho parameter resulting from the projection of the limbo line considered.
  \param theta : The theta parameter resulting from the projection of the limbo line considered.
  \param circle : A pointer to the circle we consider.
*/
void
vpMbtDistanceCylinder::getCylinderLineExtremity(double &i, double &j,double rho, double theta,
				      vpCircle *circle)
{
// This was taken from the code of art-v1. (from the artCylinder class)
  double px = cam.get_px() ;
  double py = cam.get_py() ;
  double u0 = cam.get_u0() ;
  double v0 = cam.get_v0() ;

  double mu11 = circle->p[3];
  double mu02 = circle->p[4];
  double mu20 = circle->p[2];
  double Xg = u0 + circle->p[0]*px;
  double Yg = v0 + circle->p[1]*py;

  // Find Intersection between line and ellipse in the image.

  // Optimised calculation for X
  double stheta = sin(theta);
  double ctheta = cos(theta);
  double sctheta = stheta*ctheta;
  double m11yg = mu11*Yg;
  double ctheta2 = vpMath::sqr(ctheta);
  double m02xg = mu02*Xg;
  double m11stheta = mu11*stheta;
  j = ((mu11*Xg*sctheta-mu20*Yg*sctheta+mu20*rho*ctheta
	-m11yg+m11yg*ctheta2+m02xg-m02xg*ctheta2+
	m11stheta*rho)/(mu20*ctheta2+2.0*m11stheta*ctheta
			+mu02-mu02*ctheta2));
  //Optimised calculation for Y
  double rhom02 = rho*mu02;
  double sctheta2 = stheta*ctheta2;
  double ctheta3 = ctheta2*ctheta;
  i = (-(-rho*mu11*stheta*ctheta-rhom02+rhom02*ctheta2
	 +mu11*Xg*sctheta2-mu20*Yg*sctheta2-ctheta*mu11*Yg
	 +ctheta3*mu11*Yg+ctheta*mu02*Xg-ctheta3*mu02*Xg)/
       (mu20*ctheta2+2.0*mu11*stheta*ctheta+mu02-
	mu02*ctheta2)/stheta);

}


/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the lines.
  
  \param I : The image.
  \param cMo : The pose of the camera used to initialize the moving edges.
*/
void
vpMbtDistanceCylinder::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
	// Perspective projection
	p1->changeFrame(cMo);
  p2->changeFrame(cMo);
  cercle1->changeFrame(cMo);
  cercle2->changeFrame(cMo);
  c->changeFrame(cMo);

  p1->projection();
  p2->projection();
  try{
  	cercle1->projection();
  }
  catch(...){std::cout<<"Problem when projecting circle 1\n";}
  try{
  	cercle2->projection();
  }
  catch(...){std::cout<<"Problem when projecting circle 2\n";}
  c->projection();

  double rho1,theta1;
  double rho2,theta2;

// Create the moving edges containers
  meline1 = new vpMbtMeLine ;
  meline1->setMe(me) ;
  meline2 = new vpMbtMeLine ;
  meline2->setMe(me) ;

//    meline->setDisplay(vpMeSite::RANGE_RESULT) ;
  meline1->setInitRange(0);
  meline2->setInitRange(0);

  // Conversion meter to pixels
  vpMeterPixelConversion::convertLine(cam,c->getRho1(),c->getTheta1(),rho1,theta1);
  vpMeterPixelConversion::convertLine(cam,c->getRho2(),c->getTheta2(),rho2,theta2);

	// Determine intersections between circles and limbos
	double i11,i12,i21,i22,j11,j12,j21,j22;
	getCylinderLineExtremity(i11, j11, rho1, theta1, cercle1);
  getCylinderLineExtremity(i12, j12, rho1, theta1, cercle2);
	getCylinderLineExtremity(i21, j21, rho2, theta2, cercle1);
  getCylinderLineExtremity(i22, j22, rho2, theta2, cercle2);

    // Create the image points
	vpImagePoint ip11,ip12,ip21,ip22;
	ip11.set_ij(i11,j11);
	ip12.set_ij(i12,j12);
	ip21.set_ij(i21,j21);
	ip22.set_ij(i22,j22);

  // update limits of the melines.
  int marge = /*10*/5; //ou 5 normalement
  if (ip11.get_j()<ip12.get_j()) { meline1->jmin = (int)ip11.get_j()-marge ; meline1->jmax = (int)ip12.get_j()+marge ; } else{ meline1->jmin = (int)ip12.get_j()-marge ; meline1->jmax = (int)ip11.get_j()+marge ; }
  if (ip11.get_i()<ip12.get_i()) { meline1->imin = (int)ip11.get_i()-marge ; meline1->imax = (int)ip12.get_i()+marge ; } else{ meline1->imin = (int)ip12.get_i()-marge ; meline1->imax = (int)ip11.get_i()+marge ; }

  if (ip21.get_j()<ip22.get_j()) { meline2->jmin = (int)ip21.get_j()-marge ; meline2->jmax = (int)ip22.get_j()+marge ; } else{ meline2->jmin = (int)ip22.get_j()-marge ; meline2->jmax = (int)ip21.get_j()+marge ; }
  if (ip21.get_i()<ip22.get_i()) { meline2->imin = (int)ip21.get_i()-marge ; meline2->imax = (int)ip22.get_i()+marge ; } else{ meline2->imin = (int)ip22.get_i()-marge ; meline2->imax = (int)ip21.get_i()+marge ; }
  
	// Initialize the tracking
  try
  {
    meline1->initTracking(I,ip11,ip12,rho1,theta1);
  }
  catch(...)
  {
    vpTRACE("the line can't be initialized");
  }
  try
  {
    meline2->initTracking(I,ip21,ip22,rho2,theta2);
  }
  catch(...)
  {
    vpTRACE("the line can't be initialized");
  }
}



/*!
  Track the moving edges in the image.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceCylinder::trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix & /*cMo*/)
{
  try 
  {
    meline1->track(I) ;
  }
  catch(...)
  {
    std::cout << "Track meline1 failed" << std::endl;
    Reinit = true;
  }
  try 
  {
    meline2->track(I) ;
  }
  catch(...)
  {
    std::cout << "Track meline2 failed" << std::endl;
    Reinit = true;
  }

  // Update the number of features
  nbFeaturel1 = meline1->getMeList().size();
  nbFeaturel2 = meline2->getMeList().size();
  nbFeature = meline1->getMeList().size()+meline2->getMeList().size();
}


/*!
  Update the moving edges internal parameters.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceCylinder::updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
	// Perspective projection
	p1->changeFrame(cMo);
  p2->changeFrame(cMo);
  cercle1->changeFrame(cMo);
  cercle2->changeFrame(cMo);
  c->changeFrame(cMo);

  p1->projection();
  p2->projection();
  try{
  	cercle1->projection();
  }
  catch(...){std::cout<<"Probleme projection cercle 1\n";}
  try{
  	cercle2->projection();
  }
  catch(...){std::cout<<"Probleme projection cercle 2\n";}
  c->projection();

	// Get the limbos
  double rho1,theta1;
  double rho2,theta2;

  // Conversion meter to pixels
  vpMeterPixelConversion::convertLine(cam,c->getRho1(),c->getTheta1(),rho1,theta1);
  vpMeterPixelConversion::convertLine(cam,c->getRho2(),c->getTheta2(),rho2,theta2);

	// Determine intersections between circles and limbos
	double i11,i12,i21,i22,j11,j12,j21,j22;

	getCylinderLineExtremity(i11, j11, rho1, theta1, cercle1);
  getCylinderLineExtremity(i12, j12, rho1, theta1, cercle2);

	getCylinderLineExtremity(i21, j21, rho2, theta2, cercle1);
  getCylinderLineExtremity(i22, j22, rho2, theta2, cercle2);

    // Create the image points
	vpImagePoint ip11,ip12,ip21,ip22;
	ip11.set_ij(i11,j11);
	ip12.set_ij(i12,j12);
	ip21.set_ij(i21,j21);
	ip22.set_ij(i22,j22);

  // update limits of the meline.
  int marge = /*10*/5; //ou 5 normalement
  if (ip11.get_j()<ip12.get_j()) { meline1->jmin = (int)ip11.get_j()-marge ; meline1->jmax = (int)ip12.get_j()+marge ; } else{ meline1->jmin = (int)ip12.get_j()-marge ; meline1->jmax = (int)ip11.get_j()+marge ; }
  if (ip11.get_i()<ip12.get_i()) { meline1->imin = (int)ip11.get_i()-marge ; meline1->imax = (int)ip12.get_i()+marge ; } else{ meline1->imin = (int)ip12.get_i()-marge ; meline1->imax = (int)ip11.get_i()+marge ; }

  if (ip21.get_j()<ip22.get_j()) { meline2->jmin = (int)ip21.get_j()-marge ; meline2->jmax = (int)ip22.get_j()+marge ; } else{ meline2->jmin = (int)ip22.get_j()-marge ; meline2->jmax = (int)ip21.get_j()+marge ; }
  if (ip21.get_i()<ip22.get_i()) { meline2->imin = (int)ip21.get_i()-marge ; meline2->imax = (int)ip22.get_i()+marge ; } else{ meline2->imin = (int)ip22.get_i()-marge ; meline2->imax = (int)ip21.get_i()+marge ; }

  try 
  {
    //meline1->updateParameters(I,rho1,theta1) ;
    meline1->updateParameters(I,ip11,ip12,rho1,theta1) ;
  }
  catch(...)
  {
    Reinit = true;
  }
  try 
  {
    //meline2->updateParameters(I,rho2,theta2) ;
    meline2->updateParameters(I,ip21,ip22,rho2,theta2) ;
  }
  catch(...)
  {
    Reinit = true;
  }

  // Update the numbers of features
  nbFeaturel1 = meline1->getMeList().size();
  nbFeaturel2 = meline2->getMeList().size();
  nbFeature = meline1->getMeList().size()+meline2->getMeList().size();
}


/*!
  Reinitialize the cylinder if it is required.
  
  A line is reinitialized if the 2D lines do not match enough with the projected 3D lines.
  
  \param I : the image.
  \param cMo : The pose of the camera.
*/
void
vpMbtDistanceCylinder::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
  if(meline1!= NULL)
    delete meline1;
  if(meline2!= NULL)
    delete meline2;
  
  meline1 = NULL;
  meline2 = NULL;

  initMovingEdge(I,cMo);

  Reinit = false;
}


/*!
  Display the cylinder. The 3D cylinder is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
*/
void
vpMbtDistanceCylinder::display(const vpImage<unsigned char>&I, const vpHomogeneousMatrix &cMo, const vpCameraParameters&cam, const vpColor col, const unsigned int thickness)
{
	// Perspective projection
	p1->changeFrame(cMo);
  p2->changeFrame(cMo);
  cercle1->changeFrame(cMo);
  cercle2->changeFrame(cMo);
  c->changeFrame(cMo);

  p1->projection();
  p2->projection();
  try{
  	cercle1->projection();
  }
  catch(...){std::cout<<"Problem projection circle 1";}
  try{
  	cercle2->projection();
  }
  catch(...){std::cout<<"Problem projection circle 2";}
  c->projection();

  double rho1,theta1;
  double rho2,theta2;

  // Meters to pixels conversion
  vpMeterPixelConversion::convertLine(cam,c->getRho1(),c->getTheta1(),rho1,theta1);
  vpMeterPixelConversion::convertLine(cam,c->getRho2(),c->getTheta2(),rho2,theta2);

	// Determine intersections between circles and limbos
	double i11,i12,i21,i22,j11,j12,j21,j22;

	getCylinderLineExtremity(i11, j11, rho1, theta1, cercle1);
  getCylinderLineExtremity(i12, j12, rho1, theta1, cercle2);

	getCylinderLineExtremity(i21, j21, rho2, theta2, cercle1);
  getCylinderLineExtremity(i22, j22, rho2, theta2, cercle2);

    // Create the image points
	vpImagePoint ip11,ip12,ip21,ip22;
	ip11.set_ij(i11,j11);
	ip12.set_ij(i12,j12);
	ip21.set_ij(i21,j21);
	ip22.set_ij(i22,j22);

	// Display
  vpDisplay::displayLine(I,ip11,ip12,col, thickness);
  vpDisplay::displayLine(I,ip21,ip22,col, thickness);
}

/*!
  Display the cylinder. The 3D cylinder is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param cam : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
*/
void
vpMbtDistanceCylinder::display(const vpImage<vpRGBa>&I, const vpHomogeneousMatrix &cMo, const vpCameraParameters&cam, const vpColor col, const unsigned int thickness)
{
	// Perspective projection
	p1->changeFrame(cMo);
  p2->changeFrame(cMo);
  cercle1->changeFrame(cMo);
  cercle2->changeFrame(cMo);
  c->changeFrame(cMo);

  p1->projection();
  p2->projection();
  try{
	  cercle1->projection();
  }
  catch(...){std::cout<<"Problem projection circle 1";}
  try{
	  cercle2->projection();
  }
  catch(...){std::cout<<"Problem projection circle 2";}
  c->projection();

  double rho1,theta1;
  double rho2,theta2;

  // Meters to pixels conversion
  vpMeterPixelConversion::convertLine(cam,c->getRho1(),c->getTheta1(),rho1,theta1);
  vpMeterPixelConversion::convertLine(cam,c->getRho2(),c->getTheta2(),rho2,theta2);

	// Determine intersections between circles and limbos
	double i11,i12,i21,i22,j11,j12,j21,j22;

	getCylinderLineExtremity(i11, j11, rho1, theta1, cercle1);
  getCylinderLineExtremity(i12, j12, rho1, theta1, cercle2);

	getCylinderLineExtremity(i21, j21, rho2, theta2, cercle1);
  getCylinderLineExtremity(i22, j22, rho2, theta2, cercle2);

    // Create the image points
	vpImagePoint ip11,ip12,ip21,ip22;
	ip11.set_ij(i11,j11);
	ip12.set_ij(i12,j12);
	ip21.set_ij(i21,j21);
	ip22.set_ij(i22,j22);

	// Display
  vpDisplay::displayLine(I,ip11,ip12,col, thickness);
  vpDisplay::displayLine(I,ip21,ip22,col, thickness);
}


/*!
    Enable to display the points along the lines with a color corresponding to their state.
    
    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
    - If blue : The point is removed because of the robust method in the virtual visual servoing.
    
    \param I : The image.
*/
void
vpMbtDistanceCylinder::displayMovingEdges(const vpImage<unsigned char> &I)
{
  if (meline1 != NULL)
  {
    meline1->display(I);
  }
  if (meline2 != NULL)
  {
    meline2->display(I);
  }
}

/*!
  Initialize the size of the interaction matrix and the error vector.
*/
void
vpMbtDistanceCylinder::initInteractionMatrixError()
{
    L.resize(meline1->getMeList().size()+meline2->getMeList().size(),6) ;
    error.resize(meline1->getMeList().size()+meline2->getMeList().size()) ;
    nbFeaturel1 = meline1->getMeList().size();
    nbFeaturel2 = meline2->getMeList().size();
    nbFeature = meline1->getMeList().size()+meline2->getMeList().size() ;
}

/*!
  Compute the interaction matrix and the error vector corresponding to the cylinder.
*/
void
vpMbtDistanceCylinder::computeInteractionMatrixError(const vpHomogeneousMatrix &cMo, const vpImage<unsigned char> &I)
{
	// Perspective projection
  c->changeFrame(cMo) ;
  c->projection() ;
  cercle1->changeFrame(cMo) ;
  cercle1->changeFrame(cMo) ;
  try{
  	cercle1->projection();
  }
  catch(...){std::cout<<"Problem projection circle 1\n";}
  try{
  	cercle2->projection();
  }
  catch(...){std::cout<<"Problem projection circle 2\n";}

	bool disp = false;
	bool disp2 = false;
	if (disp || disp2) vpDisplay::flush(I);

	// Build the lines
    vpFeatureBuilder::create(featureline2,*c,vpCylinder::line2) ;
    vpFeatureBuilder::create(featureline1,*c,vpCylinder::line1) ;

    double rho1 = featureline1.getRho() ;
    double theta1 = featureline1.getTheta() ;
    double rho2 = featureline2.getRho() ;
    double theta2 = featureline2.getTheta() ;

    double co1 = cos(theta1);
    double si1 = sin(theta1);
    double co2 = cos(theta2);
    double si2 = sin(theta2);

    double mx = 1.0/cam.get_px() ;
    double my = 1.0/cam.get_py() ;
    double xc = cam.get_u0() ;
    double yc = cam.get_v0() ;

    double alpha1 ;
    vpMatrix H1 ;
    H1 = featureline1.interaction() ;
    double alpha2 ;
    vpMatrix H2 ;
    H2 = featureline2.interaction() ;

    double x,y ;
    vpMeSite p ;
    unsigned int j =0 ;
    for(std::list<vpMeSite>::const_iterator it=meline1->getMeList().begin(); it!=meline1->getMeList().end(); ++it){
      x = (double)it->j;
      y = (double)it->i;

      x = (x-xc)*mx ;
      y = (y-yc)*my ;

      alpha1 = x*si1 - y*co1;

      double *Lrho = H1[0] ;
      double *Ltheta = H1[1] ;
      // Calculate interaction matrix for a distance
      for (unsigned int k=0 ; k < 6 ; k++){
        L[j][k] = (Lrho[k] + alpha1*Ltheta[k]);
      }
      error[j] = rho1 - ( x*co1 + y*si1) ;

      if (disp) vpDisplay::displayCross(I, it->i, it->j, (unsigned int)(error[j]*100), vpColor::orange,1);

      j++;
    }

    for(std::list<vpMeSite>::const_iterator it=meline2->getMeList().begin(); it!=meline2->getMeList().end(); ++it){
      x = (double)it->j;
      y = (double)it->i;

	    x = (x-xc)*mx ;
	    y = (y-yc)*my ;

      alpha2 = x*si2 - y*co2;

      double *Lrho = H2[0] ;
      double *Ltheta = H2[1] ;
      // Calculate interaction matrix for a distance
      for (unsigned int k=0 ; k < 6 ; k++){
        L[j][k] = (Lrho[k] + alpha2*Ltheta[k]);
      }
      error[j] = rho2 - ( x*co2 + y*si2) ;

      if (disp) vpDisplay::displayCross(I, it->i, it->j, (unsigned int)(error[j]*100),vpColor::red,1);

      j++;
    }
}

