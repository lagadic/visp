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
* Pose computation.
*
* Authors:
* Eric Marchand
* Francois Chaumette
*
*****************************************************************************/


/*!
\file vpPose.cpp
\brief Fichier contenant la classe vpPose (tout ce qui est necessaire
pour faire du calcul de pose par difference methode
*/

#include <visp/vpPose.h>
#include <visp/vpDebug.h>
#include <visp/vpException.h>
#include <visp/vpPoseException.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDisplay.h>
#include <visp/vpMath.h>

#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

#define DEBUG_LEVEL1 0
/*!
\brief   basic initialisation (called by the constructors)
*/
void
vpPose::init()
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::Init() " << std::endl ;
#endif
  npt = 0 ;
  listP.clear() ;

  lambda = 0.25 ;

  c3d = NULL ;

  vvsIterMax = 200 ;

  distanceToPlaneForCoplanarityTest = 0.001 ;
  
  computeCovariance = false;
  
  ransacMaxTrials = 1000;

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::Init() " << std::endl ;
#endif

}

vpPose::vpPose()
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::vpPose() " << std::endl ;
#endif

  init() ;

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::vpPose() " << std::endl ;
#endif

}

/*!
\brief destructor delete the array of point (freed the memory)
*/
vpPose::~vpPose()
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::~vpPose() " << std::endl ;
#endif

  listP.clear() ;

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::~vpPose() " << std::endl ;
#endif
}
/*!
\brief  delete the array of point
*/
void
vpPose::clearPoint()
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::ClearPoint() " << std::endl ;
#endif

  listP.clear() ;
  npt = 0 ;

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::ClearPoint() " << std::endl ;
#endif
}

/*!
\brief  Add a new point in the array of point.
\param  newP : New point to add  in the array of point.
\warning Considering a point from the class vpPoint, X, Y, and Z will
represent the 3D information and x and y its 2D informations.
These 5 fields must be initialized to be used within this library
*/
void
vpPose::addPoint(const vpPoint& newP)
{

#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::AddPoint(Dot) " << std::endl ;
#endif

  listP.push_back(newP);
  npt ++ ;

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::AddPoint(Dot) " << std::endl ;
#endif
}


void 
vpPose::setDistanceToPlaneForCoplanarityTest(double d)
{
  distanceToPlaneForCoplanarityTest = d ;
}

/*!
\brief test the coplanarity of the set of points

\param coplanar_plane_type:
   1: if plane x=cst
   2: if plane y=cst
   3: if plane z=cst
   0: any other plane
\return true if points are coplanar
false if not
*/
bool
vpPose::coplanar(int &coplanar_plane_type)
{
  coplanar_plane_type = 0;
  if (npt <2)
  {
    vpERROR_TRACE("Not enough point (%d) to compute the pose  ",npt) ;
    throw(vpPoseException(vpPoseException::notEnoughPointError,
      "Not enough points ")) ;
  }

  if (npt ==3) return true ;

  double x1,x2,x3,y1,y2,y3,z1,z2,z3 ;

  std::list<vpPoint>::const_iterator it = listP.begin();

  vpPoint P1,P2, P3 ;
  P1 = *it;  ++it;
  //if ((P1.get_oX() ==0) && (P1.get_oY() ==0) && (P1.get_oZ() ==0)) 
  if ((std::fabs(P1.get_oX()) <= std::numeric_limits<double>::epsilon()) 
    && (std::fabs(P1.get_oY()) <= std::numeric_limits<double>::epsilon()) 
    && (std::fabs(P1.get_oZ()) <= std::numeric_limits<double>::epsilon())) 
  { 
    P1 = *it; ++it;
  }
  P2 = *it; ++it;
  P3 = *it;

  x1 = P1.get_oX() ;
  x2 = P2.get_oX() ;
  x3 = P3.get_oX() ;

  y1 = P1.get_oY() ;
  y2 = P2.get_oY() ;
  y3 = P3.get_oY() ;

  z1 = P1.get_oZ() ;
  z2 = P2.get_oZ() ;
  z3 = P3.get_oZ() ;

  double a =  y1*z2 - y1*z3 - y2*z1 + y2*z3 + y3*z1 - y3*z2 ;
  double b = -x1*z2 + x1*z3 + x2*z1 - x2*z3 - x3*z1 + x3*z2 ;
  double c =  x1*y2 - x1*y3 - x2*y1 + x2*y3 + x3*y1 - x3*y2 ;
  double d = -x1*y2*z3 + x1*y3*z2 +x2*y1*z3 - x2*y3*z1 - x3*y1*z2 + x3*y2*z1 ;

  if (std::fabs(b) <= std::numeric_limits<double>::epsilon()
      && std::fabs(c) <= std::numeric_limits<double>::epsilon() ) {
    coplanar_plane_type = 1; // ax=d
  } else if (std::fabs(a) <= std::numeric_limits<double>::epsilon()
             && std::fabs(c) <= std::numeric_limits<double>::epsilon() ) {
    coplanar_plane_type = 2; // by=d
  } else if (std::fabs(a) <= std::numeric_limits<double>::epsilon()
             && std::fabs(b) <= std::numeric_limits<double>::epsilon() ) {
    coplanar_plane_type = 3; // cz=d
  }

  double  D = sqrt(vpMath::sqr(a)+vpMath::sqr(b)+vpMath::sqr(c)) ;

  double dist;

  for(it=listP.begin(); it != listP.end(); ++it)
  {
    P1 = *it ;
    dist = (a*P1.get_oX() + b*P1.get_oY()+c*P1.get_oZ()+d)/D ;

    if (fabs(dist) > distanceToPlaneForCoplanarityTest)
    {
      vpDEBUG_TRACE(10," points are not coplanar ") ;
      //	TRACE(" points are not coplanar ") ;
      return false ;
    }
  }

  vpDEBUG_TRACE(10," points are  coplanar ") ;
  //  vpTRACE(" points are  coplanar ") ;

  return true ;
}

/*!
\brief Compute and return the residual expressed in meter for
the pose matrix 'cMo'.

\param cMo : Input pose. The matrix that defines the pose to be tested.

\return The value of he residual in meter.

*/
double
vpPose::computeResidual(vpHomogeneousMatrix &cMo)
{

  double residual = 0 ;
  vpPoint P ;
  for(std::list<vpPoint>::const_iterator it=listP.begin(); it != listP.end(); ++it)
  {
    P = *it;
    double x = P.get_x() ;
    double y = P.get_y() ;

    P.track(cMo) ;

    residual += vpMath::sqr(x-P.get_x()) + vpMath::sqr(y-P.get_y())  ;
  }
  return residual ;
}



/*!
\brief Compute the pose according to the desired method

the different method are

LAGRANGE         Lagrange approach
(test is done to switch between planar and
non planar algorithm)

DEMENTHON        Dementhon approach
(test is done to switch between planar and
non planar algorithm)

VIRTUAL_VS       Virtual visual servoing approach

DEMENTHON_VIRTUAL_VS Virtual visual servoing approach initialized using
Dementhon approach

LAGRANGE_VIRTUAL_VS  Virtual visual servoing initialized using
Lagrange approach

*/
void
vpPose::computePose(vpPoseMethodType methode, vpHomogeneousMatrix& cMo)
{
#if (DEBUG_LEVEL1)
  std::cout << "begin vpPose::ComputePose()" << std::endl ;
#endif

  if (npt <4)
  {
    vpERROR_TRACE("Not enough point (%d) to compute the pose  ",npt) ;
    throw(vpPoseException(vpPoseException::notEnoughPointError,
      "No enough point ")) ;
  }

  switch (methode)
  {
  case DEMENTHON :
  case DEMENTHON_VIRTUAL_VS :
  case DEMENTHON_LOWE :
    {

      if (npt <4)
      {
        vpERROR_TRACE("Dementhon method cannot be used in that case ") ;
        vpERROR_TRACE("(at least 4 points are required)") ;
        vpERROR_TRACE("Not enough point (%d) to compute the pose  ",npt) ;
        throw(vpPoseException(vpPoseException::notEnoughPointError,
          "Not enough points ")) ;
      }

      // test si les point 3D sont coplanaires
      int coplanar_plane_type = 0;
      int plan = coplanar(coplanar_plane_type) ;
      if (plan == 1)
      {
        //std::cout << "Plan" << std::endl;
        try{
          poseDementhonPlan(cMo);
        }
        catch(...)
        {
          vpERROR_TRACE(" ") ;
          throw ;
        }
        //std::cout << "Fin Plan" << std::endl;
      }
      else
      {
        //std::cout << "No Plan" << std::endl;
        try{
          poseDementhonNonPlan(cMo) ;
        }
        catch(...)
        {
          vpERROR_TRACE(" ") ;
          throw ;
        }
        //std::cout << "Fin No Plan" << std::endl;
      }
    }
    break ;
  case LAGRANGE :
  case LAGRANGE_VIRTUAL_VS :
  case LAGRANGE_LOWE :
    {
      // test si les point 3D sont coplanaires
      int coplanar_plane_type;
      int  plan = coplanar(coplanar_plane_type) ;

      if (plan == 1)
      {

        if (npt <4)
        {
          vpERROR_TRACE("Lagrange method cannot be used in that case ") ;
          vpERROR_TRACE("(at least 4 points are required)") ;
          vpERROR_TRACE("Not enough point (%d) to compute the pose  ",npt) ;
          throw(vpPoseException(vpPoseException::notEnoughPointError,
            "Not enough points ")) ;
        }
        try {
          poseLagrangePlan(cMo, coplanar_plane_type);
        }
        catch(...)
        {
          vpERROR_TRACE(" ") ;
          throw ;
        }
      }
      else
      {
        if (npt <4)
        {
          vpERROR_TRACE("Lagrange method cannot be used in that case ") ;
          vpERROR_TRACE("(at least 4 points are required)") ;
          vpERROR_TRACE("Not enough point (%d) to compute the pose  ",npt) ;
          throw(vpPoseException(vpPoseException::notEnoughPointError,
            "Not enough points ")) ;
        }
        try {
          poseLagrangeNonPlan(cMo);
        }
        catch(...)
        {
          vpERROR_TRACE(" ") ;
          throw ;
        }
      }
    }
    break;
  case RANSAC:
    if (npt <4)
    {
      vpERROR_TRACE("Ransac method cannot be used in that case ") ;
      vpERROR_TRACE("(at least 4 points are required)") ;
      vpERROR_TRACE("Not enough point (%d) to compute the pose  ",npt) ;
      throw(vpPoseException(vpPoseException::notEnoughPointError,
        "Not enough points ")) ;
    }
    try {
      poseRansac(cMo);
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }
    break;
  case LOWE :
  case VIRTUAL_VS:
    break ;
  }

  switch (methode)
  {
  case LAGRANGE :
  case DEMENTHON :
  case RANSAC :
    break ;
  case VIRTUAL_VS:
  case LAGRANGE_VIRTUAL_VS:
  case DEMENTHON_VIRTUAL_VS:
    {
      try
      {
        poseVirtualVS(cMo);
      }
      catch(...)
      {
        vpERROR_TRACE(" ") ;
        throw ;
      }
    }
    break ;
  case LOWE:
  case LAGRANGE_LOWE:
  case DEMENTHON_LOWE:
    {
      try
      {
        poseLowe(cMo);
      }
      catch(...)
      {
        vpERROR_TRACE(" ") ;
        throw ;
      }
    }
    break ;
  }

#if (DEBUG_LEVEL1)
  std::cout << "end vpPose::ComputePose()" << std::endl ;
#endif
}



void
vpPose::printPoint()
{
  vpPoint P;
  for(std::list<vpPoint>::const_iterator it=listP.begin(); it != listP.end(); ++it)
  {
    P = *it ;

    std::cout << "3D oP " << P.oP.t() ;
    std::cout << "3D cP " << P.cP.t() ;
    std::cout << "2D    " << P.p.t() ;
  }
}


void
vpPose::display(vpImage<unsigned char> &I,
                vpHomogeneousMatrix &cMo,
                vpCameraParameters &cam,
                double size,
                vpColor col)
{
  vpDisplay::displayFrame(I, cMo, cam, size, col);
}


void
vpPose::display(vpImage<vpRGBa> &I,
                vpHomogeneousMatrix &cMo,
                vpCameraParameters &cam,
                double size,
                vpColor col)
{
  vpDisplay::displayFrame(I,cMo,cam, size,col);
}

/*
\brief display the 3D model in image I
\warning the 2D coordinate of the point have to be initialized
(lispP has to be modified)
*/
void
vpPose::displayModel(vpImage<unsigned char> &I,
                     vpCameraParameters &cam,
                     vpColor col)
{ 
  vpPoint P ;
  vpImagePoint ip;
  for(std::list<vpPoint>::const_iterator it=listP.begin(); it != listP.end(); ++it)
  {
    P = *it;
    vpMeterPixelConversion::convertPoint(cam, P.p[0], P.p[1], ip) ;
    vpDisplay::displayCross(I, ip, 5, col) ;
    //  std::cout << "3D oP " << P.oP.t() ;
    //  std::cout << "3D cP " << P.cP.t() ;
    //  std::cout << "2D    " << P.p.t() ;
  }
}

/*
\brief display the 3D model in image I
\warning the 2D coordinate of the point have to be initialized
(lispP has to be modified)
*/
void
vpPose::displayModel(vpImage<vpRGBa> &I,
                     vpCameraParameters &cam,
                     vpColor col)
{ 
  vpPoint P ;
  vpImagePoint ip;
  for(std::list<vpPoint>::const_iterator it=listP.begin(); it != listP.end(); ++it)
  {
    P = *it;
    vpMeterPixelConversion::convertPoint(cam, P.p[0], P.p[1], ip) ;
    vpDisplay::displayCross(I, ip, 5, col) ;
    //  std::cout << "3D oP " << P.oP.t() ;
    //  std::cout << "3D cP " << P.cP.t() ;
    //  std::cout << "2D    " << P.p.t() ;
  }
}


/*!
\brief Carries out the camera pose the image of a rectangle and
the intrinsec parameters, the length on x axis is known but the
proprtion of the rectangle are unknown.

This method is taken from "Markerless Tracking using Planar Structures
in the Scene" by Gilles Simon. The idea is to compute the homography H
giving the image point of the rectangle by associating them with the
coordinates (0,0)(1,0)(1,1/s)(0,1/s) (the rectangle is on the Z=0 plane).
If K is the intrinsec parameters matrix, we have  s = ||Kh1||/ ||Kh2||. s
gives us the proportion of the rectangle

\param p1,p2,p3,p4: the image of the corners of the rectangle
(respectively the image of  (0,0),(lx,0),(lx,lx/s) and (0,lx/s)) (input)
\param cam: the camera used (input)
\param lx: the rectangle size on the x axis (input)
\param cMo: the camera pose (output)
\return int : OK if no pb occurs
*/
double
vpPose::poseFromRectangle(vpPoint &p1,vpPoint &p2,
                          vpPoint &p3,vpPoint &p4,
                          double lx, vpCameraParameters & cam,
                          vpHomogeneousMatrix & cMo)
{

  double rectx[4] ;
  double recty[4] ;
  rectx[0]= 0 ;
  recty[0]=0 ;
  rectx[1]=1 ;
  recty[1]=0 ;
  rectx[2]=1 ;
  recty[2]=1 ;
  rectx[3]=0 ;
  recty[3]=1 ;
  double irectx[4] ;
  double irecty[4] ;
  irectx[0]=(p1.get_x()) ;
  irecty[0]=(p1.get_y()) ;
  irectx[1]=(p2.get_x()) ;
  irecty[1]=(p2.get_y()) ;
  irectx[2]=(p3.get_x()) ;
  irecty[2]=(p3.get_y()) ;
  irectx[3]=(p4.get_x()) ;
  irecty[3]=(p4.get_y()) ;

  //calcul de l'homographie
  vpMatrix H(3,3);
  vpHomography hom;

  //  vpHomography::HartleyDLT(4,rectx,recty,irectx,irecty,hom);
  vpHomography::HLM(4,rectx,recty,irectx,irecty,1,hom);
  for (unsigned int i=0 ; i < 3 ; i++)
    for(unsigned int j=0 ; j < 3 ; j++)
      H[i][j] = hom[i][j] ;
  //calcul de s =  ||Kh1||/ ||Kh2|| =ratio (length on x axis/ length on y axis)
  vpColVector kh1(3);
  vpColVector kh2(3);
  vpMatrix K(3,3);
  K = cam.get_K();
  K.setIdentity();
  vpMatrix Kinv =K.pseudoInverse();

  vpMatrix KinvH =Kinv*H;
  kh1=KinvH.column(1);
  kh2=KinvH.column(2);


  double s= sqrt(kh1.sumSquare())/sqrt(kh2.sumSquare());


  vpMatrix D(3,3);
  D.setIdentity();
  D[1][1]=1/s;
  vpMatrix cHo=H*D;

  //Calcul de la rotation et de la translation
  //  PoseFromRectangle(p1,p2,p3,p4,1/s,lx,cam,cMo );
  p1.setWorldCoordinates(0,0,0) ;
  p2.setWorldCoordinates(lx,0,0) ;
  p3.setWorldCoordinates(lx,lx/s,0) ;
  p4.setWorldCoordinates(0,lx/s,0) ;


  vpPose P ;
  P.addPoint(p1) ;
  P.addPoint(p2) ;
  P.addPoint(p3) ;
  P.addPoint(p4) ;


  P.computePose(vpPose::DEMENTHON_LOWE,cMo) ;
  return lx/s ;

}
