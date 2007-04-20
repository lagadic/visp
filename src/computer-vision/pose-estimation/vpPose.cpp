/****************************************************************************
 *
 * $Id: vpPose.cpp,v 1.14 2007-04-20 14:22:15 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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

#define DEBUG_LEVEL1 0
/*!
  \brief   basic initialisation (called by the constructors)
*/
void
vpPose::init()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin vpPose::Init() " << std::endl ;
  npt = 0 ;
  listP.kill() ;

  lambda = 0.25 ;

  c3d = NULL ;


  if (DEBUG_LEVEL1)
    std::cout << "end vpPose::Init() " << std::endl ;
}

vpPose::vpPose()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin vpPose::vpPose() " << std::endl ;

  init() ;

  if (DEBUG_LEVEL1)
    std::cout << "end vpPose::vpPose() " << std::endl ;
}

/*!
  \brief destructor delete the array of point (freed the memory)
*/
vpPose::~vpPose()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin vpPose::~vpPose() " << std::endl ;

  listP.kill() ;

  if (DEBUG_LEVEL1)
    std::cout << "end vpPose::~vpPose() " << std::endl ;
}
/*!
  \brief  delete the array of point
*/
void
vpPose::clearPoint()
{
  if (DEBUG_LEVEL1)
    std::cout << "begin vpPose::ClearPoint() " << std::endl ;

  listP.kill() ;
  npt = 0 ;

  if (DEBUG_LEVEL1)
    std::cout << "end vpPose::ClearPoint() " << std::endl ;
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

  if (DEBUG_LEVEL1)
    std::cout << "begin vpPose::AddPoint(Dot) " << std::endl ;

  listP+= newP ;
  npt ++ ;

  if (DEBUG_LEVEL1)
    std::cout << "end vpPose::AddPoint(Dot) " << std::endl ;
}

/*!
  \brief test the coplanarity of the set of points
  \return true if points are coplanar
  false if not
*/
bool
vpPose::coplanaire()
{

  if (npt <2)
  {
    vpERROR_TRACE("Not enough point (%d) to compute the pose  ",npt) ;
    throw(vpPoseException(vpPoseException::notEnoughPointError,
			  "Not enough points ")) ;
  }

  if (npt ==3) return true ;

  double x1,x2,x3,y1,y2,y3,z1,z2,z3 ;

  listP.front() ;

  vpPoint P1,P2, P3 ;
  P1 = listP.value() ; listP.next() ;
  if ((P1.get_oX() ==0) && (P1.get_oY() ==0) && (P1.get_oZ() ==0))
  {   P1 = listP.value() ; listP.next() ;}
  P2 = listP.value() ; listP.next() ;
  P3 = listP.value() ;

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


  double  D = sqrt(vpMath::sqr(a)+vpMath::sqr(b)+vpMath::sqr(c)) ;
  if (fabs(D) < 1e-10)
  {
    /*   listP.front() ;
	 while (!listP.outside())
	 {
	 P1 = listP.value() ;
	 P1.print() ;
	 listP.next() ;

	 }

	 std::cout << x1 <<" " << y1 << " "<< z1 << std::endl ;
	 std::cout << x2 <<" " << y2 << " "<< z2 << std::endl ;
	 std::cout << x3 <<" " << y3 << " "<< z3 << std::endl ;
	 vpERROR_TRACE("division by zero  ") ;*/
    // throw(vpException(vpException::divideByZeroError,
    //		      "division by zero  ")) ;
  }

  double dist;
  listP.front() ;
  while (!listP.outside())
  {
    P1 = listP.value() ;
    dist = (a*P1.get_oX() + b*P1.get_oY()+c*P1.get_oZ()+d)/D ;

    if (fabs(dist) > 0.01)
    {
      vpDEBUG_TRACE(10," points are not coplanar ") ;
      //	TRACE(" points are not coplanar ") ;
      return false ;
    }
    listP.next() ;
  }

  vpDEBUG_TRACE(10," points are  coplanar ") ;
  //  vpTRACE(" points are  coplanar ") ;

  return true ;
}

/*!
  \brief Compute and return the residual expressed in meter for
  the pose matrix 'cMo'
  \param input vpMatrix &pose : the matrix that defines the pose to be tested
  \return the value of he residual in meter
*/
double
vpPose::computeResidual(vpHomogeneousMatrix &_cMo)
{

  double residual = 0 ;

  listP.front() ;
  vpPoint P ;
  while (!listP.outside())
  {
    P = listP.value() ;
    double x = P.get_x() ;
    double y = P.get_y() ;

    P.track(_cMo) ;

    residual += vpMath::sqr(x-P.get_x()) + vpMath::sqr(y-P.get_y())  ;
    listP.next() ;
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
vpPose::computePose(poseMethodEnum methode, vpHomogeneousMatrix& cMo)
{
  if (DEBUG_LEVEL1)
    std::cout << "begin vpPose::ComputePose()" << std::endl ;
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
      int  plan = coplanaire() ;

      if (plan == 1)
      {
	try{
	  poseDementhonPlan(cMo);
	}
	catch(...)
	{
	  vpERROR_TRACE(" ") ;
	  throw ;
	}
      }
      else
      {
	try{
	  poseDementhonNonPlan(cMo) ;
	}
	catch(...)
	{
	  vpERROR_TRACE(" ") ;
	  throw ;
	}
      }
    }
    break ;
  case LAGRANGE :
  case LAGRANGE_VIRTUAL_VS :
  case LAGRANGE_LOWE :
    {
      // test si les point 3D sont coplanaires

      int  plan = coplanaire() ;

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
	  poseLagrangePlan(cMo);
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
  case LOWE :
  case VIRTUAL_VS:
    break ;
  }

  switch (methode)
  {
  case LAGRANGE :
  case DEMENTHON :
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

  if (DEBUG_LEVEL1)
    std::cout << "end vpPose::ComputePose()" << std::endl ;
}



void
vpPose::printPoint()
{

  listP.front() ;
  vpPoint P ;
  while (!listP.outside())
  {
    P = listP.value() ;

    std::cout << "3D oP " << P.oP.t() ;
    std::cout << "3D cP " << P.cP.t() ;
    std::cout << "2D    " << P.p.t() ;

    listP.next() ;
  }
}


void
vpPose::display(vpImage<unsigned char> &I,
		vpHomogeneousMatrix &cMo,
		vpCameraParameters &cam,
		double size,
		vpColor::vpColorType col)
{
  vpDisplay::displayFrame(I,cMo,cam, size,col);
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
    \Param cam: the camera used (input)
    \param lx: the rectangle size on the x axis (input)
    \param cMo: the camera pose (output)
    \return int : OK if no pb occurs
  */
void
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
  for (int i=0 ; i < 3 ; i++)
    for(int j=0 ; j < 3 ; j++)
      H[i][j] = hom[i][j] ;
  //calcul de s =  ||Kh1||/ ||Kh2|| =ratio (length on x axis/ length on y axis)
  vpColVector kh1(3);
  vpColVector kh2(3);
  vpMatrix K(3,3);
  K=cam.K;
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

}
