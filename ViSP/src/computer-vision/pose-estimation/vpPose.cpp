/****************************************************************************
 *
 * $Id: vpPose.cpp,v 1.10 2007-01-31 13:12:13 asaunier Exp $
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
    cout << "begin vpPose::Init() " << endl ;
  npt = 0 ;
  listP.kill() ;

  lambda = 0.25 ;

  c3d = NULL ;


  if (DEBUG_LEVEL1)
    cout << "end vpPose::Init() " << endl ;
}

vpPose::vpPose()
{
  if (DEBUG_LEVEL1)
    cout << "begin vpPose::vpPose() " << endl ;

  init() ;

  if (DEBUG_LEVEL1)
    cout << "end vpPose::vpPose() " << endl ;
}

/*!
  \brief destructor delete the array of point (freed the memory)
*/
vpPose::~vpPose()
{
  if (DEBUG_LEVEL1)
    cout << "begin vpPose::~vpPose() " << endl ;

  listP.kill() ;

  if (DEBUG_LEVEL1)
    cout << "end vpPose::~vpPose() " << endl ;
}
/*!
  \brief  delete the array of point
*/
void
vpPose::clearPoint()
{
  if (DEBUG_LEVEL1)
    cout << "begin vpPose::ClearPoint() " << endl ;

  listP.kill() ;
  npt = 0 ;

  if (DEBUG_LEVEL1)
    cout << "end vpPose::ClearPoint() " << endl ;
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
    cout << "begin vpPose::AddPoint(Dot) " << endl ;

  listP+= newP ;
  npt ++ ;

  if (DEBUG_LEVEL1)
    cout << "end vpPose::AddPoint(Dot) " << endl ;
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

	 cout << x1 <<" " << y1 << " "<< z1 << endl ;
	 cout << x2 <<" " << y2 << " "<< z2 << endl ;
	 cout << x3 <<" " << y3 << " "<< z3 << endl ;
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
    cout << "begin vpPose::ComputePose()" << endl ;
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
    cout << "end vpPose::ComputePose()" << endl ;
}



void
vpPose::printPoint()
{

  listP.front() ;
  vpPoint P ;
  while (!listP.outside())
  {
    P = listP.value() ;

    cout << "3D oP " << P.oP.t() ;
    cout << "3D cP " << P.cP.t() ;
    cout << "2D    " << P.p.t() ;

    listP.next() ;
  }
}


void
vpPose::display(vpImage<unsigned char> &I,
		vpHomogeneousMatrix &cMo,
		vpCameraParameters &cam,
		double size,
		int col)
{
  vpDisplay::displayFrame(I,cMo,cam, size,col);
}

