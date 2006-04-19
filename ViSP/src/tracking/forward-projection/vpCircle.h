
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpCircle.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpCircle.h,v 1.2 2006-04-19 09:01:22 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpCircle.h
  \brief  class that defines what is a circle
*/

#ifndef vpCircle_hh
#define vpCircle_hh


#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

/*!
  \class vpCircle
  \brief  class that defines what is a circle
*/
class vpCircle : public vpForwardProjection
{
public:
  void init() ;
  vpCircle() ;
  ~vpCircle() ;

public:
  enum lineCircleEnum
    {
      line1,
      line2
    };

  vpCircle(const vpColVector& oP) ;
  vpCircle(const double A, const double B1,
	   const double C,
	   const double X0, const double Y0,
	   const double Z0,
	   const double R) ;


  void setWorldCoordinates(const vpColVector& oP) ;
  void setWorldCoordinates(const double A, const double B1,
			   const double C,
			   const double X0, const double Y0,
			   const double Z0,
			   const double R) ;


  double getA() const { return cP[0] ; }
  double getB()  const{ return cP[1] ; }
  double getC() const { return cP[2] ; }

  double getX() const { return cP[3] ; }
  double getY() const { return cP[4] ; }
  double getZ()  const{ return cP[5] ; }

  double getR() const { return cP[6] ; }



  void projection(const vpColVector &cP, vpColVector &p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP) ;


  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const int color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const int color=vpColor::green) ;
  vpCircle *duplicate() const ;

} ;


#endif
