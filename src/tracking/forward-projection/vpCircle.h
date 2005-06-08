
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
 *  $Id: vpCircle.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
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

  vpCircle(const vpColVector& _oP) ;
  vpCircle(const double _A, const double _B1,
	   const double _C,
	   const double _X0, const double _Y0,
	   const double _Z0,
	   const double _R) ;


  void setWorldCoordinates(const vpColVector& _oP) ;
  void setWorldCoordinates(const double _A, const double _B1,
			   const double _C,
			   const double _X0, const double _Y0,
			   const double _Z0,
			   const double _R) ;


  double getA() const { return cP[0] ; }
  double getB()  const{ return cP[1] ; }
  double getC() const { return cP[2] ; }

  double getX() const { return cP[3] ; }
  double getY() const { return cP[4] ; }
  double getZ()  const{ return cP[5] ; }

  double getR() const { return cP[6] ; }



  void projection(const vpColVector &_cP, vpColVector &_p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;


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
