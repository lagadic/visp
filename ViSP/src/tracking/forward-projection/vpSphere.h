
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpSphere.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpSphere.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     forward projection of a sphere
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpSphere.h
  \brief  forward projection of a sphere
*/

#ifndef vpSphere_hh
#define vpSphere_hh


#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

/*!
  \class vpSphere
  \brief   forward projection of a sphere
*/
class vpSphere : public vpForwardProjection
{
public:
  void init() ;
  vpSphere() ;
  ~vpSphere() ;

public:
  vpSphere(const vpColVector& _oP) ;
  vpSphere(const double _X0, const double _Y0,
	   const double _Z0,
	   const double _R) ;


  void setWorldCoordinates(const vpColVector& _oP) ;
  void setWorldCoordinates(const double _X0, const double _Y0,
			   const double _Z0,
			   const double _R) ;

  double get_x() { return p[0] ; }
  double get_y() { return p[1] ; }
  double get_mu20() { return p[2] ; }
  double get_mu11() { return p[3] ; }
  double get_mu02() { return p[4] ; }

  double getX() const { return cP[0] ; }
  double getY() const { return cP[1] ; }
  double getZ()  const{ return cP[2] ; }

  double getR() const { return cP[3] ; }



  void projection(const vpColVector &_cP, vpColVector &_p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;


  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const int color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const int color=vpColor::green) ;


  vpSphere *duplicate() const ;
} ;


#endif
