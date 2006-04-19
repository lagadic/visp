
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpCylinder.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpCylinder.h,v 1.2 2006-04-19 09:01:22 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpCylinder.h
  \brief  class that defines what is a cylinder
*/

#ifndef vpCylinder_hh
#define vpCylinder_hh


#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

/*!
  \class vpCylinder
  \brief  class that defines what is a cylinder
*/
class vpCylinder : public vpForwardProjection
{
public:
  void init() ;
  vpCylinder() ;
  ~vpCylinder() ;

public:
  enum lineCylinderEnum
    {
      line1,
      line2
    };

  vpCylinder(const vpColVector& oP) ;
  vpCylinder(const double A, const double B1,
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

  double getRho1()  const  { return p[0] ; }
  double getTheta1() const  { return p[1] ; }

  double getRho2()  const  { return p[2] ; }
  double getTheta2() const { return p[3] ; }

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

  vpCylinder *duplicate() const ;
} ;


#endif
