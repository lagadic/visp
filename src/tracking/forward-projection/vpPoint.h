
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpPoint.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpPoint.h,v 1.6 2006-04-19 09:01:22 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpPoint_H
#define vpPoint_H

class vpHomography ;

/*!
  \file vpPoint.h
  \brief  class that defines what is a point
*/

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>

class vpHomography;

/*!
  \class vpPoint
  \brief  class that defines what is a point
*/
class vpPoint : public vpForwardProjection
{

public:
  //! basic construction
  void init() ;
  //! basic constructor
  vpPoint() ;
  //! destructor
  ~vpPoint() { ; }

public:
  /*
    /section Set coordinates
  */

  //@{

  //! set the point coordinates (camera frame)
  void set_X(const double X) { cP[0] = X ; }
  void set_Y(const double Y) { cP[1] = Y ; }
  void set_Z(const double Z) { cP[2] = Z ; }
  void set_W(const double W) { cP[3] = W ; }

  //! set the point coordinates (object frame)
  void set_oX(const double X) { oP[0] = X ; }
  void set_oY(const double Y) { oP[1] = Y ; }
  void set_oZ(const double Z) { oP[2] = Z ; }
  void set_oW(const double W) { oP[3] = W ; }

  //! get the point coordinates (camera frame)
  double get_X()  const { return cP[0] ; }
  double get_Y()  const { return cP[1] ; }
  double get_Z() const  { return cP[2] ; }
  double get_W()  const { return cP[3] ; }

  //! get the point coordinates (object frame)
  double get_oX() const { return oP[0] ; }
  double get_oY() const { return oP[1] ; }
  double get_oZ() const { return oP[2] ; }
  double get_oW() const { return oP[3] ; }

  //! set the point xyw-coordinates
  void set_x(const double x) {  p[0] = x ; }
  void set_y(const double y) {  p[1] = y ; }
  void set_w(const double w) {  p[2] = w ; }


  //! get the point xyw-coordinates
  double get_x()  const { return p[0] ; }
  double get_y()  const { return p[1] ; }
  double get_w()  const { return p[2] ; }


  //! set the point world coordinates
  void setWorldCoordinates(const double ox,
			   const double oy,
			   const double oz) ;
  //! set the point world coordinates
  void setWorldCoordinates(const vpColVector &_oP) ;
  //! get the point world coordinates
  void getWorldCoordinates(double& ox,
			   double& oy,
			   double& oz) ;
  //! set the point world coordinates
  void getWorldCoordinates(vpColVector &_oP) ;
  vpColVector getWorldCoordinates(void) ;
  //@}

  //! projection
  void projection(const vpColVector &_cP, vpColVector &_p) ;
  void changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &_cP) ;


  void display(vpImage<unsigned char> &I,
	       const vpCameraParameters &cam,
	       const int color=vpColor::green) ;
  void display(vpImage<unsigned char> &I,
	       const vpHomogeneousMatrix &cMo,
	       const vpCameraParameters &cam,
	       const int color=vpColor::green) ;
  vpPoint *duplicate() const ;

} ;


const vpPoint operator*(const vpHomogeneousMatrix &M, const vpPoint& p) ;
const vpPoint operator*(const vpHomography &H, const vpPoint& p) ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
