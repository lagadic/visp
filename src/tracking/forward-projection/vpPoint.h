
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
 *  $Id: vpPoint.h,v 1.1.1.1 2005-06-08 07:08:11 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpPoint_H
#define vpPoint_H

/*!
  \file vpPoint.h
  \brief  class that defines what is a point
*/

#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>

#include <visp/vpForwardProjection.h>


/*!
  \class vpPoint
  \brief  class that defines what is a point
*/
class vpPoint : public vpForwardProjection
{

  friend class vpHomography ;
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
  void set_X(const double _X) { cP[0] = _X ; }
  void set_Y(const double _Y) { cP[1] = _Y ; }
  void set_Z(const double _Z) { cP[2] = _Z ; }
  void set_W(const double _W) { cP[3] = _W ; }

  //! set the point coordinates (object frame)
  void set_oX(const double _X) { oP[0] = _X ; }
  void set_oY(const double _Y) { oP[1] = _Y ; }
  void set_oZ(const double _Z) { oP[2] = _Z ; }
  void set_oW(const double _W) { oP[3] = _W ; }

  //! get the point coordinates (camera frame)
  double get_X()  const { return cP[0] ; }
  double get_Y()  const { return cP[1] ; }
  double get_Z() const  { return cP[2] ; }
  double get_W()  const { return cP[3] ; }

  //! get the point coordinates (camera frame)
  double get_oX() const { return oP[0] ; }
  double get_oY() const { return oP[1] ; }
  double get_oZ() const { return oP[2] ; }
  double get_oW() const { return oP[3] ; }

  //! set the point xyw-coordinates
  void set_x(const double _x) {  p[0] = _x ; }
  void set_y(const double _y) {  p[1] = _y ; }
  void set_w(const double _w) {  p[2] = _w ; }


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
