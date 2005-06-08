
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTracker.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpCameraParameters.h,v 1.1.1.1 2005-06-08 07:08:13 fspindle Exp $
 *
 * Description
 * ============
 *       Class vpCameraParameters define the camera intrinsic parameters
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



/*!
  \file vpCameraParameters.h
  \brief Declaration of the vpCameraParameters class.
  Class vpCameraParameters define the camera intrinsic parameters

*/

#ifndef vpCAMERA_H
#define vpCAMERA_H

#include <visp/vpMatrix.h>


/*!
  \class vpCameraParameters
  \brief Declaration of the vpCameraParameters class.
  Class vpCameraParameters define the camera intrinsic parameters

  \date 1999, Last modified May, 3 2002

  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes

  The main parameters intrinsic that describe the camera are
  (px, py) the ratio between the focal length and the size of a pixel,
  and (u0, v0) that  are the coordinates of the principal point.


  If we denote (u,v) the position of the corresponding pixel in the digitized
  image, this position is related to the coordinates (x,y) in the normalized
  space by:
  \f[
  \left\{ \begin{array}{l}
  u = u_0 + p_x x  \\
  v = v_0 + p_y y
  \end{array}\right.
  \f]

  if a model with distortion is considered, we got:
  \f[
  \left\{ \begin{array}{l}
  u = u_0 + p_x x +  \delta_u(u,v) \\
  v = v_0 + p_y y + \delta_v(u,v)
  \end{array}\right.
  \f]
  where  \f$\delta_u\f$ and \f$\delta_v\f$ are
  geometrical distortions introduced in  the camera model. These distotions are due to imperfections in the lenses design and assembly there usually are some
  positional errors that have to be taken into account.
  \f$\delta_u\f$ and \f$\delta_v\f$  can be modeled as follow:
  \f[
  \left\{ \begin{array}{l}
  \delta_u(u,v) = K_d \;r^2\; \tilde u  \\
  \\
  \delta_v(u,v) = K_d\; r^2\; \tilde v
  \end{array}\right.
  \f]
  with \f$\tilde u = u-u_0, \;\; \tilde v = (v-v_0)\f$ and  \f$r^2 = \tilde u^2 +  \tilde
  v^2\f$.


*/

#define  VISP_DEFAULT_CAMERA_U0 192
#define  VISP_DEFAULT_CAMERA_V0 144

#define  VISP_DEFAULT_CAMERA_PX 600
#define  VISP_DEFAULT_CAMERA_PY 600

#define  VISP_DEFAULT_CAMERA_COL 300
#define  VISP_DEFAULT_CAMERA_LIG 200


class vpCameraParameters
{
public:
  double px, py ; //!< pixel size

  double u0,v0 ;  //!<  principal point
  double kd;	  //!<  Radial distortion

  int nlig, ncol ;
public:
  vpCameraParameters() ;
  vpCameraParameters(const vpCameraParameters &) ;
  vpCameraParameters(const double _px, const double _py,
		     const double _u0, const double _v0,
		     const double _kd = 0) ;

  void init() ;
  void init(const vpCameraParameters &c) ;
  void init(const double _px, const double _py,
	   const double _u0, const double _v0,
	   const double _kd = 0) ;


  ~vpCameraParameters() ;


  vpMatrix K ; //<! camera projection matrix

  void setPrincipalPoint(const double _u0, const double _v0) ;
  void setKd(const double _kd) ;
  void setPixelRatio(const double _px,const double _py) ;


  void computeMatrix() ;
  void printParameters(int version=2) ;


  vpCameraParameters& operator =(const vpCameraParameters& f) ;

  inline double get_px() { return px ; }
  inline double get_py() { return py ; }
  inline double get_u0() { return u0 ; }
  inline double get_v0() { return v0 ; }
  inline double get_kd() { return kd ; }

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
