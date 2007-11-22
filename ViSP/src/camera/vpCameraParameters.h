/****************************************************************************
 *
 * $Id: vpCameraParameters.h,v 1.6 2007-11-22 08:57:11 asaunier Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Camera intrinsic parameters.
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/



/*!
  \file vpCameraParameters.h
  \brief Declaration of the vpCameraParameters class.
  Class vpCameraParameters define the camera intrinsic parameters

*/

#ifndef vpCAMERA_H
#define vpCAMERA_H

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>

/*!
  \class vpCameraParameters
  \brief Declaration of the vpCameraParameters class.
  Generic class defining the camera intrinsic parameters

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
    -for the meter based model
  \f[
  \left\{ \begin{array}{l}
  u = u_0 + p_x x +  \delta_u(x,y) \\
  v = v_0 + p_y y + \delta_v(x,y)
  \end{array}\right.
  \f]
    -for the pixel based model
  \f[
  \left\{ \begin{array}{l}
  u = u_0 + p_x x - \delta_u(u,v) \\
  v = v_0 + p_y y - \delta_v(u,v)
  \end{array}\right.
  \f]
  where  \f$\delta_u\f$ and \f$\delta_v\f$ are
  geometrical distortions introduced in  the camera model. These distortions are
  due to imperfections in the lenses design and assembly there usually are some
  positional errors that have to be taken into account.
  \f$\delta_u\f$ and \f$\delta_v\f$  can be modeled as follow:
    -for the meter based model
  \f[
  \left\{ \begin{array}{l}
  \delta_u(x,y) = K_d \;r^2\; p_x x  \\
  \\
  \delta_v(x,y) = K_d\; r^2\; p_y y
  \end{array}\right.
  \f]
  with \f$ r^2 = x^2 + y^2\f$.
    -for the pixel based model
  \f[
  \left\{ \begin{array}{l}
  \delta_u(u,v) = K_d \;r^2\; \tilde u  \\
  \\
  \delta_v(u,v) = K_d\; r^2\; \tilde v
  \end{array}\right.
  \f]
  with \f$\tilde u = u-u_0, \;\; \tilde v = (v-v_0)\f$ and
  \f$ r^2 = \frac{\tilde u^2}{p_{x}^2} +  \frac{\tilde v^2}{p_{y}^2} \f$.

*/
class VISP_EXPORT vpCameraParameters
{
public:
  static const double DEFAULT_U0_PARAMETER;
  static const double DEFAULT_V0_PARAMETER;
  static const double DEFAULT_PX_PARAMETER;
  static const double DEFAULT_PY_PARAMETER;
  static const double DEFAULT_KD_PARAMETER;

private:
 //model without disortion
  double px, py ; //!< pixel size
  double u0, v0 ; //!<  principal point
 //meter to pixel distortion model
  double px_mp, py_mp ; //!< pixel size
  double u0_mp, v0_mp ; //!<  principal point
  double kd_mp ; //!< radial distortion
 //pixel to meter distortion model
  double px_pm, py_pm ; //!< pixel size
  double u0_pm, v0_pm ;  //!<  principal point
  double kd_pm ; //!< radial distortion

  vpMatrix K ; //<! camera projection matrix

public:
  //generic functions
  vpCameraParameters() ;
  vpCameraParameters(const vpCameraParameters &c) ;
  vpCameraParameters(const double px, const double py,
		     const double u0, const double v0) ;

  void init() ;
  void init(const vpCameraParameters &c) ;
  void init(const double px, const double py,
	   const double u0, const double v0) ;
  void init_mp(const double px, const double py,
     const double u0, const double v0, const double kd=0) ;
  void init_pm(const double px, const double py,
     const double u0, const double v0, const double kd=0) ;


  virtual ~vpCameraParameters() ;

  void printParameters() ;

  vpCameraParameters& operator =(const vpCameraParameters& f) ;

  void setPrincipalPoint(const double u0, const double v0) ;
  void setPixelRatio(const double px,const double py) ;
  void computeMatrix() ;
  inline double get_px() const { return px; }
  inline double get_py() const { return py; }
  inline double get_u0() const { return u0; }
  inline double get_v0() const { return v0; }
  inline vpMatrix get_K() const {return K;}

  void setPrincipalPoint_mp(const double u0, const double v0) ;
  void setPixelRatio_mp(const double px,const double py) ;
  void setKd_mp(const double kd);
  inline double get_px_mp() const { return px_mp; }
  inline double get_py_mp() const { return py_mp; }
  inline double get_u0_mp() const { return u0_mp; }
  inline double get_v0_mp() const { return v0_mp; }
  inline double get_kd_mp() const {return kd_mp;}

  void setPrincipalPoint_pm(const double u0, const double v0) ;
  void setPixelRatio_pm(const double px,const double py) ;
  void setKd_pm(const double kd);
  inline double get_px_pm() const { return px_pm; }
  inline double get_py_pm() const { return py_pm; }
  inline double get_u0_pm() const { return u0_pm; }
  inline double get_v0_pm() const { return v0_pm; }
  inline double get_kd_pm() const {return kd_pm;}
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
