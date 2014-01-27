/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#ifndef vpCameraParameters_H
#define vpCameraParameters_H

#include <vector>

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>

/*!
  \class vpCameraParameters

  \ingroup CameraModelTransformation CameraModel

  \brief Generic class defining intrinsic camera parameters.

  Two kinds of camera modelisation are implemented:
  - Camera parameters for a perspective projection without distortion model,
  - Camera parameters for a perspective projection with distortion model.

  The main intrinsic camera parameters are \f$(p_x, p_y)\f$ the ratio
  between the focal length and the size of a pixel, and \f$(u_0,
  v_0)\f$ the coordinates of the principal point in pixel. The lens
  distortion can also be considered by two additional parameters
  \f$(k_{ud}, k_{du})\f$.

  <b>1. Camera parameters for a perspective projection without distortion model</b>

  In this modelisation, only \f$u_0,v_0,p_x,p_y\f$ parameters are
  used.  If we denote \f$(u,v)\f$ the position of a pixel in the
  digitized image, this position is related to the corresponding
  coordinates \f$(x,y)\f$ in the normalized space (in meter) by:

  \f[
  \left\{ \begin{array}{l}
  u = u_0 + p_x x  \\
  v = v_0 + p_y y
  \end{array}\right.
  \f]

  The conversion from pixel coordinates \f$(u,v)\f$ in the normalized
  space \f$(x,y)\f$ is implemented in vpPixelMeterConversion, whereas
  the conversion from normalized coordinates into pixel is implemented
  in vpMeterPixelConversion.

  <b>2. Camera parameters for a perspective projection with distortion model</b>

  In this modelisation, \f$u_0,v_0,p_x,p_y,k_{ud},k_{du}\f$
  parameters are used.  If a model with distortion is considered, we
  got:

  \f[
  \left\{ \begin{array}{l}
  u = u_0 + p_x x +  \delta_u \\
  v = v_0 + p_y y + \delta_v
  \end{array}\right.
  \f]

  where  \f$\delta_u\f$ and \f$\delta_v\f$ are
  geometrical distortions introduced in the camera model. These distortions are
  due to imperfections in the lenses design and assembly there usually are some
  positional errors that have to be taken into account.
  \f$\delta_u\f$ and \f$\delta_v\f$  can be modeled as follow:
  - with an undistorted to distorted model
  \f[
  \left\{ \begin{array}{l}
  \delta_u(x,y) = k_{ud} \;r^2\; p_x x  \\
  \\
  \delta_v(x,y) = k_{ud}\; r^2\; p_y y
  \end{array}\right.
  \f]
  with \f[ r^2 = x^2 + y^2 \f]
  This model is useful to convert meter to pixel coordinates because in this
  case :
  \f[
  \left\{ \begin{array}{l}
  u = f_u(x,y) =  u_0 + p_x x\left(1+k_{ud}\left(x^2 + y^2\right)\right)  \\
  \\
  v = f_v(x,y) =  v_0 + p_y y\left(1+k_{ud}\left(x^2 + y^2\right)\right)
  \end{array}\right.
  \f]
  The conversion from normalized coordinates \f$(x,y)\f$ into pixel
  \f$(u,v)\f$ is implemented in vpMeterPixelConversion.

  - or with a distorted to undistorted model
  \f[
  \left\{ \begin{array}{l}
  \delta_u(u,v) = -k_{du} \;r^2\; \left(u-u_0\right)  \\
  \\
  \delta_v(u,v) = -k_{du}\; r^2\; \left(v-v_0\right)
  \end{array}\right.
  \f]
  with \f[ r^2 = {\left(\frac{u-u_0}{p_{x}}\right)}^2 +  {\left(\frac{v-v_0}{p_{y}}\right)}^2 \f]
  This model is useful to convert pixel to meter coordinates because in this
  case :
  \f[
  \left\{ \begin{array}{l}
  x = f_x(u,v) =  \frac{u-u_0}{p_x}\left(1+k_{du}\left( {\left(\frac{u-u_0}{p_{x}}\right)}^2 +  {\left(\frac{v-v_0}{p_{y}}\right)}^2 \right)\right)  \\
  \\
  y = f_y(u,v) =  \frac{v-v_0}{p_y}\left(1+k_{du}\left( {\left(\frac{u-u_0}{p_{x}}\right)}^2 +  {\left(\frac{v-v_0}{p_{y}}\right)}^2 \right)\right)
  \end{array}\right.
  \f]
  The conversion from pixel coordinates \f$(u,v)\f$ in the normalized
  space \f$(x,y)\f$ is implemented in vpPixelMeterConversion.

  The selection of one of these modelisations is done during
  vpCameraParameters initialisation.

  Here an example of camera initialisation, for a model without distortion:
  \code
  double px = 600;
  double py = 600;
  double u0 = 320;
  double v0 = 240;

  // Create a camera parameter container
  vpCameraParameters cam;
  // Camera initialization with a perspective projection without distortion model
  cam.initPersProjWithoutDistortion(px,py,u0,v0);
  // It is also possible to print the current camera parameters
  std::cout << cam << std::endl;
  \endcode

  Here an example of camera initialisation, for a model with distortion:
  \code
  double px = 600;
  double py = 600;
  double u0 = 320;
  double v0 = 240;
  double kud = -0.19;
  double kdu = 0.20;

  // Create a camera parameter container
  vpCameraParameters cam;

  // Camera initialization with a perspective projection without distortion model
  cam.initPersProjWithDistortion(px,py,u0,v0,kud,kdu);
  \endcode

  The code below shows how to know the currently used projection model:
  \code
  vpCameraParameters cam;
  ...
  vpCameraParameters::vpCameraParametersProjType projModel;
  projModel = cam.get_projModel(); // Get the projection model type
  \endcode

  An XML parser for camera parameters is also provided in vpXmlParserCamera.
*/
class VISP_EXPORT vpCameraParameters
{
  friend class vpMeterPixelConversion;
  friend class vpPixelMeterConversion;
public :
  typedef enum{
    perspectiveProjWithoutDistortion, //!< Perspective projection without distortion model
    perspectiveProjWithDistortion  //!< Perspective projection with distortion model
  } vpCameraParametersProjType ;
  

  //generic functions
  vpCameraParameters() ;
  vpCameraParameters(const vpCameraParameters &c) ;
  vpCameraParameters(const double px, const double py,
		     const double u0, const double v0) ;
  vpCameraParameters(const double px, const double py,
                     const double u0, const double v0,
                     const double kud, const double kdu) ;

  vpCameraParameters& operator =(const vpCameraParameters &c) ;
  virtual ~vpCameraParameters() ;

  void init() ;
  void init(const vpCameraParameters &c) ;
  void initFromCalibrationMatrix(const vpMatrix& _K);
  
  void initPersProjWithoutDistortion(const double px, const double py,
                                      const double u0, const double v0) ;
  void initPersProjWithDistortion(const double px, const double py,
     const double u0, const double v0, const double kud,const double kdu) ;
     
  /*!
    Specify if the fov has been computed.
    
    \sa computeFov()
    
    \return True if the fov has been computed, False otherwise.
  */
  inline bool isFovComputed() const { return isFov; }
     
  void computeFov(const unsigned int &w, const unsigned int &h);
  
  /*!
    Get the horizontal angle of the field of view.
    
    \sa computeFov()
    
    \return AngleX computed with px and width.
  */
  inline double getFovAngleX() const { 
    if(!isFov) vpTRACE("Warning: The FOV is not computed, getFovAngleX() won't be significant.");
    return fovAngleX; 
  }
  
  /*!
    Get the vertical angle of the field of view.
    
    \sa computeFov()
    
    \return AngleY computed with py and height.
  */
  inline double getFovAngleY() const { 
    if(!isFov) vpTRACE("Warning: The FOV is not computed, getFovAngleY() won't be significant.");
    return fovAngleY; 
  }
  
  /*!
    Get the list of the normals corresponding to planes describing the field of view.
      - vector[0] : Left Normal.
      - vector[1] : Right Normal.
      - vector[2] : Up Normal.
      - vector[3] : Down Normal.
      
    \sa computeFov()
    
    \return List of the normals.
  */
  inline std::vector<vpColVector> getFovNormals() const { 
    if(!isFov) vpTRACE("Warning: The FOV is not computed, getFovNormals() won't be significant.");
    return fovNormals; 
  }
  
  inline double get_px() const { return px; }
  inline double get_px_inverse() const {return inv_px; }
  inline double get_py_inverse() const {return inv_py; }
  inline double get_py() const { return py; }
  inline double get_u0() const { return u0; }
  inline double get_v0() const { return v0; }
  inline double get_kud() const { return kud; }
  inline double get_kdu() const { return kdu; }
   
  inline vpCameraParametersProjType get_projModel() const {
    return projModel; 
  } 
  
  vpMatrix get_K() const;
  vpMatrix get_K_inverse() const;

  void printParameters() ;
  friend VISP_EXPORT std::ostream & operator << (std::ostream & os, const vpCameraParameters &cam);

private:
  static const double DEFAULT_U0_PARAMETER;
  static const double DEFAULT_V0_PARAMETER;
  static const double DEFAULT_PX_PARAMETER;
  static const double DEFAULT_PY_PARAMETER;
  static const double DEFAULT_KUD_PARAMETER;
  static const double DEFAULT_KDU_PARAMETER;
  static const vpCameraParametersProjType DEFAULT_PROJ_TYPE; 


  double px, py ; //!< pixel size
  double u0, v0 ; //!<  principal point
  double kud ; //!< radial distortion (from undistorted to distorted)
  double kdu ; //!< radial distortion (from distorted to undistorted)
  
  unsigned int width ; //!< Width of the image used for the fov computation
  unsigned int height ; //!< Height of the image used for the fov computation
  bool isFov ; //!< Boolean to specify if the fov has been computed
  double fovAngleX ; //!< AngleX/2.0 of the fov
  double fovAngleY ; //!< AngleY/2.0 of the fov
  std::vector<vpColVector> fovNormals ; //!< Normals of the planes describing the fov
  
  double inv_px, inv_py; 
   
  vpCameraParametersProjType projModel ; //!< used projection model
} ;

#endif
