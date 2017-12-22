/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMatrix.h>

/*!
  \class vpCameraParameters

  \ingroup group_core_camera

  \brief Generic class defining intrinsic camera parameters.

  Let us define the pinhole camera model implemented in ViSP. In this model
  \cite Marchand16a, a scene view is formed by projecting 3D points into the
  image plane using a perspective transformation.

  \f[
  \left[ \begin{array}{c}
  u \\
  v \\
  1
  \end{array}\right] =
  \left[ \begin{array}{ccc}
  u_0 & 0   & p_x  \\
  0   & v_0 & p_y \\
  0   & 0   & 1
  \end{array}\right]
  \left[ \begin{array}{c}
  X_c  \\
  Y_c \\
  Z_c
  \end{array}\right]
  \f]

  where:

  - \f$(X_c,Y_c,Z_c)\f$ are the coordinates of a 3D point in the camera frame
  - \f$(u,v)\f$ are the coordinates in pixels of the projected 3D point
  - \f$(u_0,v_0)\f$ are the coordinates of the principal point (the
  intersection of the optical axes with the image plane) that is usually near
  the image center
  - \f$p_x\f$ (resp \f$p_y\f$) is the ratio between the focal length of the
  lens \f$f\f$ in meters and the size of the pixel \f$l_x\f$ in meters:
  \f$p_x=f/l_x\f$ (resp, \f$l_y\f$ being the height of a pixel,
  \f$p_y=f/l_y\f$).

  When \f$Z_c \neq 0\f$, the previous equation si equivalent to the following:
  \f[
  \begin{array}{lcl}
  x &=& X_c / Z_c \\
  y &=& Y_c / Z_c \\
  u &=& u_0 + x \; p_x \\
  v &=& v_0 + y \; p_y
  \end{array}
  \f]

  Real lenses usually have some radial distortion. So, the above model is
  extended as:

  \f[
  \begin{array}{lcl}
  x &=& X_c / Z_c \\
  y &=& Y_c / Z_c \\
  x^{'} &=& x (1 + k_{ud} r^2) \\
  y^{'} &=& y (1 + k_{ud} r^2) \\
  r^2 &=& x^2 + y^2 \\
  u &=& u_0 + x^{'} \; p_x \\
  v &=& v_0 + y^{'} \; p_y
  \end{array}
  \f]

  where \f$k_{ud}\f$ is the first order radial distorsion. Higher order
  distorsion coefficients are not considered in ViSP.

  Now in ViSP we consider also the inverse transformation, where from pixel
  coordinates we want to compute their normalized coordinates in the image
  plane. Previous equations could be written like:

  \f[
  \begin{array}{lcl}
  x &=& (u - u_0) / p_x \\
  y &=& (v - v_0) / p_y
  \end{array}
  \f]

  Considering radial distortion, the above model is extended as:
  \f[
  \begin{array}{lcl}
  (u-u_0)^{'} &=& (u-u_0) (1 + k_{du} r^2) \\
  (v-v_0)^{'} &=& (v-v_0) (1 + k_{du} r^2) \\
  r^2 &=& ((u-u_0)/p_x)^2 + ((v-v_0)/p_y)^2 \\
  x &=& (u - u_0)^{'} / p_x \\
  y &=& (v - v_0)^{'} / p_y
  \end{array}
  \f]

  Finally, in ViSP the main intrinsic camera parameters are \f$(p_x, p_y)\f$
  the ratio between the focal length and the size of a pixel, and \f$(u_0,
  v_0)\f$ the coordinates of the principal point in pixel. The lens
  distortion can also be considered by two additional parameters
  \f$(k_{ud}, k_{du})\f$.

  \note The \ref tutorial-calibration shows how to calibrate a camera
  to estimate the parameters corresponding to the model implemented in this
  class.

  \note Note also that \ref tutorial-bridge-opencv gives the correspondance
  between ViSP and OpenCV camera modelization.

  \note The conversion from pixel coordinates \f$(u,v)\f$ in the normalized
  space \f$(x,y)\f$ is implemented in vpPixelMeterConversion, whereas
  the conversion from normalized coordinates into pixel is implemented
  in vpMeterPixelConversion.

  From a practical point of view, two kinds of camera modelisation are
  implemented in this class:

  <b>1. Camera parameters for a perspective projection without distortion
  model</b>

  In this modelisation, only \f$u_0,v_0,p_x,p_y\f$ parameters are considered.

  Initialization of such a model can be done using:
  - initPersProjWithoutDistortion() that allows to set \f$u_0,v_0,p_x,p_y\f$
  parameters;
  - initFromFov() that computes the parameters from an image size and a camera
  field of view.

  <b>2. Camera parameters for a perspective projection with distortion
  model</b>

  In this modelisation, all the parameters \f$u_0,v_0,p_x,p_y,k_{ud},k_{du}\f$
  are considered. Initialization of such a model can be done using:
  - initPersProjWithDistortion() that allows to set
  \f$u_0,v_0,p_x,p_y,k_{ud},k_{du}\f$ parameters;

  The selection of the camera model (without or with distorsion) is done
  during vpCameraParameters initialisation.

  Here an example of camera initialisation, for a model without distortion. A
  complete example is given in initPersProjWithoutDistortion().

\code
  double px = 600; double py = 600; double u0 = 320; double v0 = 240;

  // Create a camera parameter container
  vpCameraParameters cam;
  // Camera initialization with a perspective projection without distortion
  // model
  cam.initPersProjWithoutDistortion(px,py,u0,v0);
  // It is also possible to print the current camera parameters
  std::cout << cam << std::endl;
\endcode

  Here an example of camera initialisation, for a model with distortion. A
  complete example is given in initPersProjWithDistortion().

\code
  double px = 600; double py = 600;
  double u0 = 320; double v0 = 240;
  double kud = -0.19; double kdu = 0.20;

  // Create a camera parameter container
  vpCameraParameters cam;

  // Camera initialization with a perspective projection without distortion
  model cam.initPersProjWithDistortion(px,py,u0,v0,kud,kdu);
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

public:
  typedef enum {
    perspectiveProjWithoutDistortion, //!< Perspective projection without
                                      //!< distortion model
    perspectiveProjWithDistortion     //!< Perspective projection with distortion
                                      //!< model
  } vpCameraParametersProjType;

  // generic functions
  vpCameraParameters();
  vpCameraParameters(const vpCameraParameters &c);
  vpCameraParameters(const double px, const double py, const double u0, const double v0);
  vpCameraParameters(const double px, const double py, const double u0, const double v0, const double kud,
                     const double kdu);

  vpCameraParameters &operator=(const vpCameraParameters &c);
  virtual ~vpCameraParameters();

  void init();
  void init(const vpCameraParameters &c);
  void initFromCalibrationMatrix(const vpMatrix &_K);
  void initFromFov(const unsigned int &w, const unsigned int &h, const double &hfov, const double &vfov);
  void initPersProjWithoutDistortion(const double px, const double py, const double u0, const double v0);
  void initPersProjWithDistortion(const double px, const double py, const double u0, const double v0, const double kud,
                                  const double kdu);

  /*!
    Specify if the fov has been computed.

    \sa computeFov()

    \return True if the fov has been computed, False otherwise.
  */
  inline bool isFovComputed() const { return isFov; }

  void computeFov(const unsigned int &w, const unsigned int &h);

  /*!
    Get the horizontal angle in radian of the field of view.

    \return FOV horizontal angle computed with px and width.

    \sa computeFov(), getVerticalFovAngle()
  */
  inline double getHorizontalFovAngle() const
  {
    if (!isFov)
      vpTRACE("Warning: The FOV is not computed, getHorizontalFovAngle() "
              "won't be significant.");
    return m_hFovAngle;
  }

  /*!
    Get the vertical angle in radian of the field of view.

    \return FOV vertical angle computed with py and height.

    \sa computeFov(), getHorizontalFovAngle()
  */
  inline double getVerticalFovAngle() const
  {
    if (!isFov)
      vpTRACE("Warning: The FOV is not computed, getVerticalFovAngle() won't "
              "be significant.");
    return m_vFovAngle;
  }

  /*!
    Get the list of the normals corresponding to planes describing the field
    of view.
      - vector[0] : Left Normal.
      - vector[1] : Right Normal.
      - vector[2] : Up Normal.
      - vector[3] : Down Normal.

    \sa computeFov()

    \return List of the normals.
  */
  inline std::vector<vpColVector> getFovNormals() const
  {
    if (!isFov)
      vpTRACE("Warning: The FOV is not computed, getFovNormals() won't be "
              "significant.");
    return fovNormals;
  }

  inline double get_px() const { return px; }
  inline double get_px_inverse() const { return inv_px; }
  inline double get_py_inverse() const { return inv_py; }
  inline double get_py() const { return py; }
  inline double get_u0() const { return u0; }
  inline double get_v0() const { return v0; }
  inline double get_kud() const { return kud; }
  inline double get_kdu() const { return kdu; }

  inline vpCameraParametersProjType get_projModel() const { return projModel; }

  vpMatrix get_K() const;
  vpMatrix get_K_inverse() const;

  void printParameters();
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpCameraParameters &cam);

private:
  static const double DEFAULT_U0_PARAMETER;
  static const double DEFAULT_V0_PARAMETER;
  static const double DEFAULT_PX_PARAMETER;
  static const double DEFAULT_PY_PARAMETER;
  static const double DEFAULT_KUD_PARAMETER;
  static const double DEFAULT_KDU_PARAMETER;
  static const vpCameraParametersProjType DEFAULT_PROJ_TYPE;

  double px, py; //!< pixel size
  double u0, v0; //!<  principal point
  double kud;    //!< radial distortion (from undistorted to distorted)
  double kdu;    //!< radial distortion (from distorted to undistorted)

  unsigned int width;                  //!< Width of the image used for the fov computation
  unsigned int height;                 //!< Height of the image used for the fov computation
  bool isFov;                          //!< Boolean to specify if the fov has been computed
  double m_hFovAngle;                  //!< Field of view horizontal angle
  double m_vFovAngle;                  //!< Field of view vertical angle
  std::vector<vpColVector> fovNormals; //!< Normals of the planes describing the fov

  double inv_px, inv_py;

  vpCameraParametersProjType projModel; //!< used projection model
};

#endif
