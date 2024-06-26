/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

 /*!
    \file vpCameraParameters.h
    \brief Declaration of the vpCameraParameters class.
    Class vpCameraParameters define the camera intrinsic parameters

 */

#ifndef VP_CAMERA_PARAMETERS_H
#define VP_CAMERA_PARAMETERS_H

#include <iostream>
#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include<nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \class vpCameraParameters

  \ingroup group_core_camera

  \brief Generic class defining intrinsic camera parameters.

  <b>1. Supported camera models</b>

  Two camera models are implemented in ViSP.

  <b>1.1. Pinhole camera model</b>

  In this model \cite Marchand16a, a scene view is formed by projecting 3D points
  into the image plane using a perspective transformation.

  \f[
  \left[ \begin{array}{c}
  u \\
  v \\
  1
  \end{array}\right] =
  \left[ \begin{array}{ccc}
  p_x & 0   & u_0  \\
  0   & p_y & v_0 \\
  0   & 0   & 1
  \end{array}\right]
  \left[ \begin{array}{c}
  x  \\
  y   \\
  1
  \end{array}\right]
  \f]

  where:

  - \f$(X_c,Y_c,Z_c)\f$ are the coordinates of a 3D point in the camera frame
  - \f$(x,y)\f$ are the coordinates of the projection of the 3D point in the image plane
  - \f$(u,v)\f$ are the coordinates in pixels of the projected 3D point
  - \f$(u_0,v_0)\f$ are the coordinates of the principal point (the
  intersection of the optical axes with the image plane) that is usually near
  the image center
  - \f$p_x\f$ (resp \f$p_y\f$) is the ratio between the focal length of the
  lens \f$f\f$ in meters and the size of the pixel \f$l_x\f$ in meters:
  \f$p_x=f/l_x\f$ (resp, \f$l_y\f$ being the height of a pixel,
  \f$p_y=f/l_y\f$).

  When \f$Z_c \neq 0\f$, the previous equation is equivalent to the following:
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

  where \f$k_{ud}\f$ is the first order radial distortion. Higher order
  distortion coefficients are not considered in ViSP.

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

  \note The \ref tutorial-calibration-intrinsic shows how to calibrate a camera
  to estimate the parameters corresponding to the model implemented in this
  class.

  \note Note also that \ref tutorial-bridge-opencv gives the correspondence
  between ViSP and OpenCV camera modelization.

  \note The conversion from pixel coordinates \f$(u,v)\f$ in the normalized
  space \f$(x,y)\f$ is implemented in vpPixelMeterConversion, whereas
  the conversion from normalized coordinates into pixel is implemented
  in vpMeterPixelConversion.

  From a practical point of view, two kinds of camera modelization are
  implemented in this class:

  <b>1.1.1. Camera parameters for a perspective projection without distortion
  model</b>

  In this modelization, only \f$u_0,v_0,p_x,p_y\f$ parameters are considered.

  Initialization of such a model can be done using:
  - initPersProjWithoutDistortion() that allows to set \f$u_0,v_0,p_x,p_y\f$
  parameters;
  - initFromFov() that computes the parameters from an image size and a camera
  field of view.

  <b>1.1.2. Camera parameters for a perspective projection with distortion
  model</b>

  In this modelization, all the parameters \f$u_0,v_0,p_x,p_y,k_{ud},k_{du}\f$
  are considered. Initialization of such a model can be done using:
  - initPersProjWithDistortion() that allows to set
  \f$u_0,v_0,p_x,p_y,k_{ud},k_{du}\f$ parameters;

  The selection of the camera model (without or with distortion) is done
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

  <b>1.2. Kannala-Brandt camera model</b>

  This model \cite KannalaBrandt deals with fish-eye lenses designed to cover
  the whole hemispherical field in front of the camera and the angle of view
  is very large. In this case, the inherent distortion of a fish-eye lens should
  not be considered only as a derivation from the pinhole model.

  The following projection in the general form is adapted:

  \f[
  \begin{array}{lcl}
  r(\theta) &=& k_1 \theta + k_2 \theta^3 + k_3 \theta^5 + k_4 \theta^7 + k_5 \theta^9
  \end{array}
  \f]

  where:
  - \f$\theta\f$ is the angle in rad between a point in the real world and the
  optical axis.
  - \f$r\f$ is the distance between the image point and the principal point.

  In ViSP, we only consider radially symmetric distortions (caused by fisheye lenses).

  <b>2. JSON serialization</b>

  Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpCameraParameters.
  The following sample code shows how to save camera parameters in a file named `cam.json`
  and reload the parameters from this JSON file.
  \code
  #include <visp3/core/vpCameraParameters.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_NLOHMANN_JSON)
    std::string filename = "cam.json";
    {
      // Save camera parameters in a JSON file
      vpCameraParameters cam(801, 802, 325, 245);
      std::ofstream file(filename);
      const nlohmann::json j = cam;
      file << j;
      file.close();
    }
    {
      // Load camera parameters from a JSON file
      std::ifstream file(filename);
      const nlohmann::json j = nlohmann::json::parse(file);
      vpCameraParameters cam;
      cam = j;
      file.close();
      std::cout << "Read camera parameters from " << filename << ":\n" << cam << std::endl;
    }
  #endif
  }
  \endcode
  If you build and execute the sample code, it will produce the following output:
  \code{.unparsed}
  Read camera parameters from cam.json:
  Camera parameters for perspective projection without distortion:
    px = 801   py = 802
    u0 = 325   v0 = 245
  \endcode

  The content of the `cam.json` file is the following:
  \code{.unparsed}
  $ cat cam.json
  {"model":"perspectiveWithoutDistortion","px":801.0,"py":802.0,"u0":325.0,"v0":245.0}
  \endcode
*/

class VISP_EXPORT vpCameraParameters
{
  friend class vpMeterPixelConversion;
  friend class vpPixelMeterConversion;

public:
  typedef enum
  {
    perspectiveProjWithoutDistortion, //!< Perspective projection without distortion model
    perspectiveProjWithDistortion,    //!< Perspective projection with distortion model
    ProjWithKannalaBrandtDistortion   //!< Projection with Kannala-Brandt distortion model
  } vpCameraParametersProjType;

  // generic functions
  vpCameraParameters();
  vpCameraParameters(const vpCameraParameters &c);
  vpCameraParameters(double px, double py, double u0, double v0);
  vpCameraParameters(double px, double py, double u0, double v0, double kud, double kdu);
  vpCameraParameters(double px, double py, double u0, double v0, const std::vector<double> &distortion_coefficients);

  vpCameraParameters &operator=(const vpCameraParameters &c);
  bool operator==(const vpCameraParameters &c) const;
  bool operator!=(const vpCameraParameters &c) const;
  virtual ~vpCameraParameters();

  void init();
  void init(const vpCameraParameters &c);
  void initFromCalibrationMatrix(const vpMatrix &_K);
  void initFromFov(const unsigned int &w, const unsigned int &h, const double &hfov, const double &vfov);
  void initPersProjWithoutDistortion(double px, double py, double u0, double v0);
  void initPersProjWithDistortion(double px, double py, double u0, double v0, double kud, double kdu);
  void initProjWithKannalaBrandtDistortion(double px, double py, double u0, double v0,
    const std::vector<double> &distortion_coefficients);

  /*!
    Specify if the fov has been computed.

    \sa computeFov()

    \return True if the fov has been computed, False otherwise.
  */
  inline bool isFovComputed() const { return m_isFov; }

  void computeFov(const unsigned int &w, const unsigned int &h);

  /*!
    Get the horizontal angle in radian of the field of view.

    \return FOV horizontal angle computed with px and width.

    \sa computeFov(), getVerticalFovAngle()
  */
  inline double getHorizontalFovAngle() const
  {
    if (!m_isFov) {
      std::cout << "Warning: The FOV is not computed, getHorizontalFovAngle() won't be significant." << std::endl;
    }
    return m_hFovAngle;
  }

  /*!
    Get the vertical angle in radian of the field of view.

    \return FOV vertical angle computed with py and height.

    \sa computeFov(), getHorizontalFovAngle()
  */
  inline double getVerticalFovAngle() const
  {
    if (!m_isFov) {
      std::cout << "Warning: The FOV is not computed, getVerticalFovAngle() won't be significant." << std::endl;
    }
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
    if (!m_isFov) {
      std::cout << "Warning: The FOV is not computed, getFovNormals() won't be significant." << std::endl;
    }
    return m_fovNormals;
  }

  inline double get_px() const { return m_px; }
  inline double get_px_inverse() const { return m_inv_px; }
  inline double get_py_inverse() const { return m_inv_py; }
  inline double get_py() const { return m_py; }
  inline double get_u0() const { return m_u0; }
  inline double get_v0() const { return m_v0; }
  inline double get_kud() const { return m_kud; }
  inline double get_kdu() const { return m_kdu; }
  inline std::vector<double> getKannalaBrandtDistortionCoefficients() const { return m_dist_coefs; }

  inline vpCameraParametersProjType get_projModel() const { return m_projModel; }

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

  double m_px, m_py;                     //!< Pixel size
  double m_u0, m_v0;                     //!< Principal point
  double m_kud;                          //!< Radial distortion (from undistorted to distorted)
  double m_kdu;                          //!< Radial distortion (from distorted to undistorted)
  std::vector<double> m_dist_coefs;      //!< Coefficients for Kannala-Brandt distortion model

  unsigned int m_width;                  //!< Width of the image used for the fov computation
  unsigned int m_height;                 //!< Height of the image used for the fov computation
  bool m_isFov;                          //!< Boolean to specify if the fov has been computed
  double m_hFovAngle;                    //!< Field of view horizontal angle
  double m_vFovAngle;                    //!< Field of view vertical angle
  std::vector<vpColVector> m_fovNormals; //!< Normals of the planes describing the fov

  double m_inv_px, m_inv_py;

  vpCameraParametersProjType m_projModel; //!< used projection model
#ifdef VISP_HAVE_NLOHMANN_JSON
  friend void to_json(nlohmann::json &j, const vpCameraParameters &cam);
  friend void from_json(const nlohmann::json &j, vpCameraParameters &cam);
#endif
};

#ifdef VISP_HAVE_NLOHMANN_JSON
#include<nlohmann/json.hpp>
NLOHMANN_JSON_SERIALIZE_ENUM(vpCameraParameters::vpCameraParametersProjType, {
    {vpCameraParameters::perspectiveProjWithoutDistortion, "perspectiveWithoutDistortion"},
    {vpCameraParameters::perspectiveProjWithDistortion, "perspectiveWithDistortion"},
    {vpCameraParameters::ProjWithKannalaBrandtDistortion, "kannalaBrandtDistortion"}
  });

/**
 * \brief Converts camera parameters into a JSON representation.
 * \sa from_json() for more information on the content.
 * \param j The resulting JSON object.
 * \param cam The camera to serialize.
 */
inline void to_json(nlohmann::json &j, const vpCameraParameters &cam)
{
  j["px"] = cam.m_px;
  j["py"] = cam.m_py;
  j["u0"] = cam.m_u0;
  j["v0"] = cam.m_v0;
  j["model"] = cam.m_projModel;

  switch (cam.m_projModel) {
  case vpCameraParameters::perspectiveProjWithDistortion:
  {
    j["kud"] = cam.m_kud;
    j["kdu"] = cam.m_kdu;
    break;
  }
  case vpCameraParameters::ProjWithKannalaBrandtDistortion:
  {
    j["dist_coeffs"] = cam.m_dist_coefs;
    break;
  }
  case vpCameraParameters::perspectiveProjWithoutDistortion:
    break;
  default:
    break;
  }
}

/*!
 * \brief Deserialize a JSON object into camera parameters.
 * The minimal required properties are:
 * - Pixel size: px, py
 * - Principal point: u0, v0
 *
 * If a projection model (\ref vpCameraParameters::vpCameraParametersProjType) is supplied, then other parameters may be expected:
 * - In the case of perspective projection with distortion, ku, and kud must be supplied.
 * - In the case of Kannala-Brandt distortion, the list of coefficients must be supplied.
 *
 * An example of a JSON object representing a camera is:
 * \code{.json}
 *     {
 *       "px": 300.0,
 *       "py": 300.0,
 *       "u0": 120.5,
 *       "v0": 115.0,
 *       "model": "perspectiveWithDistortion", // one of ["perspectiveWithoutDistortion", "perspectiveWithDistortion", "kannalaBrandtDistortion"]. If omitted, camera is assumed to have no distortion
 *       "kud": 0.5, // required since "model" == perspectiveWithDistortion
 *       "kdu": 0.5
 *     }
 * \endcode
 *
 * \param j The json object to deserialize.
 * \param cam The modified camera.
 */
inline void from_json(const nlohmann::json &j, vpCameraParameters &cam)
{
  const double px = j.at("px").get<double>();
  const double py = j.at("py").get<double>();
  const double u0 = j.at("u0").get<double>();
  const double v0 = j.at("v0").get<double>();
  const vpCameraParameters::vpCameraParametersProjType model = j.value("model", vpCameraParameters::perspectiveProjWithoutDistortion);

  switch (model) {
  case vpCameraParameters::perspectiveProjWithoutDistortion:
  {
    cam.initPersProjWithoutDistortion(px, py, u0, v0);
    break;
  }
  case vpCameraParameters::perspectiveProjWithDistortion:
  {
    const double kud = j.at("kud").get<double>();
    const double kdu = j.at("kdu").get<double>();
    cam.initPersProjWithDistortion(px, py, u0, v0, kud, kdu);
    break;
  }
  case vpCameraParameters::ProjWithKannalaBrandtDistortion:
  {
    const std::vector<double> coeffs = j.at("dist_coeffs").get<std::vector<double>>();
    cam.initProjWithKannalaBrandtDistortion(px, py, u0, v0, coeffs);
    break;
  }
  default:
    break;
  }
}
#endif
END_VISP_NAMESPACE
#endif
