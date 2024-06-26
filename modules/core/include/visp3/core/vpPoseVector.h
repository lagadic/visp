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
 * Pose object. A pose is a size 6 vector [t, tu]^T where tu is
 * a rotation vector (theta u representation) and t is a translation vector.
 */

/*!
  \file vpPoseVector.h

  \brief Pose representation. A pose is a 6 dimension vector [t,tu]^T
    where tu is a rotation vector (theta u representation) and t is a
    translation vector.
*/

#ifndef VP_POSE_VECTOR_H
#define VP_POSE_VECTOR_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRotationMatrix.h>

BEGIN_VISP_NAMESPACE
class vpRotationMatrix;
class vpHomogeneousMatrix;
class vpTranslationVector;
class vpThetaUVector;
class vpRowVector;

/*!
  \class vpPoseVector

  \ingroup group_core_transformations

  \brief Implementation of a pose vector and operations on poses.

  The vpPose class implements a complete representation of every rigid motion
  in the Euclidean space.

  The vpPose class is derived from vpArray2D<double>.

  The pose is composed of a translation and a rotation
  minimally represented by a 6 dimension pose vector as: \f[ ^{a}{\bf
  r}_b = [^{a}{\bf t}_{b},\theta {\bf u}]^\top \in R^6\f]

  where \f$ ^{a}{\bf r}_b \f$ is the pose from frame \f$ a \f$ to
  frame \f$ b \f$, with \f$ ^{a}{\bf t}_{b} \f$ being the translation
  vector between these frames along the x,y,z
  axis and \f$\theta \bf u \f$, the axis-angle representation of the
  rotation \f$^{a}\bf{R}_{b}\f$ between these frames.

  Translations are expressed in meters, while the angles in the \f$\theta {\bf
  u}\f$ axis-angle representation are expressed in radians.

  To know more about the \f$\theta \bf u\f$ rotation representation,
  see vpThetaUVector documentation.

  The following code shows how to initialize a pose vector:
  \code
  #include <visp3/core/vpPoseVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpPoseVector pose;

    pose[0] = 0.1;    // tx
    pose[1] = 0.2;    // ty
    pose[2] = 0.3;    // tz

    pose[3] = M_PI;   // tux
    pose[4] = M_PI_2; // tux
    pose[5] = M_PI_4; // tuz

    std::cout << "pose vector:\n" << pose << std::endl;
  }
  \endcode
  It produces the following printings:
  \code{.unparsed}
  pose vector:
  0.1
  0.2
  0.3
  3.141592654
  1.570796327
  0.7853981634
  \endcode
  The same initialization could be achieved this way:
  \code
  #include <visp3/core/vpPoseVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector t;
    vpThetaUVector tu;

    t << 0.1, 0.2, 0.3;
    tu << M_PI, M_PI_2, M_PI_4;
    vpPoseVector pose(t, tu);
  }
  \endcode
  If ViSP is build with c++11 support, you could also initialize the vector using:
  \code
  #include <visp3/core/vpPoseVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector t;
    vpThetaUVector tu;
    t = { 0.1, 0.2, 0.3 };
    tu = { M_PI, M_PI_2, M_PI_4 };
    vpPoseVector pose(t, tu);
  }
  \endcode

  <b>JSON serialization</b>

  Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpPoseVector.
  The following sample code shows how to save a pose vector in a file named `pose-vector.json`
  and reload the values from this JSON file.
  \code
  #include <visp3/core/vpPoseVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_NLOHMANN_JSON)
    std::string filename = "pose-vector.json";
    {
      vpPoseVector pose(0.1, 0.2, 0.3, M_PI, M_PI_2, M_PI_4);
      std::ofstream file(filename);
      const nlohmann::json j = pose;
      file << j;
      file.close();
    }
    {
      std::ifstream file(filename);
      const nlohmann::json j = nlohmann::json::parse(file);
      vpPoseVector pose;
      pose = j;
      file.close();
      std::cout << "Read pose vector from " << filename << ":\n" << pose.t() << std::endl;
    }
  #endif
  }
  \endcode
  If you build and execute the sample code, it will produce the following output:
  \code{.unparsed}
  Read pose vector from pose-vector.json:
  0.1  0.2  0.3  3.141592654  1.570796327  0.7853981634
  \endcode

  The content of the `pose-vector.json` file is the following:
  \code{.unparsed}
  $ cat pose-vector.json
  {"cols":1,"data":[0.1,0.2,0.3,3.141592653589793,1.5707963267948966,0.7853981633974483],"rows":6,"type":"vpPoseVector"}
  \endcode
*/
class VISP_EXPORT vpPoseVector : public vpArray2D<double>
{
public:
  // constructor
  vpPoseVector();
  // constructor from 3 angles (in radian)
  vpPoseVector(double tx, double ty, double tz, double tux, double tuy, double tuz);
  // constructor convert an homogeneous matrix in a pose
  VP_EXPLICIT vpPoseVector(const vpHomogeneousMatrix &M);
  // constructor  convert a translation and a "thetau" vector into a pose
  vpPoseVector(const vpTranslationVector &tv, const vpThetaUVector &tu);
  // constructor  convert a translation and a rotation matrix into a pose
  vpPoseVector(const vpTranslationVector &tv, const vpRotationMatrix &R);

  virtual ~vpPoseVector() { }

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  VP_DEPRECATED vpPoseVector buildFrom(double tx, double ty, double tz, double tux, double tuy, double tuz);
  // convert an homogeneous matrix in a pose
  VP_DEPRECATED vpPoseVector buildFrom(const vpHomogeneousMatrix &M);
  //  convert a translation and a "thetau" vector into a pose
  VP_DEPRECATED vpPoseVector buildFrom(const vpTranslationVector &tv, const vpThetaUVector &tu);
  //  convert a translation and a rotation matrix into a pose
  VP_DEPRECATED vpPoseVector buildFrom(const vpTranslationVector &tv, const vpRotationMatrix &R);
#endif

  vpPoseVector &build(const double &tx, const double &ty, const double &tz, const double &tux, const double &tuy, const double &tuz);
  // convert an homogeneous matrix in a pose
  vpPoseVector &build(const vpHomogeneousMatrix &M);
  //  convert a translation and a "thetau" vector into a pose
  vpPoseVector &build(const vpTranslationVector &tv, const vpThetaUVector &tu);
  //  convert a translation and a rotation matrix into a pose
  vpPoseVector &build(const vpTranslationVector &tv, const vpRotationMatrix &R);

  void extract(vpRotationMatrix &R) const;
  void extract(vpThetaUVector &tu) const;
  void extract(vpTranslationVector &tv) const;
  void extract(vpQuaternionVector &q) const;

  vpRotationMatrix getRotationMatrix() const;
  vpThetaUVector getThetaUVector() const;
  vpTranslationVector getTranslationVector() const;

  // Load an homogeneous matrix from a file
  void load(std::ifstream &f);

  /*!
    Set the value of an element of the pose vector: r[i] = x.

    \param i : Pose vector element index

    \code
    // Create a pose vector with translation and rotation set to zero
    vpPoseVector r;

    // Initialize the pose vector
    r[0] = 1;
    r[1] = 2;
    r[2] = 3;
    r[3] = M_PI;
    r[4] = -M_PI;
    r[5] = 0;
    \endcode

    This code produces the same effect:
    \code
    vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);
    \endcode

  */
  inline double &operator[](unsigned int i) { return *(data + i); }
  /*!
    Get the value of an element of the pose vector: x = r[i].

    \param i : Pose vector element index

    \code
    vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);

    double tx,ty,tz; // Translation
    double tux, tuy,tuz; // Theta u rotation
    tx  = r[0];
    ty  = r[1];
    tz  = r[2];
    tux = r[3];
    tuy = r[4];
    tuz = r[5];
    \endcode
  */
  inline const double &operator[](unsigned int i) const { return *(data + i); }

  // Print  a vector [T thetaU] thetaU in degree
  void print() const;
  int print(std::ostream &s, unsigned int length, char const *intro = 0) const;

  /*!
    This function is not applicable to a pose vector that is always a
    6-by-1 column vector.
    \exception vpException::fatalError When this function is called.
  */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize a pose vector"));
  }

  // Save an homogeneous matrix in a file
  void save(std::ofstream &f) const;
  void set(double tx, double ty, double tz, double tux, double tuy, double tuz);
  vpRowVector t() const;

  std::vector<double> toStdVector() const;

#ifdef VISP_HAVE_NLOHMANN_JSON
public:
  static const std::string jsonTypeName;
private:
  friend void to_json(nlohmann::json &j, const vpPoseVector &cam);
  friend void from_json(const nlohmann::json &j, vpPoseVector &cam);
  // Conversion helper function to avoid circular dependencies and MSVC errors that are not exported in the DLL
  void parse_json(const nlohmann::json &j);
  void convert_to_json(nlohmann::json &j) const;
public:
#endif

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Provided only for compat with previous releases.
     This function does nothing.
  */
  VP_DEPRECATED void init() { };
  //@}
#endif
};

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
inline void to_json(nlohmann::json &j, const vpPoseVector &r)
{
  r.convert_to_json(j);
}

inline void from_json(const nlohmann::json &j, vpPoseVector &r)
{
  r.parse_json(j);
}
#endif
END_VISP_NAMESPACE
#endif
