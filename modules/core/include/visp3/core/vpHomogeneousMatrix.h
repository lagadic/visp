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
 * Homogeneous matrix.
 */

/*!
  \file vpHomogeneousMatrix.h
  \brief Definition and computation on the homogeneous matrices
 */

#ifndef VP_HOMOGENEOUS_MATRIX_H
#define VP_HOMOGENEOUS_MATRIX_H

#include <fstream>
#include <vector>

#include <visp3/core/vpConfig.h>

BEGIN_VISP_NAMESPACE

class vpTranslationVector;
class vpPoseVector;
class vpMatrix;
class vpRotationMatrix;
class vpPoseVector;
class vpThetaUVector;
class vpQuaternionVector;
class vpPoint;

END_VISP_NAMESPACE

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpPoseVector.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
  \class vpHomogeneousMatrix

  \ingroup group_core_transformations

  \brief Implementation of an homogeneous matrix and operations on such kind
  of matrices.

  The class provides a data structure for the homogeneous matrices
  as well as a set of operations on these matrices.

  The vpHomogeneousMatrix class is derived from vpArray2D<double>.

  An homogeneous matrix is 4x4 matrix defines as
  \f[
  ^a{\bf M}_b = \left(\begin{array}{cc}
  ^a{\bf R}_b & ^a{\bf t}_b \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right)
  \f]
  that defines the position of frame <em>b</em> in frame <em>a</em>

  \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.

  There are different ways to initialize an homogeneous matrix. You can set each element of the matrix like:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpHomogeneousMatrix M;
    M[0][0] =  0; M[0][1] =  0; M[0][2] = -1; M[0][3] = 0.1;
    M[1][0] =  0; M[1][1] = -1; M[1][2] =  0; M[1][3] = 0.2;
    M[2][0] = -1; M[2][1] =  0; M[2][2] =  0; M[2][3] = 0.3;

    std::cout << "M:" << std::endl;
    for (unsigned int i = 0; i < M.getRows(); ++i) {
      for (unsigned int j = 0; j < M.getCols(); ++j) {
        std::cout << M[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
  \endcode
  It produces the following printings:
  \code{.unparsed}
  M:
  0 0 -1 0.1
  0 -1 0 0.2
  -1 0 0 0.3
  0 0 0 1
  \endcode
  You can also use vpRotationMatrix::operator<< and vpTranslationVector::operator<< like:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector t;
    vpRotationMatrix R;
    R << 0, 0, -1,
        0, -1, 0,
        -1, 0, 0;
    t << 0.1, 0.2, 0.3;
    vpHomogeneousMatrix M(t, R);
    std::cout << "M:\n" << M << std::endl;
  }
  \endcode
  If ViSP is build with c++11 enabled, you can do the same using:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    {
      vpHomogeneousMatrix M( vpTranslationVector(0.1, 0.2, 0.3), vpRotationMatrix( {0, 0, -1, 0, -1, 0, -1, 0, 0} ) );
      std::cout << "M:\n" << M << std::endl;
    }
    {
      vpHomogeneousMatrix M { 0,  0, -1, 0.1,
                              0, -1,  0, 0.2,
                            -1,  0,  0, 0.3 };
      std::cout << "M:\n" << M << std::endl;
    }
  }
  \endcode

  <b>JSON serialization</b>

  Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpHomogeneousMatrix.
  The following sample code shows how to save a homogeneous matrix in a file named `homo-mat.json`
  and reload the values from this JSON file.
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  #if defined(VISP_HAVE_NLOHMANN_JSON)
    std::string filename = "homo-mat.json";
    {
      vpHomogeneousMatrix M(vpTranslationVector(0.1, 0.2, 0.3), vpRotationMatrix({ 0, 0, -1, 0, -1, 0, -1, 0, 0 }));
      std::ofstream file(filename);
      const nlohmann::json j = M;
      file << j;
      file.close();
    }
    {
      std::ifstream file(filename);
      const nlohmann::json j = nlohmann::json::parse(file);
      vpHomogeneousMatrix M;
      M = j;
      file.close();
      std::cout << "Read homogeneous matrix from " << filename << ":\n" << M << std::endl;
    }
  #endif
  }
  \endcode
  If you build and execute the sample code, it will produce the following output:
  \code{.unparsed}
  Read homogeneous matrix from homo-mat.json:
  0  0  -1  0.1
  0  -1  0  0.2
  -1  0  0  0.3
  0  0  0  1
  \endcode

  The content of the `homo-mat.json` file is the following:
  \code{.unparsed}
  $ cat homo-mat.json
  {"cols":4,"data":[0.0,0.0,-1.0,0.1,0.0,-1.0,0.0,0.2,-1.0,0.0,0.0,0.3,0.0,0.0,0.0,1.0],"rows":4,"type":"vpHomogeneousMatrix"}
  \endcode
*/
class VISP_EXPORT vpHomogeneousMatrix : public vpArray2D<double>
{
public:
  vpHomogeneousMatrix();
  vpHomogeneousMatrix(const vpHomogeneousMatrix &M);
  vpHomogeneousMatrix(const vpTranslationVector &t, const vpRotationMatrix &R);
  vpHomogeneousMatrix(const vpTranslationVector &t, const vpThetaUVector &tu);
  vpHomogeneousMatrix(const vpTranslationVector &t, const vpQuaternionVector &q);
  VP_EXPLICIT vpHomogeneousMatrix(const vpPoseVector &p);
  VP_EXPLICIT vpHomogeneousMatrix(const std::vector<float> &v);
  VP_EXPLICIT vpHomogeneousMatrix(const std::vector<double> &v);
  vpHomogeneousMatrix(double tx, double ty, double tz, double tux, double tuy, double tuz);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  VP_EXPLICIT vpHomogeneousMatrix(const std::initializer_list<double> &list);
#endif

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  VP_DEPRECATED void buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R);
  VP_DEPRECATED void buildFrom(const vpTranslationVector &t, const vpThetaUVector &tu);
  VP_DEPRECATED void buildFrom(const vpTranslationVector &t, const vpQuaternionVector &q);
  VP_DEPRECATED void buildFrom(const vpPoseVector &p);
  VP_DEPRECATED void buildFrom(const std::vector<float> &v);
  VP_DEPRECATED void buildFrom(const std::vector<double> &v);
  VP_DEPRECATED void buildFrom(double tx, double ty, double tz, double tux, double tuy, double tuz);
#endif
  vpHomogeneousMatrix &build(const vpTranslationVector &t, const vpRotationMatrix &R);
  vpHomogeneousMatrix &build(const vpTranslationVector &t, const vpThetaUVector &tu);
  vpHomogeneousMatrix &build(const vpTranslationVector &t, const vpQuaternionVector &q);
  vpHomogeneousMatrix &build(const vpPoseVector &p);
  vpHomogeneousMatrix &build(const std::vector<float> &v);
  vpHomogeneousMatrix &build(const std::vector<double> &v);
  vpHomogeneousMatrix &build(const double &tx, const double &ty, const double &tz, const double &tux, const double &tuy, const double &tuz);

  void convert(std::vector<float> &M);
  void convert(std::vector<double> &M);

  // Set to identity
  void eye();

  vpColVector getCol(unsigned int j) const;
  vpRotationMatrix getRotationMatrix() const;
  vpThetaUVector getThetaUVector() const;
  vpTranslationVector getTranslationVector() const;

  // Invert the homogeneous matrix.
  vpHomogeneousMatrix inverse() const;
  // Invert the homogeneous matrix.
  void inverse(vpHomogeneousMatrix &Mi) const;

  // Test if the rotational part of the matrix is a rotation matrix.
  bool isAnHomogeneousMatrix(double threshold = 1e-6) const;
  bool isValid() const;

  void insert(const vpRotationMatrix &R);
  void insert(const vpThetaUVector &tu);
  void insert(const vpTranslationVector &t);
  void insert(const vpQuaternionVector &t);

  void extract(vpRotationMatrix &R) const;
  void extract(vpThetaUVector &tu) const;
  void extract(vpTranslationVector &t) const;
  void extract(vpQuaternionVector &q) const;

  /*!
   * Read an homogeneous matrix from an input file stream. The
   * homogeneous matrix is considered as a 4 by 4 matrix.
   *
   * \param f : Input file stream.
   *
   * The code below shows how to get an homogeneous matrix from a file.
   *
   * \code
   * vpHomogeneousMatrix M;
   *
   * std::ifstream f("homogeneous.dat");
   * M.load(f);
   * \endcode
   *
   * \sa load(const std::string &), save(std::ifstream &)
   */
  void load(std::ifstream &f);

  /*!
   * Read an homogeneous matrix from an input file. The
   * homogeneous matrix is considered as a 4 by 4 matrix.
   *
   * \param filename : Input file name.
   *
   * The code below shows how to get an homogeneous matrix from a file.
   *
   * \code
   * vpHomogeneousMatrix M;
   *
   * M.load("homogeneous.dat");
   * \endcode
   *
   * \sa load(std::ifstream &), save(const std::string &)
   */
  void load(const std::string &filename);

  /*!
   * Save an homogeneous matrix in an output file stream.
   *
   * \param f : Output file stream. The homogeneous matrix is saved as a 4 by 4 matrix.
   *
   * The code below shows how to save an homogeneous matrix in a file.
   *
   * \code
   * // Construct an homogeneous matrix
   * vpTranslationVector t(1,2,3);
   * vpRxyzVector r(M_PI, 0, -M_PI/4.);
   * vpRotationMatrix R(r);
   * vpHomogeneousMatrix M(t, R);
   *
   * // Save the content of the matrix in "homogeneous.dat"
   * std::ofstream f("homogeneous.dat");
   * M.save(f);
   * \endcode
   *
   * \sa save(const std::string &), load(std::ifstream &)
   */
  void save(std::ofstream &f) const;

  /*!
   * Save an homogeneous matrix in a file.
   *
   * \param filename : Output file name. The homogeneous matrix is saved as a 4 by 4 matrix.
   *
   * The code below shows how to save an homogeneous matrix in a file.
   *
   * \code
   * // Construct an homogeneous matrix
   * vpTranslationVector t(1,2,3);
   * vpRxyzVector r(M_PI, 0, -M_PI/4.);
   * vpRotationMatrix R(r);
   * vpHomogeneousMatrix M(t, R);
   *
   * // Save the content of the matrix in "homogeneous.dat"
   * M.save("homogeneous.dat");
   * \endcode
   *
   * \sa save(std::ofstream &), load(const std::string &)
   */
  void save(const std::string &filename) const;

  vpHomogeneousMatrix &operator=(const vpHomogeneousMatrix &M);
  vpHomogeneousMatrix operator*(const vpHomogeneousMatrix &M) const;
  vpHomogeneousMatrix &operator*=(const vpHomogeneousMatrix &M);

  vpColVector operator*(const vpColVector &v) const;
  vpTranslationVector operator*(const vpTranslationVector &t) const;
  vpHomogeneousMatrix operator*(const vpRotationMatrix &R) const;

  // Multiply by a point
  vpPoint operator*(const vpPoint &bP) const;

  vpHomogeneousMatrix &operator<<(double val);
  vpHomogeneousMatrix &operator,(double val);

  void orthogonalizeRotation();

  void print() const;

  /*!
   * This function is not applicable to an homogeneous matrix that is always a
   * 4-by-4 matrix.
   * \exception vpException::fatalError When this function is called.
   */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize an homogeneous matrix"));
  }

  static vpHomogeneousMatrix compute3d3dTransformation(const std::vector<vpPoint> &p, const std::vector<vpPoint> &q);

  static vpHomogeneousMatrix mean(const std::vector<vpHomogeneousMatrix> &vec_M);

#ifdef VISP_HAVE_NLOHMANN_JSON
public:
  static const std::string jsonTypeName;
private:
  friend void to_json(nlohmann::json &j, const vpHomogeneousMatrix &T);
  friend void from_json(const nlohmann::json &j, vpHomogeneousMatrix &T);
  // Conversion helper function to avoid circular dependencies and MSVC errors that are not exported in the DLL
  void parse_json(const nlohmann::json &j);
  void convert_to_json(nlohmann::json &j) const;
public:

#endif

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
   * @name Deprecated functions
   */
   //@{
   /*!
    * \deprecated Provided only for compat with previous releases.
    *  This function does nothing.
    */
  VP_DEPRECATED void init() { }
  /*!
   *  \deprecated You should rather use eye().
   */
  VP_DEPRECATED void setIdentity();
  //@}
#endif

protected:
  unsigned int m_index;
};

#ifdef VISP_HAVE_NLOHMANN_JSON
inline void to_json(nlohmann::json &j, const vpHomogeneousMatrix &T)
{
  T.convert_to_json(j);
}

inline void from_json(const nlohmann::json &j, vpHomogeneousMatrix &T)
{
  T.parse_json(j);
}
#endif
END_VISP_NAMESPACE
#endif
