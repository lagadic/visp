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
 */

#ifndef VP_PARABOLA_MODEL_H
#define VP_PARABOLA_MODEL_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#ifndef DOXYGEN_SHOULD_SKIP_THIS
namespace tutorial
{
/*!
 * \brief Model of a parabola \f[v = \sum_{i = 0}^N a_i u^i \f] where \f[N\f] is the
 * degree of the polynomial.
 */
class vpTutoParabolaModel
{
public:
  inline vpTutoParabolaModel(const unsigned int &degree, const unsigned int &height, const unsigned int &width)
    : m_degree(degree)
    , m_height(static_cast<unsigned int>(height))
    , m_width(static_cast<unsigned int>(width))
    , m_coeffs(degree + 1, 0.)
  { }

  /**
   * \brief Construct a new vpTutoParabolaModel object
   *
   * \param[in] coeffs The coefficients of the polynomial, where coeffs[0] = offset
   * and coeffs[m_degree] is the coefficient applied to the highest degree input.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   */
  inline vpTutoParabolaModel(const VISP_NAMESPACE_ADDRESSING vpColVector &coeffs, const unsigned int &height, const unsigned int &width)
    : m_degree(coeffs.size() - 1)
    , m_height(static_cast<unsigned int>(height))
    , m_width(static_cast<unsigned int>(width))
    , m_coeffs(coeffs)
  { }

  /**
   * \brief Construct a new vpTutoParabolaModel object
   *
   * \param[in] coeffs The coefficients of the polynomial, where coeffs[0][0] = offset
   * and coeffs[m_degree][0] is the coefficient applied to the highest degree input.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   */
  inline vpTutoParabolaModel(const VISP_NAMESPACE_ADDRESSING vpMatrix &coeffs, const unsigned int &height, const unsigned int &width)
    : m_degree(coeffs.getRows() - 1)
    , m_height(static_cast<unsigned int>(height))
    , m_width(static_cast<unsigned int>(width))
    , m_coeffs(coeffs.getCol(0))
  { }

  /**
   * @brief Compute \f[v = \sum_{i = 0}^N a_i u^i \f]
   *
   * \param[in] u Input
   * \return float The corresponding v.
   */
  inline double eval(const double &u) const
  {
    double normalizedU = u / m_width;
    double v = 0.;
    for (unsigned int i = 0; i <= m_degree; ++i) {
      v += m_coeffs[i] * std::pow(normalizedU, i);
    }
    v *= m_height;
    return v;
  }

  /**
   * \brief Cast into a vpColVector
   *
   * \return coeffs a:=coeffs[0] b:=coeffs[1] c:=coeffs[2]
   */
  inline VISP_NAMESPACE_ADDRESSING vpColVector toVpColVector() const
  {
    return m_coeffs;
  }

  inline vpTutoParabolaModel &operator=(const vpTutoParabolaModel &other)
  {
    m_degree = other.m_degree;
    m_height = other.m_height;
    m_width = other.m_width;
    m_coeffs = other.m_coeffs;
    return *this;
  }

  /**
   * @brief Fill the matrices that form the linear system A X = b
   * where A contains the different powers of the u-coordinates,
   * X contains the model coefficients and b contains the
   * v-coordinates.
   *
   * \param[in] degree The highest degree of the polynomial.
   * \param[in] height The height of the input image.
   * \param[in] width The width of the input image.
   * \param[in] pts The points to use to interpolate the coefficients of the parabola.
   * \param[out] A The matrix that contains the different powers of the u-coordinates.
   * \param[out] b The matrix that contains the v-coordinates.
   * \return Fill
   */
  //! [Fill_LMS_system]
  static void fillSystem(const unsigned int &degree, const double &height, const double &width, const std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> &pts, VISP_NAMESPACE_ADDRESSING vpMatrix &A, VISP_NAMESPACE_ADDRESSING vpMatrix &b)
  {
    const unsigned int nbPts = static_cast<unsigned int>(pts.size());
    const unsigned int nbCoeffs = degree + 1;
    std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> normalizedPts;
    // Normalization to avoid numerical instability
    for (const auto &pt: pts) {
      normalizedPts.push_back(VISP_NAMESPACE_ADDRESSING vpImagePoint(pt.get_i() / height, pt.get_j() / width));
    }
    A.resize(nbPts, nbCoeffs, 1.); // Contains the u^i
    b.resize(nbPts, 1); // Contains the v coordinates
    for (unsigned int i = 0; i < nbPts; ++i) {
      double u = normalizedPts[i].get_u();
      double v = normalizedPts[i].get_v();
      for (unsigned int j = 0; j < nbCoeffs; ++j) {
        A[i][j] = std::pow(u, j);
      }
      b[i][0] = v;
    }
  }
  //! [Fill_LMS_system]

  friend std::ostream &operator<<(std::ostream &os, const vpTutoParabolaModel &model)
  {
    os << "Highest degree = " << model.m_degree << std::endl;
    os << "Coeffs = [ " << model.m_coeffs.transpose() << " ]" << std::endl;
    return os;
  }

private:
  unsigned int m_degree; /*!< The highest degree of the polynomial.*/
  unsigned int m_height; /*!< The height of the input image*/
  unsigned int m_width; /*!< The width of the input image*/
  VISP_NAMESPACE_ADDRESSING vpColVector m_coeffs; /*!< The coefficient of the polynomial, where m_coeffs[0] is the offset and m_coeffs[m_degree] is the coefficient applied to the highest degree.*/
};
}
#endif
#endif
#endif
