/****************************************************************************
 *
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
*****************************************************************************/

#ifndef VP_PARABOLA_MODEL_H
#define VP_PARABOLA_MODEL_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

/*!
 * \brief Model of a parabola \f[v = \sum_{i = 0}^N a_i u^i \f] where \f[N\f] is the
 * degree of the polynomial.
 */
class vpTutoParabolaModel
{
public:
  inline vpTutoParabolaModel(const unsigned int &degree)
    : m_degree(degree)
    , m_coeffs(degree + 1, 0.)
  { }

  /**
   * \brief Construct a new vpTutoParabolaModel object
   *
   * \param coeffs The coefficients of the polynomial, where coeffs[0] = offset
   * and coeffs[m_degree] is the coefficient applied to the highest degree input.
   */
  inline vpTutoParabolaModel(const VISP_NAMESPACE_ADDRESSING vpColVector &coeffs)
    : m_degree(coeffs.size())
    , m_coeffs(coeffs)
  { }

  /**
   * \brief Construct a new vpTutoParabolaModel object
   *
   * \param coeffs The coefficients of the polynomial, where coeffs[0][0] = offset
   * and coeffs[m_degree][0] is the coefficient applied to the highest degree input.
   */
  inline vpTutoParabolaModel(const VISP_NAMESPACE_ADDRESSING vpMatrix &coeffs)
    : m_degree(coeffs.getRows())
    , m_coeffs(coeffs.getCol(0))
  { }

  /**
   * @brief Compute v = a u^2 + b u + c
   *
   * \param[in] u Input
   * \return float The corresponding v.
   */
  inline float eval(const float &u) const
  {
    float v = 0.;
    for (unsigned int i = 0; i <= m_degree; ++i) {
      v += m_coeffs[i] * std::pow(u, i);
    }
    return v;
  }

  /**
   * \brief Cast into a vpColVector
   *
   * \return coeffs a:=coeffs[0] b:=coeffs[1] c:=coeffs[2]
   */
  inline vpColVector toVpColVector() const
  {
    return m_coeffs;
  }

  inline vpTutoParabolaModel &operator=(const vpTutoParabolaModel &other)
  {
    m_degree = other.m_degree;
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
   * \param[in] pts The points to use to interpolate the coefficients of the parabola.
   * \param[out] A The matrix that contains the different powers of the u-coordinates.
   * \param[out] b The matrix that contains the v-coordinates.
   * \return Fill
   */
  static void fillSystem(const unsigned int &degree, const std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> &pts, VISP_NAMESPACE_ADDRESSING vpMatrix &A, VISP_NAMESPACE_ADDRESSING vpMatrix &b)
  {
    unsigned int nbPts = pts.size();
    A.resize(nbPts, degree + 1, 1.);
    b.resize(nbPts, 1);
    for (unsigned int i = 0; i < nbPts; ++i) {
      double u = pts[i].get_u();
      double v = pts[i].get_v();
      for (unsigned int j = 0; j < degree + 1; ++j) {
        A[i][j] = std::pow(u, j);
      }
      b[i][0] = v;
    }
  }

private:
  unsigned int m_degree; /*!< The highest degree of the polynomial.*/
  vpColVector m_coeffs; /*!< The coefficient of the polynomial, where m_coeffs[0] is the offset and m_coeffs[m_degree] is the coefficient applied to the highest degree.*/
};

#endif
