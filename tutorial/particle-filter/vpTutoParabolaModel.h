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
 * \brief Model of a parabola v = a u^2 + b u + c
 */
class vpTutoParabolaModel
{
public:
  inline vpTutoParabolaModel()
    : m_a(0.f)
    , m_b(0.f)
    , m_c(0.f)
  { }

  inline vpTutoParabolaModel(const float &a, const float &b, const float &c)
    : m_a(a)
    , m_b(b)
    , m_c(c)
  { }

  /**
   * \brief Construct a new vpTutoParabolaModel object
   *
   * \param coeffs a:=coeffs[0] b:=coeffs[1] c:=coeffs[2]
   */
  inline vpTutoParabolaModel(const VISP_NAMESPACE_ADDRESSING vpColVector &coeffs)
    : m_a(coeffs[0])
    , m_b(coeffs[1])
    , m_c(coeffs[2])
  { }

  /**
   * \brief Construct a new vpTutoParabolaModel object
   *
   * \param coeffs a:=coeffs[0][0] b:=coeffs[1][0] c:=coeffs[2][0]
   */
  inline vpTutoParabolaModel(const VISP_NAMESPACE_ADDRESSING vpMatrix &coeffs)
    : m_a(coeffs[0][0])
    , m_b(coeffs[1][0])
    , m_c(coeffs[2][0])
  { }

  /**
   * @brief Compute v = a u^2 + b u + c
   *
   * \param[in] u Input
   * \return float The corresponding v.
   */
  inline float eval(const float &u) const
  {
    float v = (m_a * u * u) + (m_b * u) + m_c;
    return v;
  }

  /**
   * \brief Cast into a vpColVector
   *
   * \return coeffs a:=coeffs[0] b:=coeffs[1] c:=coeffs[2]
   */
  inline vpColVector toVpColVector() const
  {
    vpColVector coeffs(3);
    coeffs[0] = m_a;
    coeffs[1] = m_b;
    coeffs[2] = m_c;
    return coeffs;
  }

  inline vpTutoParabolaModel &operator=(const vpTutoParabolaModel &other)
  {
    m_a = other.m_a;
    m_b = other.m_b;
    m_c = other.m_c;
    return *this;
  }

  /**
   * @brief Fill the matrices that form the linear system A X = b
   * where A contains the different powers of the u-coordinates,
   * X contains the model coefficients and b contains the
   * v-coordinates.
   *
   * \param[in] pts The points to use to interpolate the coefficients of the parabola.
   * \param[out] A The matrix that contains the different powers of the u-coordinates.
   * \param[out] b The matrix that contains the v-coordinates.
   * \return Fill
   */
  static void fillSystem(const std::vector<VISP_NAMESPACE_ADDRESSING vpImagePoint> &pts, VISP_NAMESPACE_ADDRESSING vpMatrix &A, VISP_NAMESPACE_ADDRESSING vpMatrix &b)
  {
    unsigned int nbPts = pts.size();
    A.resize(nbPts, 3, 1.);
    b.resize(nbPts, 1);
    for (unsigned int i = 0; i < nbPts; ++i) {
      double u = pts[i].get_u();
      double v = pts[i].get_v();
      A[i][0] = u *u;
      A[i][1] = u;
      A[i][2] = 1.;
      b[i][0] = v;
    }
  }

private:
  double m_a; /*!< Coefficient applied to u^2*/
  double m_b; /*!< Coefficient applied to u*/
  double m_c; /*!< Offset*/
};

#endif
