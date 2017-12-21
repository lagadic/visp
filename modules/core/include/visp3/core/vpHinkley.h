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
 * Hinkley's cumulative sum test implementation.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpHinkley_H
#define vpHinkley_H

/*!
  \file vpHinkley.h
  \brief class for Hinkley's cumulative test computation.
*/
#include <visp3/core/vpConfig.h>

/*!
  \class vpHinkley

  \ingroup group_core_math_tools
  \brief This class implements the Hinkley's cumulative sum test.

  \author Fabien Spindler (Fabien.Spindler@irisa.fr), Irisa / Inria Rennes

  The Hinkley's cumulative sum test is designed to detect jump in mean
  of an observed signal \f$ s(t) \f$. It is known to be robust (by
  taking into account all the past of the observed quantity),
  efficient, and inducing a very low computational load. The other
  attractive features of this test are two-fold. First, it can
  straightforwardly and accurately provide the jump instant. Secondly,
  due to its formulation (cumulative sum test), it can simultaneously
  handle both very abrupt and important changes, and gradual smaller
  ones without adapting the involved thresholds.

  Two tests are performed in parallel to look for downwards or upwards
  jumps in \f$ s(t) \f$, respectively defined by:

  \f[ S_k = \sum_{t=0}^{k} (s(t) - m_0 + \frac{\delta}{2}) \f]
  \f[ M_k = \max_{0 \leq i \leq k} S_i\f]
  \f[ T_k = \sum_{t=0}^{k} (s(t) - m_0 - \frac{\delta}{2}) \f]
  \f[ N_k = \max_{0 \leq i \leq k} T_i\f]

  In which \f$m_o\f$ is computed on-line and corresponds to the mean
  of the signal \f$ s(t) \f$ we want to detect a jump. \f$m_o\f$ is
  re-initialized at zero after each jump detection. \f$\delta\f$
  denotes the jump minimal magnitude that we want to detect and
  \f$\alpha\f$ is a predefined threshold. These values are set by
  default to 0.2 in the default constructor vpHinkley(). To modify the
  default values use setAlpha() and setDelta() or the
  vpHinkley(double alpha, double delta) constructor.

  A downward jump is detected if \f$ M_k - S_k > \alpha \f$.
  A upward jump is detected if \f$ T_k - N_k > \alpha \f$.

  To detect only downward jumps in \f$ s(t) \f$ use
  testDownwardJump().To detect only upward jumps in \f$ s(t) \f$ use
  testUpwardJump(). To detect both, downard and upward jumps use
  testDownUpwardJump().

  If a jump is detected, the jump location is given by the last instant
  \f$k^{'}\f$ when \f$ M_{k^{'}} - S_{k^{'}} = 0 \f$, or \f$ T_{k^{'}} -
  N_{k^{'}} = 0 \f$.

*/
class VISP_EXPORT vpHinkley
{
public:
  /*! \enum vpHinkleyJumpType
    Indicates if a jump is detected by the Hinkley test.
  */
  typedef enum {
    noJump,       /*!< No jump is detected by the Hinkley test. */
    downwardJump, /*!< A downward jump is detected by the Hinkley test. */
    upwardJump    /*!< An upward jump is detected by the Hinkley test. */
  } vpHinkleyJumpType;

public:
  vpHinkley();
  ~vpHinkley();
  vpHinkley(double alpha, double delta);

  void init();
  void init(double alpha, double delta);

  void setDelta(double delta);
  void setAlpha(double alpha);
  vpHinkleyJumpType testDownwardJump(double signal);
  vpHinkleyJumpType testUpwardJump(double signal);
  vpHinkleyJumpType testDownUpwardJump(double signal);

  static void print(vpHinkleyJumpType jump);

  /*!
    \return The mean value \f$m_0\f$ of the signal \f$ s(t) \f$.

  */
  inline double getMean() { return mean; }
  /*!
    \return The value of \f$S_k = \sum_{t=0}^{k} (s(t) - m_0 +
    \frac{\delta}{2})\f$

  */
  inline double getSk() { return Sk; }
  /*!
    \return The value of \f$M_k\f$, the maximum value of \f$S_k\f$.

  */
  inline double getMk() { return Mk; }
  /*!

    \return The value of \f$T_k = \sum_{t=0}^{k} (s(t) - m_0 -
    \frac{\delta}{2})\f$

  */
  inline double getTk() { return Tk; }
  /*!
    \return The value of \f$N_k\f$, the maximum value of \f$T_k\f$.

  */
  inline double getNk() { return Nk; }

private:
  void computeMean(double signal);
  void computeSk(double signal);
  void computeMk();
  void computeTk(double signal);
  void computeNk();

private:
  double dmin2;
  double alpha;
  int nsignal; // Signal length
  double mean; // Signal mean value
  double Sk;
  double Mk;
  double Tk;
  double Nk;
};

#endif
