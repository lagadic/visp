/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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
#include <visp/vpConfig.h>

/*!
  \class vpHinkley

  \ingroup Hinkley
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
    noJump, /*!< No jump is detected by the Hinkley test. */
    downwardJump, /*!< A downward jump is detected by the Hinkley test. */
    upwardJump /*!< An upward jump is detected by the Hinkley test. */
  } vpHinkleyJumpType;

 public:
  vpHinkley();
  ~vpHinkley();
  vpHinkley(double alpha, double delta);

  void init();
  void init(double alpha, double delta) ;

  void setDelta(double delta);
  void setAlpha(double alpha);
  vpHinkleyJumpType testDownwardJump(double signal);
  vpHinkleyJumpType testUpwardJump(double signal);
  vpHinkleyJumpType testDownUpwardJump(double signal);

  static void print(vpHinkleyJumpType jump) ;

  /*!
    \return The mean value \f$m_0\f$ of the signal \f$ s(t) \f$.

  */
  inline double getMean() {return mean;}
  /*!
    \return The value of \f$S_k = \sum_{t=0}^{k} (s(t) - m_0 + \frac{\delta}{2})\f$

  */
  inline double getSk() {return Sk;}
  /*!
    \return The value of \f$M_k\f$, the maximum value of \f$S_k\f$.

  */
  inline double getMk() {return Mk;}
  /*!

    \return The value of \f$T_k = \sum_{t=0}^{k} (s(t) - m_0 -
    \frac{\delta}{2})\f$

  */
  inline double getTk() {return Tk;}
  /*!
    \return The value of \f$N_k\f$, the maximum value of \f$T_k\f$.

  */
  inline double getNk() {return Nk;}
 private:
  void computeMean(double signal);
  void computeSk(double signal);
  void computeMk();
  void computeTk(double signal);
  void computeNk();

 private:
  double dmin2;
  double alpha;
  int    nsignal;	// Signal lenght
  double mean;	// Signal mean value
  double Sk;
  double Mk;
  double Tk;
  double Nk;
};

#endif
