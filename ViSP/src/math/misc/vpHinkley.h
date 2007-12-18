/****************************************************************************
 *
 * $Id: vpHinkley.h,v 1.7 2007-12-18 14:29:03 fspindle Exp $
 *
 * Copyright (C) 1998-2007 Inria. All rights reserved.
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
