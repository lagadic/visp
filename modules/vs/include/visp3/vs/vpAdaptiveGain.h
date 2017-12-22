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
 * Adaptive gain.
 *
 * Authors:
 * Nicolas Mansard
 *
 *****************************************************************************/
/*!
\file vpAdaptiveGain.h
\brief Adaptive gain
*/

#ifndef __VP_ADAPTIVE_GAIN_H
#define __VP_ADAPTIVE_GAIN_H

#include <iostream>
#include <visp3/core/vpConfig.h>

class vpColVector;
/*!
  \class vpAdaptiveGain

  \ingroup group_task

  \brief Adaptive gain computation.

  As described in \cite Kermorgant14a, a varying gain \f$ \lambda \f$ could be
used in the visual servoing control law \f[{\bf v}_c = -\lambda {\bf
L}^{+}_{e} {\bf e}\f] with

  \f[ \lambda (|| {\bf e}||) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
\lambda'_0}{\lambda_0 - \lambda_\infty}||{\bf e}||} + \lambda_\infty \f]

  where:

  - \f$\lambda_0 = \lambda(0)\f$ is the gain in 0, that is for very small
values of \f$||{\bf e}||\f$
  - \f$\lambda_\infty = \lambda_{||{\bf e}|| \rightarrow \infty}\lambda(||{\bf
e}||)\f$ is the gain to infinity, that is for very high values of \f$||{\bf
e}||\f$
  - \f$\lambda'_0\f$ is the slope of \f$\lambda\f$ at \f$||{\bf e}|| = 0\f$

  As described in \ref tutorial-boost-vs, the interest of \ref adaptive_gain
is to reduce the time to convergence in order to speed up the servo.

  The following example shows how to use this class in order to use an
adaptive gain with the following parameters \f$\lambda_0 = 4\f$,
\f$\lambda_\infty = 0.4 \f$ and \f$\lambda'_0 = 30\f$.

\code
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServo.h>

int main()
{
  vpAdaptiveGain lambda(4, 0.4, 30);   // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30

  vpServo servo;
  servo.setLambda(lambda);

  while(1) {

    vpColVector v = servo.computeControlLaw();
  }
}
  \endcode

  This other example shows how to use this class in order to set a constant
gain \f$\lambda = 0.5\f$ that will ensure an exponential decrease of the task
error.

\code
#include <visp3/vs/vpAdaptiveGain.h>
#include <visp3/vs/vpServo.h>

int main()
{
  vpAdaptiveGain lambda(0.5);

  vpServo servo;
  servo.setLambda(lambda);

  while(1) {

    vpColVector v = servo.computeControlLaw();
  }
}
  \endcode
*/

class VISP_EXPORT vpAdaptiveGain
{
public:
  static const double DEFAULT_LAMBDA_ZERO;
  static const double DEFAULT_LAMBDA_INFINITY;
  static const double DEFAULT_LAMBDA_SLOPE;

private:
  // Coefficient such as lambda (x) = a * exp (-b*x) + c
  double coeff_a; // \f$ a = \lambda(0) - \lambda(\infty) \f$
  double coeff_b; // \f$ b = {\dot \lambda}(0) / a \f$
  double coeff_c; // \f$ c = \lambda(\infty) \f$

  // Last computed value
  mutable double lambda;

public:
  /* --- CONSTRUCTOR --------------------------------------------------------
   */

  vpAdaptiveGain();
  explicit vpAdaptiveGain(double c);
  vpAdaptiveGain(double gain_at_zero, double gain_at_infinity, double slope_at_zero);

  /* --- INIT ---------------------------------------------------------------
   */
  void initFromConstant(double c);
  void initFromVoid(void);
  void initStandard(double gain_at_zero, double gain_at_infinity, double slope_at_zero);

  /* --- MODIFIORS ----------------------------------------------------------
   */
  double setConstant(void);

  /* --- COMPUTE ------------------------------------------------------------
   */
  /* \brief Calcule la valeur de lambda au point courrant.
   *
   * Determine la valeur du lambda adaptatif en fonction de la valeur
   * de la norme de la fonction de tache e par extrapolation exponentielle.
   * La fonction est : (en_infini - en_zero) * exp (-pente * ||e|| ) +
   * en_infini. On a bien :
   *    - lambda(10^5) = en_infini ;
   *    - lambda(0) = en_zero ;
   *    - lambda(x ~ 0) ~ - pente * x + en_zero.
   * \param val_e: valeur de la norme de l'erreur.
   * \return: valeur de gain au point courrant.
   */
  double value_const(double x) const;

  /* \brief Calcule la valeur de lambda au point courrant et stockage du
   * resultat.
   *
   * La fonction calcule la valeur de lambda d'apres la valeur de la norme
   * de l'erreur, comme le fait la fonction valeur_const.
   * La fonction non constante stocke de plus le resultat dans this ->lambda.
   * \param val_e: valeur de la norme de l'erreur.
   * \return: valeur de gain au point courrant.
   */
  double value(double x) const;

  double limitValue_const(void) const;

  double limitValue(void) const;

  /* --- ACCESSORS ----------------------------------------------------------
   */

  /*!
      Gets the last adaptive gain value which was stored in the class.

      \return It returns the last adaptive gain value which was stored in the
     class.
    */
  inline double getLastValue(void) const { return this->lambda; }

  double operator()(double x) const;

  /* \brief Lance la fonction valeur avec la norme INFINIE du vecteur. */
  double operator()(const vpColVector &x) const;

  /* \brief Idem function limitValue. */
  double operator()(void) const;

  /* --- IOSTREAM -----------------------------------------------------------
   */
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpAdaptiveGain &lambda);
};

#endif
