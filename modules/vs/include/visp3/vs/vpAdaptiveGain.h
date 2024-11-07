/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Adaptive gain.
 */

/*!
 * \file vpAdaptiveGain.h
 * \brief Adaptive gain
 */

#ifndef _vpAdaptiveGain_h_
#define _vpAdaptiveGain_h_

#include <iostream>
#include <visp3/core/vpConfig.h>

BEGIN_VISP_NAMESPACE
class vpColVector;

/*!
 * \class vpAdaptiveGain
 *
 * \ingroup group_task
 *
 * \brief Adaptive gain computation.
 *
 * As described in \cite Kermorgant14a, a varying gain \f$ \lambda \f$ could be
 * used in the visual servoing control law \f[{\bf v}_c = -\lambda {\bf
 * L}^{+}_{e} {\bf e}\f] with
 *
 * \f[ \lambda (|| {\bf e}||) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
 * \lambda'_0}{\lambda_0 - \lambda_\infty}||{\bf e}||} + \lambda_\infty \f]
 *
 * where:
 *
 * - \f$\lambda_0 = \lambda(0)\f$ is the gain in 0, that is for very small
 *   values of \f$||{\bf e}||\f$
 * - \f$\lambda_\infty = \lambda_{||{\bf e}|| \rightarrow \infty}\lambda(||{\bf
 *   e}||)\f$ is the gain to infinity, that is for very high values of \f$||{\bf
 *   e}||\f$
 * - \f$\lambda'_0\f$ is the slope of \f$\lambda\f$ at \f$||{\bf e}|| = 0\f$
 *
 * As described in \ref tutorial-boost-vs, the interest of \ref adaptive_gain
 * is to reduce the time to convergence in order to speed up the servo.
 *
 * The following example shows how to use this class in order to use an
 * adaptive gain with the following parameters \f$\lambda_0 = 4\f$,
 * \f$\lambda_\infty = 0.4 \f$ and \f$\lambda'_0 = 30\f$.
 *
 * \code
 * #include <visp3/vs/vpAdaptiveGain.h>
 * #include <visp3/vs/vpServo.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpAdaptiveGain lambda(4, 0.4, 30);   // lambda(0)=4, lambda(oo)=0.4 and lambda'(0)=30
 *
 *   vpServo servo;
 *   servo.setLambda(lambda);
 *
 *   while(1) {
 *     vpColVector v = servo.computeControlLaw();
 *   }
 * }
 * \endcode
 *
 * This other example shows how to use this class in order to set a constant
 * gain \f$\lambda = 0.5\f$ that will ensure an exponential decrease of the task
 * error.
 *
 * \code
 * #include <visp3/vs/vpAdaptiveGain.h>
 * #include <visp3/vs/vpServo.h>
 *
 * int main()
 * {
 *   vpAdaptiveGain lambda(0.5);
 *
 *   vpServo servo;
 *   servo.setLambda(lambda);
 *
 *   while(1) {
 *     vpColVector v = servo.computeControlLaw();
 *   }
 * }
 * \endcode
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
  /*!
   * Basic constructor which initializes all the parameters with their default
   * value:
   * - \f$ \lambda_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_ZERO
   * - \f$ \lambda_\infty = 0.1666 \f$ using
   * vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY
   * - \f$ \lambda'_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE
  */
  vpAdaptiveGain();

  /*!
   * Constructor that initializes the gain as constant. In that case
   * \f$\lambda(||{\bf e}||) = c\f$.
   *
   * \param c : Value of the constant gain. A typical value is 0.5.
   */
  VP_EXPLICIT vpAdaptiveGain(double c);

  /*!
   * Constructor that initializes the gain as adaptive.
   *
   * \param gain_at_zero : the expected gain when \f$||{\bf e}||=0\f$:
   * \f$\lambda_0\f$.
   * \param gain_at_infinity : the expected gain when \f$||{\bf
   * e}||\rightarrow\infty\f$: \f$\lambda_\infty\f$.
   * \param slope_at_zero : the
   * expected slope of \f$\lambda(||{\bf e}||)\f$ when \f$||{\bf e}||=0\f$:
   * \f$\lambda'_0\f$.
   */
  vpAdaptiveGain(double gain_at_zero, double gain_at_infinity, double slope_at_zero);

  /*!
   * Initializes the parameters to have a constant gain. In that case
   * \f$\lambda(||{\bf e}||) = c\f$.
   *
   * \param c : Value of the constant gain. A typical value is 0.5.
   */
  void initFromConstant(double c);

  /*!
   * Initializes the parameters with the default value :
   * - \f$ \lambda_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_ZERO
   * - \f$ \lambda_\infty = 0.1666 \f$ using
   * vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY
   * - \f$ \lambda'_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE
   */
  void initFromVoid(void);

  /*!
   * Set the parameters \f$\lambda_0, \lambda_\infty, \lambda'_0\f$ used to
   * compute \f$\lambda(||{\bf e}||)\f$.
   *
   * \param gain_at_zero : the expected gain when \f$||{\bf e}||=0\f$:
   * \f$\lambda_0\f$.
   * \param gain_at_infinity : the expected gain when \f$||{\bf
   * e}||\rightarrow\infty\f$: \f$\lambda_\infty\f$.
   * \param slope_at_zero : the
   * expected slope of \f$\lambda(||{\bf e}||)\f$ when \f$||{\bf e}||=0\f$:
   * \f$\lambda'_0\f$.
   */
  void initStandard(double gain_at_zero, double gain_at_infinity, double slope_at_zero);

  /*!
   * Sets the internal parameters in order to obtain a constant gain equal to
   * the gain in 0 set through the parameter \f$\lambda_0\f$.
   *
   * \return It returns the value of the constant gain \f$\lambda_0\f$.
   */
  double setConstant(void);

  /*!
   * Computes the value of the adaptive gain \f$\lambda(x)\f$ using:
   *
   * \f[ \lambda (x) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
   * \lambda'_0}{\lambda_0 - \lambda_\infty}x} + \lambda_\infty \f]
   *
   * \param x : Input value to consider. During a visual servo this value can be
   * the Euclidean norm \f$||{\bf e}||\f$ or the infinity norm \f$||{\bf
   * e}||_{\infty}\f$ of the task function.
   *
   * \return It returns the value of the computed gain.
   */
  double value_const(double x) const;

  /*!
   * Computes the value of the adaptive gain \f$\lambda(x)\f$ using:
   *
   * \f[ \lambda (x) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
   * \lambda'_0}{\lambda_0 - \lambda_\infty}x} + \lambda_\infty \f]
   *
   * This value is stored as a parameter of the class.
   *
   * \param x : Input value to consider. During a visual servo this value can be
   * the Euclidean norm \f$||{\bf e}||\f$ or the infinity norm \f$||{\bf
   * e}||_{\infty}\f$ of the task function.
   *
   * \return It returns the value of the computed gain.
   */
  double value(double x) const;

  /*!
   * Gets the value of the gain at infinity (ie the value of \f$ \lambda_\infty =
   * c \f$). This function is similar to limitValue() except that here the value
   * is not stored as a parameter of the class.
   *
   * \return It returns the value of the gain at infinity.
   */
  double limitValue_const(void) const;

  /*!
   * Gets the value of the gain at infinity (ie the value of \f$\lambda_\infty =
   * c \f$) and stores it as a parameter of the class.
   *
   * \return It returns the value of the gain at infinity.
   */
  double limitValue(void) const;

  /*!
   * Gets the last adaptive gain value which was stored in the class.
   *
   * \return It returns the last adaptive gain value which was stored in the
   *  class.
   */
  inline double getLastValue(void) const { return this->lambda; }

  /*!
   * Operator that computes \f$\lambda(x)\f$ where
   *
   * \f[ \lambda (x) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
   * \lambda'_0}{\lambda_0 - \lambda_\infty}x} + \lambda_\infty \f]
   *
   * \param x : Input value to consider. During a visual servo this value can be
   * the Euclidean norm \f$||{\bf e}||\f$ or the infinity norm \f$||{\bf
   * e}||_{\infty}\f$ of the task function.
   *
   * \return It returns the value of the computed gain.
   *
   * \sa value()
   */
  double operator()(double x) const;

  /*!
   * Operator which computes \f$\lambda({||x||}_{\infty})\f$ where
   *
   * \f[ \lambda ({||x||}_{\infty}) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
   * \lambda'_0}{\lambda_0 - \lambda_\infty}{||x||}_{\infty}} + \lambda_\infty
   * \f]
   *
   * \param x : Input vector \f$ \bf x\f$ to consider.
   *
   * \return It returns the value of the computed gain.
   */
  double operator()(const vpColVector &x) const;

  /*!
   * Gets the value of the gain at infinity (ie the value of \f$\lambda_\infty =
   * c \f$).
   *
   * \return It returns the value of the gain at infinity.
   *
   * \sa limitValue()
   */
  double operator()(void) const;

  /*!
   * Prints the adaptive gain parameters \f$\lambda_0, \lambda_\infty,
   * \lambda'_0\f$.
   *
   * \param os : The stream where to print the adaptive gain parameters.
   * \param lambda : The adaptive gain containing the parameters to print.
   */
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpAdaptiveGain &lambda);
};
END_VISP_NAMESPACE
#endif
