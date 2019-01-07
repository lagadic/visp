/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
\file vpAdaptiveGain.cpp
*/

/* --- VISP --- */
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/vs/vpAdaptiveGain.h>

#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits

const double vpAdaptiveGain::DEFAULT_LAMBDA_ZERO = 1.666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY = 0.1666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE = 1.666;

/* --------------------------------------------------------------------------
 */
/* --- CONSTRUCTION ---------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!
  Basic constructor which initializes all the parameters with their default
  value:
  - \f$ \lambda_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_ZERO
  - \f$ \lambda_\infty = 0.1666 \f$ using
  vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY
  - \f$ \lambda'_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE

*/
vpAdaptiveGain::vpAdaptiveGain() : coeff_a(), coeff_b(), coeff_c(), lambda(1.)
{
  this->initFromVoid();

  return;
}

/*!
  Constructor that initializes the gain as constant. In that case
  \f$\lambda(||{\bf e}||) = c\f$.

  \param c : Value of the constant gain. A typical value is 0.5.
*/
vpAdaptiveGain::vpAdaptiveGain(double c) : coeff_a(), coeff_b(), coeff_c(), lambda(1.) { initFromConstant(c); }

/*!
  Constructor that initializes the gain as adaptive.

  \param gain_at_zero : the expected gain when \f$||{\bf e}||=0\f$:
  \f$\lambda_0\f$. \param gain_at_infinity : the expected gain when \f$||{\bf
  e}||\rightarrow\infty\f$: \f$\lambda_\infty\f$. \param slope_at_zero : the
  expected slope of \f$\lambda(||{\bf e}||)\f$ when \f$||{\bf e}||=0\f$:
  \f$\lambda'_0\f$.

*/
vpAdaptiveGain::vpAdaptiveGain(double gain_at_zero, double gain_at_infinity, double slope_at_zero)
  : coeff_a(), coeff_b(), coeff_c(), lambda(1.)
{
  initStandard(gain_at_zero, gain_at_infinity, slope_at_zero);
}

/* --------------------------------------------------------------------------
 */
/* --- INIT -----------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!
  Initializes the parameters to have a constant gain. In that case
  \f$\lambda(||{\bf e}||) = c\f$.

  \param c : Value of the constant gain. A typical value is 0.5.
*/
void vpAdaptiveGain::initFromConstant(const double c)
{
  this->coeff_a = 0;
  this->coeff_b = 1;
  this->coeff_c = c;
  return;
}

/*!
  Initializes the parameters with the default value :
  - \f$ \lambda_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_ZERO
  - \f$ \lambda_\infty = 0.1666 \f$ using
  vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY
  - \f$ \lambda'_0 = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE
*/
void vpAdaptiveGain::initFromVoid(void)
{
  this->initStandard(vpAdaptiveGain::DEFAULT_LAMBDA_ZERO, vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY,
                     vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE);
  return;
}

/*!
  Set the parameters \f$\lambda_0, \lambda_\infty, \lambda'_0\f$ used to
  compute \f$\lambda(||{\bf e}||)\f$.

  \param gain_at_zero : the expected gain when \f$||{\bf e}||=0\f$:
  \f$\lambda_0\f$. \param gain_at_infinity : the expected gain when \f$||{\bf
  e}||\rightarrow\infty\f$: \f$\lambda_\infty\f$. \param slope_at_zero : the
  expected slope of \f$\lambda(||{\bf e}||)\f$ when \f$||{\bf e}||=0\f$:
  \f$\lambda'_0\f$.
*/
void vpAdaptiveGain::initStandard(const double gain_at_zero, const double gain_at_infinity, const double slope_at_zero)
{
  this->coeff_a = gain_at_zero - gain_at_infinity;
  // if (0 == this ->coeff_a)
  if (std::fabs(this->coeff_a) <= std::numeric_limits<double>::epsilon()) {
    this->coeff_b = 0;
  } else {
    this->coeff_b = slope_at_zero / (this->coeff_a);
  }
  this->coeff_c = gain_at_infinity;

  return;
}

/* --------------------------------------------------------------------------
 */
/* --- MODIFICATOR ----------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!
  Sets the internal parameters in order to obtain a constant gain equal to
  the gain in 0 set through the parameter \f$\lambda_0\f$.

  \return It returns the value of the constant gain \f$\lambda_0\f$.
*/
double vpAdaptiveGain::setConstant(void)
{
  double res = this->coeff_a + this->coeff_c;

  this->coeff_a = 0;
  this->coeff_b = 1;
  this->coeff_c = res;

  return res;
}

/* --------------------------------------------------------------------------
 */
/* --- VALEUR ---------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!
  Computes the value of the adaptive gain \f$\lambda(x)\f$ using:

  \f[ \lambda (x) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
  \lambda'_0}{\lambda_0 - \lambda_\infty}x} + \lambda_\infty \f]

  \param x : Input value to consider. During a visual servo this value can be
  the euclidian norm \f$||{\bf e}||\f$ or the infinity norm \f$||{\bf
  e}||_{\infty}\f$ of the task function.

  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::value_const(const double x) const
{
  double res = this->coeff_a * exp(-this->coeff_b * x) + this->coeff_c;

  return res;
}

/*!
  Gets the value of the gain at infinity (ie the value of \f$ \lambda_\infty =
  c \f$). This function is similar to limitValue() except that here the value
  is not stored as a parameter of the class.

  \return It returns the value of the gain at infinity.
 */
double vpAdaptiveGain::limitValue_const(void) const
{
  double res = this->coeff_c;

  return res;
}

/*!
  Computes the value of the adaptive gain \f$\lambda(x)\f$ using:

  \f[ \lambda (x) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
  \lambda'_0}{\lambda_0 - \lambda_\infty}x} + \lambda_\infty \f]

  This value is stored as a parameter of the class.

  \param x : Input value to consider. During a visual servo this value can be
  the euclidian norm \f$||{\bf e}||\f$ or the infinity norm \f$||{\bf
  e}||_{\infty}\f$ of the task function.

  \return It returns the value of the computed gain.
  */
double vpAdaptiveGain::value(const double x) const
{
  this->lambda = this->value_const(x);

  return lambda;
}

/*!
  Gets the value of the gain at infinity (ie the value of \f$\lambda_\infty =
  c \f$) and stores it as a parameter of the class.

  \return It returns the value of the gain at infinity.
 */
double vpAdaptiveGain::limitValue(void) const
{
  this->lambda = this->limitValue_const();

  return lambda;
}

/* --------------------------------------------------------------------------
 */
/* --- ACCESSORS ------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!
  Operator that computes \f$\lambda(x)\f$ where

  \f[ \lambda (x) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
  \lambda'_0}{\lambda_0 - \lambda_\infty}x} + \lambda_\infty \f]

  \param x : Input value to consider. During a visual servo this value can be
  the euclidian norm \f$||{\bf e}||\f$ or the infinity norm \f$||{\bf
  e}||_{\infty}\f$ of the task function.

  \return It returns the value of the computed gain.

  \sa value()
*/
double vpAdaptiveGain::operator()(const double x) const { return this->value(x); }

/*!
  Gets the value of the gain at infinity (ie the value of \f$\lambda_\infty =
  c \f$).

  \return It returns the value of the gain at infinity.

  \sa limitValue()
 */
double vpAdaptiveGain::operator()(void) const { return this->limitValue(); }

/*!
  Operator which computes \f$\lambda({||x||}_{\infty})\f$ where

  \f[ \lambda ({||x||}_{\infty}) = (\lambda_0 - \lambda_\infty) e^{ -\frac{
  \lambda'_0}{\lambda_0 - \lambda_\infty}{||x||}_{\infty}} + \lambda_\infty
  \f]

  \param x : Input vector \f$ \bf x\f$ to consider.

  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::operator()(const vpColVector &x) const { return this->value(x.infinityNorm()); }

/* --------------------------------------------------------------------------
 */
/* --- OUTPUT ---------------------------------------------------------------
 */
/* --------------------------------------------------------------------------
 */

/*!
  Prints the adaptive gain parameters \f$\lambda_0, \lambda_\infty,
  \lambda'_0\f$.

  \param os : The stream where to print the adaptive gain parameters.
  \param lambda : The adaptive gain containing the parameters to print.
*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpAdaptiveGain &lambda)
{
  os << "Zero= " << lambda.coeff_a + lambda.coeff_c << "\tInf= " << lambda.coeff_c
     << "\tSlope= " << lambda.coeff_a * lambda.coeff_b;

  return os;
}
