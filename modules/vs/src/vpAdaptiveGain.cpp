/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#include <iostream>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

const double vpAdaptiveGain::DEFAULT_LAMBDA_ZERO = 1.666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY = 0.1666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE  = 1.666;

/* -------------------------------------------------------------------------- */
/* --- CONSTRUCTION --------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Basic constructor which initializes all the parameters with their default value:
  - \f$ \lambda(0) = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_ZERO
  - \f$ \lambda(\infty) = 0.1666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY
  - \f$ {\dot \lambda}(0) = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE

*/
vpAdaptiveGain::vpAdaptiveGain ()
  :
  coeff_a (),
  coeff_b (),
  coeff_c (),
  lambda(1.)
{
  vpDEBUG_TRACE (10, "# Entree constructeur par default.");
  this ->initFromVoid ();

  vpDEBUG_TRACE (10, "# Sortie constructeur par default.");
  return;
}

/*!
  Constructor that initializes the gain as constant. In that case \f$\lambda(x) = c\f$.

  \param c : Value of the constant gain.
*/
vpAdaptiveGain::vpAdaptiveGain (double c)
  :
  coeff_a (),
  coeff_b (),
  coeff_c (),
  lambda(1.)
{
  initFromConstant(c);
}

/*!
  Constructor that initializes the gain as adaptive.

  \param gain_at_zero : the expected gain when \f$x=0\f$: \f$\lambda(0)\f$.
  \param gain_at_infinity : the expected gain when \f$x=\infty\f$: \f$\lambda(\infty)\f$.
  \param slope_at_zero : the expected slope of \f$\lambda(x)\f$ when \f$x=0\f$: \f${\dot \lambda}(0)\f$.

*/
vpAdaptiveGain::vpAdaptiveGain (double gain_at_zero, double gain_at_infinity, double slope_at_zero)
  :
  coeff_a (),
  coeff_b (),
  coeff_c (),
  lambda(1.)
{
  initStandard(gain_at_zero, gain_at_infinity, slope_at_zero);
}

/* -------------------------------------------------------------------------- */
/* --- INIT ----------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Initializes the parameters to have a constant gain. In that case \f$\lambda(x) = c\f$.

  \param c : Value of the constant gain.
*/
void vpAdaptiveGain::initFromConstant (const double c)
{
    vpDEBUG_TRACE (10, "# Entree.");

    this ->coeff_a = 0;
    this ->coeff_b = 1;
    this ->coeff_c = c;

    vpDEBUG_TRACE (10, "# Sortie.");
    return;
}


/*!
  Initializes the parameters with the default value :
  - \f$ \lambda(0) = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_ZERO
  - \f$ \lambda(\infty) = 0.1666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY
  - \f$ {\dot \lambda}(0) = 1.666 \f$ using vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE
*/
void vpAdaptiveGain::initFromVoid (void)
{
  vpDEBUG_TRACE (10, "# Entree.");

  this ->initStandard (vpAdaptiveGain::DEFAULT_LAMBDA_ZERO,
                       vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY,
                       vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE);

  vpDEBUG_TRACE (10, "# Sortie.");
  return;
}


/*!
  Set the parameters \f$\lambda(0), \lambda(\infty), {\dot \lambda}(0)\f$ used to compute \f$\lambda(x)\f$.
  
  \param gain_at_zero : the expected gain when \f$x=0\f$: \f$\lambda(0)\f$.
  \param gain_at_infinity : the expected gain when \f$x=\infty\f$: \f$\lambda(\infty)\f$.
  \param slope_at_zero : the expected slope of \f$\lambda(x)\f$ when \f$x=0\f$: \f${\dot \lambda}(0)\f$.
*/
void vpAdaptiveGain::initStandard (const double gain_at_zero,
                                   const double gain_at_infinity,
                                   const double slope_at_zero)
{
  vpDEBUG_TRACE (10, "# Entree.");

  this ->coeff_a = gain_at_zero - gain_at_infinity;
  //if (0 == this ->coeff_a)
  if (std::fabs(this ->coeff_a) <= std::numeric_limits<double>::epsilon())
    {
      this ->coeff_b = 0;
    }
  else
    {
      this ->coeff_b = slope_at_zero / ( this ->coeff_a);
    }
  this ->coeff_c = gain_at_infinity;

  vpDEBUG_TRACE (10, "# Sortie :a,b,c= %.3f,%.3f,%.3f.",
	       this ->coeff_a, this ->coeff_b, this ->coeff_c);
  return;
}



/* -------------------------------------------------------------------------- */
/* --- MODIFICATOR ---------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Sets the internal parameters \f$a,b,c\f$ in order to obtain a constant gain equal to
  the gain in 0 set through the parameter \f$\lambda(0)\f$.
  
  \return It returns the value of the constant gain \f$\lambda(0)\f$.
*/
double vpAdaptiveGain::setConstant (void)
{
  vpDEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_a + this ->coeff_c;

  this ->coeff_a = 0;
  this ->coeff_b = 1;
  this ->coeff_c = res;

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}

/* -------------------------------------------------------------------------- */
/* --- VALEUR --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Computes the value of the adaptive gain \f$\lambda(x)\f$ using:

  \f[\lambda(x) = a * exp(-b*x) + c\f]

  \param x : Input value to consider. During a visual servo this value can be the euclidian
  norm \f$||s - s^*||\f$ or the infinity norm \f$||s - s^*||_{\infty}\f$ of the task function.
  
  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::value_const (const double x) const
{
  vpDEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_a * exp (- this ->coeff_b * x) + this ->coeff_c;

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}

/*!
  Gets the value of the gain at infinity (ie the value of \f$ \lambda(\infty) = c \f$).
  Similar to limitValue() except that here the value is not stored as a parameter of the class.

  \return It returns the value of the gain at infinity.
 */
double vpAdaptiveGain::limitValue_const (void) const
{
  vpDEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_c;

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}

/*!
  Computes the value of the adaptive gain \f$\lambda(x)\f$ using:

  \f[\lambda(x) = a * exp(-b*x) + c\f]

  This value is stored as a parameter of the class.

  \param x : Input value to consider. During a visual servo this value can be the euclidian
  norm \f$||s - s^*||\f$ or the infinity norm \f$||s - s^*||_{\infty}\f$ of the task function.

  \return It returns the value of the computed gain.
  */
double vpAdaptiveGain::value (const double x) const
{
  vpDEBUG_TRACE (10, "# Entree.");

  this ->lambda = this ->value_const (x);

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", this ->lambda);
  return lambda;
}


/*!
  Gets the value of the gain at infinity (ie the value of \f$\lambda(\infty) = c \f$) and stores it
  as a parameter of the class.

  \return It returns the value of the gain at infinity.
 */
double vpAdaptiveGain:: limitValue (void) const
{
  vpDEBUG_TRACE (10, "# Entree.");

  this ->lambda = this ->limitValue_const ();

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", this ->lambda);
  return lambda;
}

/* -------------------------------------------------------------------------- */
/* --- ACCESSORS ------------------------------------------------------------ */
/* -------------------------------------------------------------------------- */



// double vpAdaptiveGain::
// getLastValue (void) const
// {
//   return this ->lambda;
// }

/*!
  Operator that computes \f$\lambda(x)\f$.
     
  \param x : Input value to consider. During a visual servo this value can be the euclidian
  norm \f$||s - s^*||\f$ or the infinity norm \f$||s - s^*||_{\infty}\f$ of the task function.

  \return It returns the value of the computed gain.

  \sa value()
*/
double vpAdaptiveGain::operator() (const double x) const
{
  return this ->value (x);
}

/*!
  Gets the value of the gain at infinity (ie the value of \f$\lambda(\infty) = c \f$).

  \return It returns the value of the gain at infinity.

  \sa limitValue()
 */
double vpAdaptiveGain::operator() (void) const
{
  return this ->limitValue ();
}

/*!
  Operator which computes \f$\lambda({||x||}_{\infty})\f$.
     
  \param x : Input vector to consider.
      
  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::operator() (const vpColVector & x) const
{
  return this ->value (x .infinityNorm());
}


//   double operator() (double val_e)  const;
//   double operator()  (const CColVector & e) const;

/* -------------------------------------------------------------------------- */
/* --- OUTPUT --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */



/*!
  Prints the adaptive gain parameters \f$\lambda(0), \lambda(\infty), {\dot \lambda}(0)\f$.
  
  \param os : The stream where to print the adaptive gain parameters.
  \param lambda : The adaptive gain containing the parameters to print.
*/
VISP_EXPORT std::ostream& operator<< (std::ostream &os, const vpAdaptiveGain& lambda)
{
  os << "Zero= " << lambda .coeff_a + lambda .coeff_c
     << "\tInf= " << lambda .coeff_c
     << "\tDeriv= " << lambda .coeff_a * lambda .coeff_b;

  return os;
}
