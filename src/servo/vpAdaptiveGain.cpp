/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>
#include <visp/vpAdaptiveGain.h>

#include <iostream>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

const double vpAdaptiveGain::DEFAULT_LAMBDA_ZERO = 1.666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_INFINI = 0.1666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_PENTE  = 1.666;

/* -------------------------------------------------------------------------- */
/* --- CONSTRUCTION --------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Basic constructor which initializes the parameters with a default value.
*/
vpAdaptiveGain::
vpAdaptiveGain (void)
  :
  coeff_a (),
  coeff_b (),
  coeff_c ()
{
  lambda = 1.;
  vpDEBUG_TRACE (10, "# Entree constructeur par default.");
  this ->initFromVoid ();

  vpDEBUG_TRACE (10, "# Sortie constructeur par default.");
  return;
}

/* -------------------------------------------------------------------------- */
/* --- INIT ----------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Initializes the parameters to have a constant gain.
  Thus \f$ a = 0 \f$, \f$ b = 1 \f$ and \f$ c = lambda \f$
  
  \param lambda : the expected constant gain.
*/
void vpAdaptiveGain::
initFromConstant (const double lambda)
{
    vpDEBUG_TRACE (10, "# Entree.");

    this ->coeff_a = 0;
    this ->coeff_b = 1;
    this ->coeff_c = lambda;

    vpDEBUG_TRACE (10, "# Sortie.");
    return;
}


/*!
  Initializes the parameters with the default value :
  - \f$ lambda(0) = 1.666 \f$
  - \f$ lambda(inf) = 0.1666 \f$
  - \f$ lambda'(0) = 1.666 \f$
*/
void vpAdaptiveGain::
initFromVoid (void)
{
  vpDEBUG_TRACE (10, "# Entree.");

  this ->initStandard (vpAdaptiveGain::DEFAULT_LAMBDA_ZERO,
		       vpAdaptiveGain::DEFAULT_LAMBDA_INFINI,
		       vpAdaptiveGain::DEFAULT_LAMBDA_PENTE);

  vpDEBUG_TRACE (10, "# Sortie.");
  return;
}


/*!
  Computes the parameters thanks to the given \f$ lambda(0)\f$, \f$ lambda(inf)\f$ and \f$ lambda'(0)\f$.
  
  \f$ lambda(0)\f$ represents the gain in 0, \f$ lambda(inf)\f$ represents the gain to infinity and \f$ lambda'(0)\f$ represents the slope in 0.
  
  \param en_zero : the expected gain in 0.
  \param en_infini : the expected gain to infinity.
  \param pente_en_zero : the expected slope in 0.
*/
void vpAdaptiveGain::
initStandard (const double en_zero,
	      const double en_infini,
	      const double pente_en_zero)
{
  vpDEBUG_TRACE (10, "# Entree.");

  this ->coeff_a = en_zero - en_infini;
  //if (0 == this ->coeff_a)
  if (std::fabs(this ->coeff_a) <= std::numeric_limits<double>::epsilon())
    {
      this ->coeff_b = 0;
    }
  else
    {
      this ->coeff_b = pente_en_zero / ( this ->coeff_a);
    }
  this ->coeff_c = en_infini;

  vpDEBUG_TRACE (10, "# Sortie :a,b,c= %.3f,%.3f,%.3f.",
	       this ->coeff_a, this ->coeff_b, this ->coeff_c);
  return;
}



/* -------------------------------------------------------------------------- */
/* --- MODIFICATOR ---------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/*!
  Sets the parameters in order to obtain a constant gain equal to the gain in 0.
  
  \return It returns the value of the computed constant gain.
*/
double vpAdaptiveGain::
setConstant (void)
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
  Computes the value of the adaptive gain \f$\lambda\f$ corresponding to
  the norm of the task function. The formula is the following:

  \f[\lambda = a * exp(-b*val_e) + c\f]

  \param val_e : Norm of the task function \f$\mid s - s^*\mid\f$.
  
  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::
value_const (const double val_e) const
{
  vpDEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_a * exp (- this ->coeff_b * val_e)
    + this ->coeff_c;

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}

/*!
  Gets the value of the gain at infinity (ie the value of \f$ c \f$).

  \return It returns the value of the gain at infinity (ie the value of \f$ c \f$).
 */
double vpAdaptiveGain::
limitValue_const (void) const
{
  vpDEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_c;

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}



/*!
  Computes the value of the adaptive gain \f$\lambda\f$ corresponding to
  the norm of the task function and stores it as a parameter of the class.
  The formula used for the gain computation is the following:

  \f[\lambda = a * exp(-b*val_e) + c\f]

  \param val_e : Norm of the task function \f$\mid s - s^*\mid\f$.
  
  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::
value (const double val_e) const
{
  vpDEBUG_TRACE (10, "# Entree.");

  this ->lambda = this ->value_const (val_e);

  vpDEBUG_TRACE (10, "# Sortie: %.3f.", this ->lambda);
  return lambda;
}


/*!
  Gets the value of the gain at infinity (ie the value of \f$ c \f$)and stores it
  as a parameter of the class.

  \return It returns the value of the gain at infinity (ie the value of \f$ c \f$).
 */
double vpAdaptiveGain::
limitValue (void) const
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
  Operator which calls the value(double val_e) method with \e val_e in parameter in order
  to compute the adaptive gain corresponding to \e val_e.
     
  \param val_e : Norm of the task function \f$\mid s - s^*\mid\f$.
      
  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::
operator() (const double val_e) const
{
  return this ->value (val_e);
}

/*!
  Gets the value of the gain at infinity (ie the value of \f$ c \f$).

  \return It returns the value of the gain at infinity (ie the value of \f$ c \f$).
 */
double vpAdaptiveGain::
operator() (void) const
{
  return this ->limitValue ();
}

/*!
  Operator which calls the value(double val_e) method with the infinity norm of \e e in parameter in order
  to compute the adaptive gain corresponding to \f$ |e| \f$.
     
  \param e : the task function \f$\mid s - s^*\mid\f$.
      
  \return It returns the value of the computed gain.
*/
double vpAdaptiveGain::
operator()  (const vpColVector & e) const
{
  return this ->value (e .infinityNorm());
}


//   double operator() (double val_e)  const;
//   double operator()  (const CColVector & e) const;

/* -------------------------------------------------------------------------- */
/* --- OUTPUT --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */



/*!
  Prints the adaptive gain  coefficients. It prints the gain in 0, 
  the gain to infinity and the slope in 0.
  
  \param os : The stream where to print the adaptive gain parameters.
  \param lambda : The adaptive gain containing the parameters to print.
*/
std::ostream&
operator<< (std::ostream &os, const vpAdaptiveGain& lambda)
{
  os << "Zero= " << lambda .coeff_a + lambda .coeff_c
     << "\tInf= " << lambda .coeff_c
     << "\tDeriv= " << lambda .coeff_a * lambda .coeff_b;

  return os;
}



/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
