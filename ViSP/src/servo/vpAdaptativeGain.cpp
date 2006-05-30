/****************************************************************************
 *
 * $Id: vpAdaptativeGain.cpp,v 1.4 2006-05-30 08:40:45 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
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
 * Adaptative gain.
 *
 * Authors:
 * Nicolas Mansard
 *
 *****************************************************************************/

#include <iostream>

/* --- VISP --- */
#include <visp/vpColVector.h>
#include <visp/vpDebug.h>
#include <visp/vpAdaptativeGain.h>


const double vpAdaptativeGain::DEFAULT_LAMBDA_ZERO = 1.666;
const double vpAdaptativeGain::DEFAULT_LAMBDA_INFINI = 0.1666;
const double vpAdaptativeGain::DEFAULT_LAMBDA_PENTE  = 1.666;

/* -------------------------------------------------------------------------- */
/* --- CONSTRUCTION --------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Initialisation aux valeurs par default. */
vpAdaptativeGain::
vpAdaptativeGain (void)
  :
  coeff_a (),
  coeff_b (),
  coeff_c ()
{
  DEBUG_TRACE (10, "# Entree constructeur par default.");
  this ->initFromVoid ();

  DEBUG_TRACE (10, "# Sortie constructeur par default.");
  return;
}

/* -------------------------------------------------------------------------- */
/* --- INIT ----------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Initialisation pour un gain constant (l0 = linf).
 */
void vpAdaptativeGain::
initFromConstant (const double lambda)
{
    DEBUG_TRACE (10, "# Entree.");

    this ->coeff_a = 0;
    this ->coeff_b = 1;
    this ->coeff_c = lambda;

    DEBUG_TRACE (10, "# Sortie.");
    return;
}


/* Initialisation par default (faite apres construction).
 */
void vpAdaptativeGain::
initFromVoid (void)
{
  DEBUG_TRACE (10, "# Entree.");

  this ->initStandard (vpAdaptativeGain::DEFAULT_LAMBDA_ZERO,
		       vpAdaptativeGain::DEFAULT_LAMBDA_INFINI,
		       vpAdaptativeGain::DEFAULT_LAMBDA_PENTE);

  DEBUG_TRACE (10, "# Sortie.");
  return;
}


/* Calcule les trois parametres a, b et c a partir de la valeur lambda
 * a l'infini (c'est a dire la valeur theorique dans les equations), de
 * la valeur en 0, et de la pente en 0.
 */
void vpAdaptativeGain::
initStandard (const double en_zero,
	      const double en_infini,
	      const double pente_en_zero)
{
  DEBUG_TRACE (10, "# Entree.");

  this ->coeff_a = en_zero - en_infini;
  if (0 == this ->coeff_a)
    {
      this ->coeff_b = 0;
    }
  else
    {
      this ->coeff_b = pente_en_zero / ( this ->coeff_a);
    }
  this ->coeff_c = en_infini;

  DEBUG_TRACE (10, "# Sortie :a,b,c= %.3f,%.3f,%.3f.",
	       this ->coeff_a, this ->coeff_b, this ->coeff_c);
  return;
}



/* -------------------------------------------------------------------------- */
/* --- MODIFICATOR ---------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Passe le gain adaptatif en gain constant, egal a la valeur du gain
 * adaptatif en 0.
 * OUTPUT
 *   - renvoie la valeur du gain constant.
 */
double vpAdaptativeGain::
setConstant (void)
{
  DEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_a + this ->coeff_c;

  this ->coeff_a = 0;
  this ->coeff_b = 1;
  this ->coeff_c = res;

  DEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}

/* -------------------------------------------------------------------------- */
/* --- VALEUR --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */

/* Determine la valeur du lambda adaptatif en fonction de la valeur
 * de la norme de la fonction de tache e par extrapolation exponentielle.
 */
double vpAdaptativeGain::
value_const (const double val_e) const
{
  DEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_a * exp (- this ->coeff_b * val_e)
    + this ->coeff_c;

  DEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}

/* Renvoie la valeur en l'infini.
 */
double vpAdaptativeGain::
limitValue_const (void) const
{
  DEBUG_TRACE (10, "# Entree.");

  double res = this ->coeff_c;

  DEBUG_TRACE (10, "# Sortie: %.3f.", res);
  return res;
}



/* Determine la valeur du lambda adaptatif en fonction de la valeur
 * de la norme de la fonction de tache e par extrapolation exponentielle.
 */
double vpAdaptativeGain::
value (const double val_e) const
{
  DEBUG_TRACE (10, "# Entree.");

  this ->lambda = this ->value_const (val_e);

  DEBUG_TRACE (10, "# Sortie: %.3f.", this ->lambda);
  return lambda;
}

/* Renvoie la valeur en l'infini.
 */
double vpAdaptativeGain::
limitValue (void) const
{
  DEBUG_TRACE (10, "# Entree.");

  this ->lambda = this ->limitValue_const ();

  DEBUG_TRACE (10, "# Sortie: %.3f.", this ->lambda);
  return lambda;
}

/* -------------------------------------------------------------------------- */
/* --- ACCESSORS ------------------------------------------------------------ */
/* -------------------------------------------------------------------------- */


/* Renvoie la derniere valeur stockee dans this ->lambda. */
double vpAdaptativeGain::
getLastValue (void) const
{
  return this ->lambda;
}


double vpAdaptativeGain::
operator() (const double val_e) const
{
  return this ->value (val_e);
}

double vpAdaptativeGain::
operator() (void) const
{
  return this ->limitValue ();
}

double vpAdaptativeGain::
operator()  (const vpColVector & e) const
{
  return this ->value (e .infinityNorm());
}


//   double operator() (double val_e)  const;
//   double operator()  (const CColVector & e) const;

/* -------------------------------------------------------------------------- */
/* --- OUTPUT --------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


using namespace std;

ostream&
operator<< (ostream &os, const vpAdaptativeGain& lambda)
{
  os << "Zero= " << lambda .coeff_a + lambda .coeff_c
     << "\tInf= " << lambda .coeff_c
     << "\tDeriv= " << lambda .coeff_a * lambda .coeff_b;

  return os;
}


/** \file $Source: /udd/fspindle/poub/cvs2svn/ViSP/cvsroot/visp/ViSP/src/servo/vpAdaptativeGain.cpp,v $ */
/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
