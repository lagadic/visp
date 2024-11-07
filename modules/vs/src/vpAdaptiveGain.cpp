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
  \file vpAdaptiveGain.cpp
*/

/* --- VISP --- */
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/vs/vpAdaptiveGain.h>

#include <cmath> // std::fabs
#include <iostream>
#include <limits> // numeric_limits

BEGIN_VISP_NAMESPACE
const double vpAdaptiveGain::DEFAULT_LAMBDA_ZERO = 1.666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY = 0.1666;
const double vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE = 1.666;

vpAdaptiveGain::vpAdaptiveGain() : coeff_a(), coeff_b(), coeff_c(), lambda(1.)
{
  this->initFromVoid();

  return;
}

vpAdaptiveGain::vpAdaptiveGain(double c) : coeff_a(), coeff_b(), coeff_c(), lambda(1.) { initFromConstant(c); }

vpAdaptiveGain::vpAdaptiveGain(double gain_at_zero, double gain_at_infinity, double slope_at_zero)
  : coeff_a(), coeff_b(), coeff_c(), lambda(1.)
{
  initStandard(gain_at_zero, gain_at_infinity, slope_at_zero);
}

void vpAdaptiveGain::initFromConstant(const double c)
{
  this->coeff_a = 0;
  this->coeff_b = 1;
  this->coeff_c = c;
  return;
}

void vpAdaptiveGain::initFromVoid(void)
{
  this->initStandard(vpAdaptiveGain::DEFAULT_LAMBDA_ZERO, vpAdaptiveGain::DEFAULT_LAMBDA_INFINITY,
                     vpAdaptiveGain::DEFAULT_LAMBDA_SLOPE);
  return;
}

void vpAdaptiveGain::initStandard(double gain_at_zero, double gain_at_infinity, double slope_at_zero)
{
  this->coeff_a = gain_at_zero - gain_at_infinity;
  // if (0 == this ->coeff_a)
  if (std::fabs(this->coeff_a) <= std::numeric_limits<double>::epsilon()) {
    this->coeff_b = 0;
  }
  else {
    this->coeff_b = slope_at_zero / (this->coeff_a);
  }
  this->coeff_c = gain_at_infinity;

  return;
}

double vpAdaptiveGain::setConstant(void)
{
  double res = this->coeff_a + this->coeff_c;

  this->coeff_a = 0;
  this->coeff_b = 1;
  this->coeff_c = res;

  return res;
}

double vpAdaptiveGain::value_const(double x) const
{
  double res = this->coeff_a * exp(-this->coeff_b * x) + this->coeff_c;

  return res;
}

double vpAdaptiveGain::limitValue_const(void) const
{
  double res = this->coeff_c;

  return res;
}

double vpAdaptiveGain::value(double x) const
{
  this->lambda = this->value_const(x);

  return lambda;
}

double vpAdaptiveGain::limitValue(void) const
{
  this->lambda = this->limitValue_const();

  return lambda;
}

double vpAdaptiveGain::operator()(double x) const { return this->value(x); }

double vpAdaptiveGain::operator()(void) const { return this->limitValue(); }

double vpAdaptiveGain::operator()(const vpColVector &x) const { return this->value(x.infinityNorm()); }

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpAdaptiveGain &lambda)
{
  os << "Zero= " << lambda.coeff_a + lambda.coeff_c << "\tInf= " << lambda.coeff_c
    << "\tSlope= " << lambda.coeff_a * lambda.coeff_b;

  return os;
}

END_VISP_NAMESPACE
