/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * M-Estimator and various influence function.
 */

/*!
  \file vpRobust.cpp
*/

#include <algorithm> // std::swap
#include <cmath>     // std::fabs
#include <limits>    // numeric_limits
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRobust.h>

BEGIN_VISP_NAMESPACE
/*!
  Default constructor.
*/
vpRobust::vpRobust()
  : m_normres(), m_sorted_normres(), m_sorted_residues(), m_mad_min(0.0017), m_mad_prev(0),
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  m_iter(0),
#endif
  m_size(0), m_mad(0)
{ }

/*!
  Copy constructor.
*/
vpRobust::vpRobust(const vpRobust &other) { *this = other; }

/*!
  Copy operator.
 */
vpRobust &vpRobust::operator=(const vpRobust &other)
{
  m_normres = other.m_normres;
  m_sorted_normres = other.m_sorted_normres;
  m_sorted_residues = other.m_sorted_residues;
  m_mad_min = other.m_mad_min;
  m_mad = other.m_mad;
  m_mad_prev = other.m_mad_prev;
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  m_iter = other.m_iter;
#endif
  m_size = other.m_size;
  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Move operator.
 */
vpRobust &vpRobust::operator=(const vpRobust &&other)
{
  m_normres = std::move(other.m_normres);
  m_sorted_normres = std::move(other.m_sorted_normres);
  m_sorted_residues = std::move(other.m_sorted_residues);
  m_mad_min = std::move(other.m_mad_min);
  m_mad_prev = std::move(other.m_mad_prev);
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  m_iter = std::move(other.m_iter);
#endif
  m_size = std::move(other.m_size);
  return *this;
}
#endif

/*!
  Resize containers.
  \param n_data : size of input data vector.
*/
void vpRobust::resize(unsigned int n_data)
{
  if (n_data != m_size) {
    m_normres.resize(n_data);
    m_sorted_normres.resize(n_data);
    m_sorted_residues.resize(n_data);
    m_size = n_data;
  }
}

// ===================================================================
/*!

  Calculate an M-estimate given a particular influence function using MAD
  (Median Absolute Deviation) as a scale estimate at each iteration.

  \param[in] method : Type of influence function.

  \param[in] residues : Vector of residues \f$ r \f$ of the parameters to estimate.

  \param[out] weights : Vector of weights \f$w(r)\f$. Values are in [0, 1]. A value near zero
  means that the data is an outlier.
 */
void vpRobust::MEstimator(const vpRobustEstimatorType method, const vpColVector &residues, vpColVector &weights)
{
  double med = 0;        // median
  double normmedian = 0; // Normalized median

  // resize vector only if the size of residue vector has changed
  unsigned int n_data = residues.getRows();
  weights.resize(n_data, false);
  resize(n_data);

  m_sorted_residues = residues;

  unsigned int ind_med = static_cast<unsigned int>(ceil(n_data / 2.0)) - 1;

  // Calculate median
  med = select(m_sorted_residues, 0, n_data - 1, ind_med);
  // --comment: residualMedian = med

  // Normalize residues
  for (unsigned int i = 0; i < n_data; ++i) {
    m_normres[i] = (fabs(residues[i] - med));
    m_sorted_normres[i] = (fabs(m_sorted_residues[i] - med));
  }

  // Calculate MAD
  normmedian = select(m_sorted_normres, 0, n_data - 1, ind_med);
  // normalizedResidualMedian = normmedian ;
  // 1.48 keeps scale estimate consistent for a normal probability dist.
  m_mad = 1.4826 * normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (m_mad < m_mad_min) {
    m_mad = m_mad_min;
  }
  switch (method) {
  case TUKEY: {
    psiTukey(m_mad, m_normres, weights);
    break;
  }
  case CAUCHY: {
    psiCauchy(m_mad, m_normres, weights);
    break;
  }
  case HUBER: {
    psiHuber(m_mad, m_normres, weights);
    break;
  }
  default:
    // TODO
    std::cout << "MEstimator: method not recognised - id = " << method << std::endl;
  }
}

/*!
  Calculation of Tukey's influence function.

  \param sigma : sigma parameters.
  \param x : normalized residue vector.
  \param weights : weight vector.
*/

void vpRobust::psiTukey(double sig, const vpColVector &x, vpColVector &weights)
{
  unsigned int n_data = x.getRows();
  double C = sig * 4.6851;

  // Here we consider that sig cannot be equal to 0
  for (unsigned int i = 0; i < n_data; ++i) {
    double xi = x[i] / C;
    xi *= xi;

    if (xi > 1.) {
      weights[i] = 0;
    }
    else {
      xi = 1 - xi;
      xi *= xi;
      weights[i] = xi;
    }
  }
}

/*!
  Calculation of Tukey's influence function.

  \param sigma : sigma parameters.
  \param x : normalized residue vector.
  \param weights : weight vector.
*/
void vpRobust::psiHuber(double sig, const vpColVector &x, vpColVector &weights)
{
  double C = sig * 1.2107;
  unsigned int n_data = x.getRows();

  for (unsigned int i = 0; i < n_data; ++i) {
    double xi = x[i] / C;
    if (fabs(xi) > 1.) {
      weights[i] = std::fabs(1. / xi);
    }
    else {
      weights[i] = 1;
    }
  }
}

/*!
  Calculation of Cauchy's influence function.

  \param sigma : sigma parameter.
  \param x : normalized residue vector.
  \param weights : weight vector.
*/

void vpRobust::psiCauchy(double sig, const vpColVector &x, vpColVector &weights)
{
  unsigned int n_data = x.getRows();
  double C = sig * 2.3849;

  // Calculate Cauchy's equation
  for (unsigned int i = 0; i < n_data; ++i) {
    weights[i] = 1. / (1. + vpMath::sqr(x[i] / C));
  }
}

/*!
  Partition function.
  \param a : vector to be sorted.
  \param l : first value to be considered.
  \param r : last value to be considered.
*/
int vpRobust::partition(vpColVector &a, int l, int r)
{
  int i = l - 1;
  int j = r;
  double v = a[r];

  for (;;) {
    while (a[++i] < v) { }

    while (v < a[--j]) {
      if (j == l) {
        break;
      }
    }
    if (i >= j) {
      break;
    }
    std::swap(a[i], a[j]);
  }
  std::swap(a[i], a[r]);
  return i;
}

/*!
  \brief Sort a part of a vector and select a value of this new vector.
  \param a : vector to be sorted
  \param l : first value to be considered
  \param r : last value to be considered
  \param k : value to be selected
*/
double vpRobust::select(vpColVector &a, int l, int r, int k)
{
  while (r > l) {
    int i = partition(a, l, r);
    if (i >= k) {
      r = i - 1;
    }
    if (i <= k) {
      l = i + 1;
    }
  }
  return a[k];
}

/**********************
 * Below are deprecated functions
 */
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
#define vpITMAX 100
#define vpEPS 3.0e-7

/*!
  \deprecated You should rather use the default constructor.
  \param n_data : Size of the data vector.
*/
vpRobust::vpRobust(unsigned int n_data)
  : m_normres(), m_sorted_normres(), m_sorted_residues(), m_mad_min(0.0017), m_mad_prev(0),
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  m_iter(0),
#endif
  m_size(n_data), m_mad(0)
{
  m_normres.resize(n_data);
  m_sorted_normres.resize(n_data);
  m_sorted_residues.resize(n_data);
  // m_mad_min=0.0017; //Can not be more accurate than 1 pixel
}

void vpRobust::MEstimator(const vpRobustEstimatorType method, const vpColVector &residues,
                          const vpColVector &all_residues, vpColVector &weights)
{
  double normmedian = 0; // Normalized median

  unsigned int n_all_data = all_residues.getRows();
  vpColVector all_normres(n_all_data);

  // compute median with the residues vector, return all_normres which are the
  // normalized all_residues vector.
  normmedian = computeNormalizedMedian(all_normres, residues, all_residues, weights);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  m_mad = 1.4826 * normmedian; // Median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (m_mad < m_mad_min) {
    m_mad = m_mad_min;
  }

  switch (method) {
  case TUKEY: {
    psiTukey(m_mad, all_normres, weights);
    break;
  }
  case CAUCHY: {
    psiCauchy(m_mad, all_normres, weights);
    break;
  }
  case HUBER: {
    psiHuber(m_mad, all_normres, weights);
    break;
  }
  };
}

double vpRobust::computeNormalizedMedian(vpColVector &all_normres, const vpColVector &residues,
                                         const vpColVector &all_residues, const vpColVector &weights)
{
  double med = 0;
  double normmedian = 0;

  unsigned int n_all_data = all_residues.getRows();
  unsigned int n_data = residues.getRows();

  // resize vector only if the size of residue vector has changed
  resize(n_data);

  m_sorted_residues = residues;
  vpColVector no_null_weight_residues;
  no_null_weight_residues.resize(n_data);

  unsigned int index = 0;
  for (unsigned int j = 0; j < n_data; ++j) {
    // if(weights[j]!=0)
    if (std::fabs(weights[j]) > std::numeric_limits<double>::epsilon()) {
      no_null_weight_residues[index] = residues[j];
      index++;
    }
  }
  m_sorted_residues.resize(index);
  memcpy(m_sorted_residues.data, no_null_weight_residues.data, index * sizeof(double));
  n_data = index;

  // Calculate Median
  // Be careful to not use the rejected residues for the
  // calculation.

  unsigned int ind_med = (unsigned int)(ceil(n_data / 2.0)) - 1;
  med = select(m_sorted_residues, 0, n_data - 1, ind_med);

  // Normalize residues
  for (unsigned int i = 0; i < n_all_data; ++i) {
    all_normres[i] = (fabs(all_residues[i] - med));
  }

  for (unsigned int i = 0; i < n_data; ++i) {
    m_sorted_normres[i] = (fabs(m_sorted_residues[i] - med));
  }
  // MAD calculated only on first iteration
  normmedian = select(m_sorted_normres, 0, n_data - 1, ind_med);

  return normmedian;
}

/*!
 * \deprecated This function is useless.
 * Calculate an Mestimate with a simultaneous scale estimate using HUBER's influence function
 * \param[in] residues : Vector of residues. The content of the vector is changed.
 * \return Returns a vector of weights associated to each residue.
 */
vpColVector vpRobust::simultMEstimator(vpColVector &residues)
{
  double med = 0; // Median

  unsigned int n_data = residues.getRows();
  vpColVector norm_res(n_data); // Normalized Residue
  vpColVector w(n_data);

  // Calculate Median
  unsigned int ind_med = (unsigned int)(ceil(n_data / 2.0)) - 1;
  med = select(residues, 0, n_data - 1, ind_med /*(int)n_data/2*/);

  // Normalize residues
  for (unsigned int i = 0; i < n_data; ++i)
    norm_res[i] = (fabs(residues[i] - med));

  // Check for various methods.
  // For Huber compute Simultaneous scale estimate
  // For Others use MAD calculated on first iteration
  if (m_iter == 0) {
    double normmedian = select(norm_res, 0, n_data - 1, ind_med); // Normalized Median
    // 1.48 keeps scale estimate consistent for a normal probability dist.
    m_mad = 1.4826 * normmedian; // Median Absolute Deviation
  }
  else {
    // compute simultaneous scale estimate
    m_mad = simultscale(residues);
  }

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (m_mad < m_mad_min) {
    m_mad = m_mad_min;
  }

  psiHuber(m_mad, norm_res, w);

  m_mad_prev = m_mad;

  return w;
}

double vpRobust::simultscale(const vpColVector &x)
{
  unsigned int p = 6; // Number of parameters to be estimated.
  unsigned int n = x.getRows();
  double sigma2 = 0;
  /* long */ double Expectation = 0;
  /* long */ double Sum_chi = 0;

  for (unsigned int i = 0; i < n; ++i) {

    double chiTmp = simult_chi_huber(x[i]);
#if defined(VISP_HAVE_FUNC_STD_ERFC)
    Expectation += chiTmp * std::erfc(chiTmp);
#elif defined(VISP_HAVE_FUNC_ERFC)
    Expectation += chiTmp * erfc(chiTmp);
#else
    Expectation += chiTmp * (1 - erf(chiTmp));
#endif
    Sum_chi += chiTmp;

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
    {
#if defined(VISP_HAVE_FUNC_STD_ERFC)
      std::cout << "erf = " << std::erfc(chiTmp) << std::endl;
#elif defined(VISP_HAVE_FUNC_ERFC)
      std::cout << "erf = " << erfc(chiTmp) << std::endl;
#else
      std::cout << "erf = " << (1 - erf(chiTmp)) << std::endl;
#endif
      std::cout << "x[i] = " << x[i] << std::endl;
      std::cout << "chi = " << chiTmp << std::endl;
      std::cout << "Sum chi = " << chiTmp * vpMath::sqr(m_mad_prev) << std::endl;
#if defined(VISP_HAVE_FUNC_STD_ERFC)
      std::cout << "Expectation = " << chiTmp * std::erfc(chiTmp) << std::endl;
#elif defined(VISP_HAVE_FUNC_ERFC)
      std::cout << "Expectation = " << chiTmp * erfc(chiTmp) << std::endl;
#else
      std::cout << "Expectation = " << chiTmp * (1 - erf(chiTmp)) << std::endl;
#endif
      // getchar();
    }
#endif
#endif
  }

  sigma2 = Sum_chi * vpMath::sqr(m_mad_prev) / ((n - p) * Expectation);

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
  {
    std::cout << "Expectation = " << Expectation << std::endl;
    std::cout << "Sum chi = " << Sum_chi << std::endl;
    std::cout << "MAD prev" << m_mad_prev << std::endl;
    std::cout << "sig_out" << sqrt(fabs(sigma2)) << std::endl;
  }
#endif
#endif

  return sqrt(fabs(sigma2));
}

double vpRobust::constrainedChi(vpRobustEstimatorType method, double x)
{
  switch (method) {
  case TUKEY:
    return constrainedChiTukey(x);
  case CAUCHY:
    return constrainedChiCauchy(x);
  case HUBER:
    return constrainedChiHuber(x);
  };

  return -1;
}

double vpRobust::constrainedChiTukey(double x)
{
  double sct = 0;
  double s = m_mad_prev;
  // double epsillon=0.5;

  if (fabs(x) <= 4.7 * m_mad_prev) {
    double a = 4.7;
    // sct =
    // (vpMath::sqr(s*a-x)*vpMath::sqr(s*a+x)*vpMath::sqr(x))/(s*vpMath::sqr(vpMath::sqr(a*vpMath::sqr(s))));
    sct = (vpMath::sqr(s * a) * x - s * vpMath::sqr(s * a) - x * vpMath::sqr(x)) *
      (vpMath::sqr(s * a) * x + s * vpMath::sqr(s * a) - x * vpMath::sqr(x)) / s *
      vpMath::sqr(vpMath::sqr(vpMath::sqr(s))) / vpMath::sqr(vpMath::sqr(a));
  }
  else
    sct = -1 / s;

  return sct;
}

double vpRobust::constrainedChiCauchy(double x)
{
  double sct = 0;
  // double u = x/m_mad_prev;
  double s = m_mad_prev;
  double b = 2.3849;

  sct = -1 * (vpMath::sqr(x) * b) / (s * (vpMath::sqr(s * b) + vpMath::sqr(x)));

  return sct;
}

double vpRobust::constrainedChiHuber(double x)
{
  double sct = 0;
  double u = x / m_mad_prev;
  double c = 1.2107; // 1.345;

  if (fabs(u) <= c)
    sct = vpMath::sqr(u);
  else
    sct = vpMath::sqr(c);

  return sct;
}

double vpRobust::simult_chi_huber(double x)
{
  double sct = 0;
  double u = x / m_mad_prev;
  double c = 1.2107; // 1.345;

  if (fabs(u) <= c) {
    // sct = 0.5*vpMath::sqr(u);
    sct = vpMath::sqr(u);
  }
  else {
    // sct = 0.5*vpMath::sqr(c);
    sct = vpMath::sqr(c);
  }

  return sct;
}

#if !defined(VISP_HAVE_FUNC_ERFC) && !defined(VISP_HAVE_FUNC_STD_ERFC)
double vpRobust::erf(double x) { return x < 0.0 ? -gammp(0.5, x * x) : gammp(0.5, x * x); }

double vpRobust::gammp(double a, double x)
{
  double gamser = 0., gammcf = 0., gln;

  if (x < 0.0 || a <= 0.0)
    std::cout << "Invalid arguments in routine GAMMP";
  if (x < (a + 1.0)) {
    gser(&gamser, a, x, &gln);
    return gamser;
  }
  else {
    gcf(&gammcf, a, x, &gln);
    return 1.0 - gammcf;
  }
}

void vpRobust::gser(double *gamser, double a, double x, double *gln)
{
  *gln = gammln(a);
  if (x <= 0.0) {
    if (x < 0.0)
      std::cout << "x less than 0 in routine GSER";
    *gamser = 0.0;
    return;
  }
  else {
    double ap = a;
    double sum = 1.0 / a;
    double del = sum;
    for (int n = 1; n <= vpITMAX; ++n) {
      ap += 1.0;
      del *= x / ap;
      sum += del;
      if (fabs(del) < fabs(sum) * vpEPS) {
        *gamser = sum * exp(-x + a * log(x) - (*gln));
        return;
      }
    }
    std::cout << "a too large, vpITMAX too small in routine GSER";
    return;
  }
}

void vpRobust::gcf(double *gammcf, double a, double x, double *gln)
{
  double gold = 0.0, g, fac = 1.0, b1 = 1.0;
  double b0 = 0.0, a1, a0 = 1.0;

  *gln = gammln(a);
  a1 = x;
  for (int n = 1; n <= vpITMAX; ++n) {
    double an = static_cast<double>(n);
    double ana = an - a;
    a0 = (a1 + a0 * ana) * fac;
    b0 = (b1 + b0 * ana) * fac;
    double anf = an * fac;
    a1 = x * a0 + anf * a1;
    b1 = x * b0 + anf * b1;
    // if (a1)
    if (std::fabs(a1) > std::numeric_limits<double>::epsilon()) {
      fac = 1.0 / a1;
      g = b1 * fac;
      if (fabs((g - gold) / g) < vpEPS) {
        *gammcf = exp(-x + a * log(x) - (*gln)) * g;
        return;
      }
      gold = g;
    }
  }
  std::cout << "a too large, vpITMAX too small in routine GCF";
}

double vpRobust::gammln(double xx)
{
  double x, tmp, ser;
  static double cof[6] = { 76.18009173, -86.50532033, 24.01409822, -1.231739516, 0.120858003e-2, -0.536382e-5 };

  x = xx - 1.0;
  tmp = x + 5.5;
  tmp -= (x + 0.5) * log(tmp);
  ser = 1.0;
  for (int j = 0; j <= 5; ++j) {
    x += 1.0;
    ser += cof[j] / x;
  }
  return -tmp + log(2.50662827465 * ser);
}
#endif

#undef vpITMAX
#undef vpEPS

#endif
END_VISP_NAMESPACE
