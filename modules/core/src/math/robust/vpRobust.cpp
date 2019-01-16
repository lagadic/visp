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
 * M-Estimator and various influence function.
 *
 * Authors:
 * Andrew Comport
 * Jean Laneurit
 *
 *****************************************************************************/

/*!
  \file vpRobust.cpp
*/

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <visp3/core/vpRobust.h>

#define vpITMAX 100
#define vpEPS 3.0e-7
#define vpCST 1

// ===================================================================
/*!
  \brief Constructor.
  \param n_data : Size of the data vector.

*/
vpRobust::vpRobust(unsigned int n_data)
  : normres(), sorted_normres(), sorted_residues(), NoiseThreshold(0.0017), sig_prev(0), it(0), swap(0), size(n_data)
{
  vpCDEBUG(2) << "vpRobust constructor reached" << std::endl;

  normres.resize(n_data);
  sorted_normres.resize(n_data);
  sorted_residues.resize(n_data);
  // NoiseThreshold=0.0017; //Can not be more accurate than 1 pixel
}

/*!
  Default constructor.
*/
vpRobust::vpRobust()
  : normres(), sorted_normres(), sorted_residues(), NoiseThreshold(0.0017), sig_prev(0), it(0), swap(0), size(0)
{
}

/*!
  Copy constructor.
*/
vpRobust::vpRobust(const vpRobust &other) { *this = other; }

/*!
  Copy operator.
 */
vpRobust &vpRobust::operator=(const vpRobust &other)
{
  normres = other.normres;
  sorted_normres = other.sorted_normres;
  sorted_residues = other.sorted_residues;
  NoiseThreshold = other.NoiseThreshold;
  sig_prev = other.sig_prev;
  it = other.it;
  swap = other.swap;
  size = other.size;
  return *this;
}

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
/*!
  Move operator.
 */
vpRobust &vpRobust::operator=(const vpRobust &&other)
{
  normres = std::move(other.normres);
  sorted_normres = std::move(other.sorted_normres);
  sorted_residues = std::move(other.sorted_residues);
  NoiseThreshold = std::move(other.NoiseThreshold);
  sig_prev = std::move(other.sig_prev);
  it = std::move(other.it);
  swap = std::move(other.swap);
  size = std::move(other.size);
  return *this;
}
#endif

/*!
  \brief Resize containers.
  \param n_data : size of input data vector.

*/
void vpRobust::resize(unsigned int n_data)
{

  if (n_data != size) {
    normres.resize(n_data);
    sorted_normres.resize(n_data);
    sorted_residues.resize(n_data);
    size = n_data;
  }
}

// ===================================================================
/*!

  \brief Calculate an Mestimate given a particular loss function using MAD
  (Median Absolute Deviation) as a scale estimate at each iteration.

  \pre Requires a column vector of residues.

  \post Keeps a copy of the weights so that rejected points are kept at zero
  weight.

  \param method : Type of M-Estimator \f$\rho(r_i)\f$:

  - TUKEY : \f$ \rho(r_i, C) = \left\{
  \begin{array}{ll}
  \frac{r_i^6}{6} - \frac{C^2r_i^4}{2} +\frac{C^4r_i^2}{2} & \mbox{if} |r_i| <
  C \\ \frac{1}{6} C^6 & \mbox{else} \end{array} \right. \f$ with influence
  function \f$ \psi(r_i, C) = \left\{ \begin{array}{ll} r_i(r_i^2-C^2)^2 &
  \mbox{if} |r_i| < C \\ 0 & \mbox{else} \end{array} \right. \f$ where
  \f$C=4.7 \hat{\sigma} \f$ and with \f$ \hat{\sigma} = 1.48{Med}_i(|r_i -
  {Med}_j(r_j)|) \f$

  - CAUCHY :

  - HUBER :

  \param residues : Residues \f$ r_i \f$ used in the previous formula.

  \param weights : Vector of weights \f$w_i =
  \frac{\psi(r_i)}{r_i}\f$. Values are in [0, 1]. A value near zero
  means that the data is an outlier. This vector must have the same size
  residue vector.

  \return Returns a Column Vector of weights associated to each residue.
 */

// ===================================================================
void vpRobust::MEstimator(const vpRobustEstimatorType method, const vpColVector &residues, vpColVector &weights)
{

  double med = 0;        // median
  double normmedian = 0; // Normalized median
  double sigma = 0;      // Standard Deviation

  // resize vector only if the size of residue vector has changed
  unsigned int n_data = residues.getRows();
  resize(n_data);

  sorted_residues = residues;

  unsigned int ind_med = (unsigned int)(ceil(n_data / 2.0)) - 1;

  // Calculate median
  med = select(sorted_residues, 0, (int)n_data - 1, (int)ind_med /*(int)n_data/2*/);
  // residualMedian = med ;

  // Normalize residues
  for (unsigned int i = 0; i < n_data; i++) {
    normres[i] = (fabs(residues[i] - med));
    sorted_normres[i] = (fabs(sorted_residues[i] - med));
  }

  // Calculate MAD
  normmedian = select(sorted_normres, 0, (int)n_data - 1, (int)ind_med /*(int)n_data/2*/);
  // normalizedResidualMedian = normmedian ;
  // 1.48 keeps scale estimate consistent for a normal probability dist.
  sigma = 1.4826 * normmedian; // median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  switch (method) {
  case TUKEY: {
    psiTukey(sigma, normres, weights);

    vpCDEBUG(2) << "Tukey's function computed" << std::endl;
    break;
  }
  case CAUCHY: {
    psiCauchy(sigma, normres, weights);
    break;
  }
  case HUBER: {
    psiHuber(sigma, normres, weights);
    break;
  }
  }
}

void vpRobust::MEstimator(const vpRobustEstimatorType method, const vpColVector &residues,
                          const vpColVector &all_residues, vpColVector &weights)
{

  double normmedian = 0; // Normalized median
  double sigma = 0;      // Standard Deviation

  unsigned int n_all_data = all_residues.getRows();
  vpColVector all_normres(n_all_data);

  // compute median with the residues vector, return all_normres which are the
  // normalized all_residues vector.
  normmedian = computeNormalizedMedian(all_normres, residues, all_residues, weights);

  // 1.48 keeps scale estimate consistent for a normal probability dist.
  sigma = 1.4826 * normmedian; // Median Absolute Deviation

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  switch (method) {
  case TUKEY: {
    psiTukey(sigma, all_normres, weights);

    vpCDEBUG(2) << "Tukey's function computed" << std::endl;
    break;
  }
  case CAUCHY: {
    psiCauchy(sigma, all_normres, weights);
    break;
  }
  case HUBER: {
    psiHuber(sigma, all_normres, weights);
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

  sorted_residues = residues;
  vpColVector no_null_weight_residues;
  no_null_weight_residues.resize(n_data);

  // all_normres.resize(n_all_data); // Normalized Residue
  // vpColVector sorted_normres(n_data); // Normalized Residue
  // vpColVector sorted_residues = residues;
  // vpColVector sorted_residues;

  unsigned int index = 0;
  for (unsigned int j = 0; j < n_data; j++) {
    // if(weights[j]!=0)
    if (std::fabs(weights[j]) > std::numeric_limits<double>::epsilon()) {
      no_null_weight_residues[index] = residues[j];
      index++;
    }
  }
  sorted_residues.resize(index);
  memcpy(sorted_residues.data, no_null_weight_residues.data, index * sizeof(double));
  n_data = index;

  vpCDEBUG(2) << "vpRobust MEstimator reached. No. data = " << n_data << std::endl;

  // Calculate Median
  // Be careful to not use the rejected residues for the
  // calculation.

  unsigned int ind_med = (unsigned int)(ceil(n_data / 2.0)) - 1;
  med = select(sorted_residues, 0, (int)n_data - 1, (int)ind_med /*(int)n_data/2*/);

  unsigned int i;
  // Normalize residues
  for (i = 0; i < n_all_data; i++) {
    all_normres[i] = (fabs(all_residues[i] - med));
  }

  for (i = 0; i < n_data; i++) {
    sorted_normres[i] = (fabs(sorted_residues[i] - med));
  }
  // MAD calculated only on first iteration

  // normmedian = Median(normres, weights);
  // normmedian = Median(normres);
  normmedian = select(sorted_normres, 0, (int)n_data - 1, (int)ind_med /*(int)n_data/2*/);

  return normmedian;
}

// ===================================================================
/*!
 * \brief Calculate an Mestimate with a simultaneous scale estimate
 *				using HUBER's influence function
 * \pre Requires a column vector of residues
 * \post None
 * \return Returns a Column Vector of weights associated to each residue
 */
// ===================================================================
vpColVector vpRobust::simultMEstimator(vpColVector &residues)
{

  double med = 0;   // Median
  double sigma = 0; // Standard Deviation

  unsigned int n_data = residues.getRows();
  vpColVector norm_res(n_data); // Normalized Residue
  vpColVector w(n_data);

  vpCDEBUG(2) << "vpRobust MEstimator reached. No. data = " << n_data << std::endl;

  // Calculate Median
  unsigned int ind_med = (unsigned int)(ceil(n_data / 2.0)) - 1;
  med = select(residues, 0, (int)n_data - 1, (int)ind_med /*(int)n_data/2*/);

  // Normalize residues
  for (unsigned int i = 0; i < n_data; i++)
    norm_res[i] = (fabs(residues[i] - med));

  // Check for various methods.
  // For Huber compute Simultaneous scale estimate
  // For Others use MAD calculated on first iteration
  if (it == 0) {
    double normmedian = select(norm_res, 0, (int)n_data - 1, (int)ind_med /*(int)n_data/2*/); // Normalized Median
    // 1.48 keeps scale estimate consistent for a normal probability dist.
    sigma = 1.4826 * normmedian; // Median Absolute Deviation
  } else {
    // compute simultaneous scale estimate
    sigma = simultscale(residues);
  }

  // Set a minimum threshold for sigma
  // (when sigma reaches the level of noise in the image)
  if (sigma < NoiseThreshold) {
    sigma = NoiseThreshold;
  }

  vpCDEBUG(2) << "MAD and C computed" << std::endl;

  psiHuber(sigma, norm_res, w);

  sig_prev = sigma;

  return w;
}

double vpRobust::simultscale(vpColVector &x)
{
  unsigned int p = 6; // Number of parameters to be estimated.
  unsigned int n = x.getRows();
  double sigma2 = 0;
  /* long */ double Expectation = 0;
  /* long */ double Sum_chi = 0;

  for (unsigned int i = 0; i < n; i++) {

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
      std::cout << "Sum chi = " << chiTmp * vpMath::sqr(sig_prev) << std::endl;
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

  sigma2 = Sum_chi * vpMath::sqr(sig_prev) / ((n - p) * Expectation);

#ifdef VP_DEBUG
#if VP_DEBUG_MODE == 3
  {
    std::cout << "Expectation = " << Expectation << std::endl;
    std::cout << "Sum chi = " << Sum_chi << std::endl;
    std::cout << "sig_prev" << sig_prev << std::endl;
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
  double s = sig_prev;
  // double epsillon=0.5;

  if (fabs(x) <= 4.7 * sig_prev) {
    double a = 4.7;
    // sct =
    // (vpMath::sqr(s*a-x)*vpMath::sqr(s*a+x)*vpMath::sqr(x))/(s*vpMath::sqr(vpMath::sqr(a*vpMath::sqr(s))));
    sct = (vpMath::sqr(s * a) * x - s * vpMath::sqr(s * a) - x * vpMath::sqr(x)) *
          (vpMath::sqr(s * a) * x + s * vpMath::sqr(s * a) - x * vpMath::sqr(x)) / s *
          vpMath::sqr(vpMath::sqr(vpMath::sqr(s))) / vpMath::sqr(vpMath::sqr(a));
  } else
    sct = -1 / s;

  return sct;
}

double vpRobust::constrainedChiCauchy(double x)
{
  double sct = 0;
  // double u = x/sig_prev;
  double s = sig_prev;
  double b = 2.3849;

  sct = -1 * (vpMath::sqr(x) * b) / (s * (vpMath::sqr(s * b) + vpMath::sqr(x)));

  return sct;
}

double vpRobust::constrainedChiHuber(double x)
{
  double sct = 0;
  double u = x / sig_prev;
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
  double u = x / sig_prev;
  double c = 1.2107; // 1.345;

  if (fabs(u) <= c) {
    // sct = 0.5*vpMath::sqr(u);
    sct = vpMath::sqr(u);
  } else {
    // sct = 0.5*vpMath::sqr(c);
    sct = vpMath::sqr(c);
  }

  return sct;
}

/*!
  \brief calculation of Tukey's influence function

  \param sigma : sigma parameters
  \param x : normalized residue vector
  \param weights : weight vector
*/

void vpRobust::psiTukey(double sig, vpColVector &x, vpColVector &weights)
{

  unsigned int n_data = x.getRows();
  double cst_const = vpCST * 4.6851;

  for (unsigned int i = 0; i < n_data; i++) {
    // if(sig==0 && weights[i]!=0)
    if (std::fabs(sig) <= std::numeric_limits<double>::epsilon() &&
        std::fabs(weights[i]) > std::numeric_limits<double>::epsilon()) {
      weights[i] = 1;
      continue;
    }

    double xi_sig = x[i] / sig;

    // if((fabs(xi_sig)<=(cst_const)) && weights[i]!=0)
    if ((std::fabs(xi_sig) <= (cst_const)) && std::fabs(weights[i]) > std::numeric_limits<double>::epsilon()) {
      weights[i] = vpMath::sqr(1 - vpMath::sqr(xi_sig / cst_const));
      // w[i] = vpMath::sqr(1-vpMath::sqr(x[i]/sig/4.7));
    } else {
      // Outlier - could resize list of points tracked here?
      weights[i] = 0;
    }
  }
}

/*!
  \brief calculation of Tukey's influence function

  \param sigma : sigma parameters
  \param x : normalized residue vector
  \param weights : weight vector
*/
void vpRobust::psiHuber(double sig, vpColVector &x, vpColVector &weights)
{
  double c = 1.2107; // 1.345;
  unsigned int n_data = x.getRows();

  for (unsigned int i = 0; i < n_data; i++) {
    // if(weights[i]!=0)
    if (std::fabs(weights[i]) > std::numeric_limits<double>::epsilon()) {
      double xi_sig = x[i] / sig;
      if (fabs(xi_sig) <= c)
        weights[i] = 1;
      else
        weights[i] = c / fabs(xi_sig);
    }
  }
}

/*!
  \brief calculation of Cauchy's influence function

  \param sigma : sigma parameters
  \param x : normalized residue vector
  \param weights : weight vector
*/

void vpRobust::psiCauchy(double sig, vpColVector &x, vpColVector &weights)
{
  unsigned int n_data = x.getRows();
  double const_sig = 2.3849 * sig;

  // Calculate Cauchy's equation
  for (unsigned int i = 0; i < n_data; i++) {
    weights[i] = 1 / (1 + vpMath::sqr(x[i] / (const_sig)));

    // If one coordinate is an outlier the other is too!
    // w[i] < 0.01 is a threshold to be set
    /*if(w[i] < 0.01)
      {
      if(i%2 == 0)
      {
      w[i+1] = w[i];
      i++;
      }
      else
      w[i-1] = w[i];
      }*/
  }
}

/*!
  \brief partition function
  \param a : vector to be sorted
  \param l : first value to be considered
  \param r : last value to be considered
*/
int vpRobust::partition(vpColVector &a, int l, int r)
{
  int i = l - 1;
  int j = r;
  double v = a[(unsigned int)r];

  for (;;) {
    while (a[(unsigned int)++i] < v)
      ;
    while (v < a[(unsigned int)--j])
      if (j == l)
        break;
    if (i >= j)
      break;
    exch(a[(unsigned int)i], a[(unsigned int)j]);
  }
  exch(a[(unsigned int)i], a[(unsigned int)r]);
  return i;
}

/*!
  \brief sort a part of a vector and select a value of this new vector
  \param a : vector to be sorted
  \param l : first value to be considered
  \param r : last value to be considered
  \param k : value to be selected
*/
double vpRobust::select(vpColVector &a, int l, int r, int k)
{
  while (r > l) {
    int i = partition(a, l, r);
    if (i >= k)
      r = i - 1;
    if (i <= k)
      l = i + 1;
  }
  return a[(unsigned int)k];
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
  } else {
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
  } else {
    double ap = a;
    double sum = 1.0 / a;
    double del = sum;
    for (int n = 1; n <= vpITMAX; n++) {
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
  for (int n = 1; n <= vpITMAX; n++) {
    double an = (double)n;
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
  static double cof[6] = {76.18009173, -86.50532033, 24.01409822, -1.231739516, 0.120858003e-2, -0.536382e-5};

  x = xx - 1.0;
  tmp = x + 5.5;
  tmp -= (x + 0.5) * log(tmp);
  ser = 1.0;
  for (int j = 0; j <= 5; j++) {
    x += 1.0;
    ser += cof[j] / x;
  }
  return -tmp + log(2.50662827465 * ser);
}
#endif

// double
// vpRobust::median(const vpColVector &v)
// {
//   int i,j;
//   int inf, sup;
//   int n = v.getRows() ;
//   vpColVector infsup(n) ;
//   vpColVector eq(n) ;
//
//   for (i=0;i<n;i++)
//   {
//     // We compute the number of elements superior to the current value
//     (sup)
//     // the number of elements inferior (inf) to the current value and
//     // the number of elements equal to the current value (eq)
//     inf = sup = 0;
//     for (j=0;j<n;j++)
//     {
//       if (i != j)
//       {
// 	if (v[i] <= v[j]) inf++;
// 	if (v[i] >= v[j]) sup++;
// 	if (v[i] == v[j]) eq[i]++;
//       }
//     }
//     // We compute then difference between inf and sup
//     // the median should be for |inf-sup| = 0 (1 if an even number of
//     element)
//     // which means that there are the same number of element in the array
//     // that are greater and smaller that this value.
//     infsup[i] = abs(inf-sup);
//   }
//
//   // seek for the smaller value of |inf-sup| (should be 0 or 1)
//   int imin = 0 ; // index of the median in the array
//   //double eqmax = 0 ; // count of equal values
//   // min cannot be greater than the number of element
//   double min = n;
//
//   // number of medians
//   int mediancount = 0;
//   // array of medians
//   int *medianindex = new int[n];
//
//   for (i=0; i<n; i++)
//   {
//     if(infsup[i] < min)
//     {
//       min = infsup[i];
//       imin = i ;
//
//       //reset count of median values
//       mediancount=0;
//       medianindex[mediancount]=i;
//     }
//     else if(infsup[i]==min) //If there is another median
//     {
//       mediancount++;
//       medianindex[mediancount]=i;
//     }
//   }
//
//   // Choose smalest data to be the median
//   /*for(i=0; i<mediancount+1; i++)
//     {
//     //Choose the value with the greatest count
//     if(eq[medianindex[i]] > eqmax)
//     {
//     eqmax = eq[medianindex[i]];
//     imin = medianindex[i];
//     }
//     //If we have identical counts
//     // Choose smalest data to be the median
//     //if(v[medianindex[i]] < v[imin])
//     //	imin = medianindex[i];
//     }*/
//
//   // return the median
//   delete []medianindex;
//   return(v[imin]);
// }
//
// // Calculate median only for the residues which have
// // not be rejected. i.e. weight=0
// double
// vpRobust::median(const vpColVector &v, vpColVector &weights)
// {
//   int i,j;
//   int inf, sup;
//   int n = v.getRows() ;
//   vpColVector infsup(n) ;
//   vpColVector eq(n) ;
//
//   for (i=0;i<n;i++)
//   {
//     if(weights[i]!=0)
//     {
//       // We compute the number of elements superior to the current value
//       (sup)
//       // the number of elements inferior (inf) to the current value and
//       // the number of elements equal to the current value (eq)
//       inf = sup = 0;
//       for (j=0;j<n;j++)
//       {
// 	if (weights[j]!=0 && i!=j)
// 	{
// 	  if (v[i] <= v[j]) inf++;
// 	  if (v[i] >= v[j]) sup++;
// 	  if (v[i] == v[j]) eq[i]++;
// 	}
//       }
//       // We compute then difference between inf and sup
//       // the median should be for |inf-sup| = 0 (1 if an even number of
//       element)
//       // which means that there are the same number of element in the array
//       // that are greater and smaller that this value.
//       infsup[i] = abs(inf-sup);
//     }
//   }
//
//   // seek for the smaller value of |inf-sup| (should be 0 or 1)
//   int imin = 0 ; // index of the median in the array
//   //double eqmax = 0 ; // count of equal values
//   // min cannot be greater than the number of element
//   double min = n;
//
//   for (i=0; i<n; i++)
//   {
//     if(weights[i]!=0)
//     {
//       if(infsup[i] < min)
//       {
// 	min = infsup[i];
// 	imin = i ;
//       }
//     }
//   }
//
//   // return the median
//   return(v[imin]);
// }

#undef vpITMAX
#undef vpEPS
#undef vpCST
