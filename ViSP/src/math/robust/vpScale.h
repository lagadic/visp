/*!
 \file vpScale.h
*/

// =========================================================
/*!
 * \brief Contains various estimators for scale.
 * \n Methods : Median Absolute Deviation (MAD),
 * 							MPDE, Mean shift kernel density estimation.
 * \author Andrew Comport
 * \date 24/10/03
 */
// ==========================================================


#ifndef CSCALE_HH
#define CSCALE_HH

#define EPANECHNIKOV 0

#include <math.h>
#include <visp/vpColVector.h>

class vpScale
{

private:
  double bandwidth;
  int dimension;
  int kernel_type;

public:

  //! Constructor
  vpScale();
  vpScale(double, int, int);
  //! Destructor
  ~vpScale(void);

  double MeanShift(vpColVector &error);
  double KernelDensity(vpColVector &error, int position);
  double KernelDensityGradient(vpColVector &error, int position);

  double KernelDensity_EPANECHNIKOV(vpColVector &X);
  double KernelDensityGradient_EPANECHNIKOV(double X, int n);

};

#endif
