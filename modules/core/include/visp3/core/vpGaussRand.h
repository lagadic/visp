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
 * Generation of random number with uniform and normal probability density.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpGaussRand_hh
#define vpGaussRand_hh

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpUniRand.h>

/*!
  \class vpGaussRand
  \ingroup group_core_random
  \brief Class for generating random number with normal probability density.

  The algorithms and notations used are described in \cite Gentle:2004.

  The code below shows how to use the random generator to get values that have
their mean equal to 10 with a standart deviation equal to 0.5.

  \code
#include <iostream>
#include <visp3/core/vpGaussRand.h>

int main()
{
  vpGaussRand noise(0.5, 10);
  for(int i=0; i< 10; i++) {
    std::cout << "noise " << i << ": " << noise() << std::endl;
  }
  return 0;
}
  \endcode

  The previous example produces the following printings:
\code
noise 0: 9.43873
noise 1: 10.1977
noise 2: 10.8145
noise 3: 9.13729
noise 4: 8.86476
noise 5: 9.83382
noise 6: 9.43609
noise 7: 9.34311
noise 8: 9.62742
noise 9: 9.37701
\endcode

  Note that the previous example produces always the same "random" results. To
produce real random values, you need to initialize the random generator with
different values using seed(). For example, this could be done using the
current time. The code becomes:

\code
#include <iostream>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpTime.h>

int main()
{
  vpGaussRand noise(0.5, 10);
  long seed = (long)vpTime::measureTimeMs();

  noise.seed(seed);
  for(int i=0; i< 10; i++) {
    std::cout << "noise " << i << ": " << noise() << std::endl;
  }
  return 0;
}
\endcode

  Now if you run the previous example you will always get different values:
  \code
noise 0: 10.5982
noise 1: 9.19111
noise 2: 9.82498
noise 3: 9.07857
noise 4: 9.9285
noise 5: 10.3688
noise 6: 9.75621
noise 7: 10.3259
noise 8: 10.4238
noise 9: 10.2391
  \endcode
 */
class VISP_EXPORT vpGaussRand : public vpUniRand
{
private:
  double mean;
  double sigma;
  double gaussianDraw();

public:
  /*!
      Default noise generator constructor.
     */
  vpGaussRand() : vpUniRand(), mean(0), sigma(0) {}

  /*!
      Gaussian noise random generator constructor.

      \param sigma_val : Standard deviation.
      \param mean_val : Mean value.
      \param noise_seed : Seed of the noise
    */
  vpGaussRand(const double sigma_val, const double mean_val, const long noise_seed = 0)
    : vpUniRand(noise_seed), mean(mean_val), sigma(sigma_val)
  {
  }

  /*!
      Set the standard deviation and mean for gaussian noise.

      \param sigma_val : New standard deviation sigma.
      \param mean_val : New mean value.
    */
  void setSigmaMean(const double sigma_val, const double mean_val)
  {
    this->mean = mean_val;
    this->sigma = sigma_val;
  }

  /*!
      Set the seed of the noise.

      \param seed_val : New seed.
    */
  void seed(const long seed_val) { x = seed_val; }

  /*!
      Return a random value from the Gaussian noise generator.
    */
  double operator()() { return sigma * gaussianDraw() + mean; }
};

#endif
