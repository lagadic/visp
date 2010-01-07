/****************************************************************************
 *
 * $Id$
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
 * This file is part of the ViSP toolkit
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
 * Generation of random number with uniform and normal probability density.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#ifndef vpNoise_hh
#define vpNoise_hh


/*!
  \file vpNoise.h
  \brief Class for generating random number
  with uniform and normal probability density

  The algorithms and notations used are described in

  James E. Gentle, Random Number Generation and Monte Carlo Methods,
  Springer 1998
*/

#include <visp/vpConfig.h>


/*!
  \class vpUniRand

  \ingroup Random
  \brief Class for generating random numbers with uniform probability density.

  The algorithms and notations used are described in
  Random Number Generation and Monte Carlo Methods
  James E. Gentle

  Springer 1998

 */
class VISP_EXPORT vpUniRand
{


  unsigned long    a  ;
  unsigned long    m ; //2^31-1
  unsigned long    q ; //integer part of m/a
  unsigned long    r ;//r=m mod a
  double    normalizer ; //we use a normalizer > m to ensure ans will never be 1 (it is the case if x = 739806647)


private:
  void draw0();
protected:
  long x;
  double draw1();
  void init()
  {
    a = 16807 ;
    m = (unsigned long)2147483647 ; //2^31-1
    q = 127773 ; //integer part of m/a
    r = 2836 ;//r=m mod a
    //we use a normalizer > m to ensure ans will never be
    // 1 (it is the case if x = 739806647)
    normalizer = 2147484721.0 ;
  }

public:
  vpUniRand(const long seed = 0):x((seed)? seed : 739806647)
  {
    init() ;
  }
  double operator()() {return draw1();}

};

/*!
  \class vpGaussRand
  \ingroup Random
  \brief Class for generating random number with normal probability density.

  The algorithms and notations used are described in

  Random Number Generation and Monte Carlo Methods
  James E. Gentle, Springer 1998
*/
class vpGaussRand : public vpUniRand
{
  double mean;
  double sigma;

public:

  // Initialiazation
  vpGaussRand(const double sqrtvariance,
	     const double _mean,
	     const long seed = 0):mean(_mean), sigma(sqrtvariance)
  {
    init() ;
    mean = 0 ;
    if (seed) x=seed; else x=739806647;
  }
  double operator()() {return sigma*gaussianDraw()+mean;}

private :
  double gaussianDraw();
};

#endif
