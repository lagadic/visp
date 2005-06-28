
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpNoise.h
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpNoise.h,v 1.1 2005-06-28 08:35:43 marchand Exp $
 *
 * Description
 * ============
 *    Class for generating random number
  with uniform and normal probability density
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


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


/*!
  \brief CUniRand is a class for generating random number
  with uniform probability density


  The algorithms and notations used are described in
  Random Number Generation and Monte Carlo Methods
  James E. Gentle

  Springer 1998

 */
class vpUniRand
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
  \brief   vpGaussrand is a class for generating random number
  with normal probability density

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
