

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRansac.h
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpRansac.h,v 1.1 2005-06-28 08:37:14 marchand Exp $
 *
 * Description
 * ============
 *   template class for ransac robust algorithm
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file vpRansac.cpp

  template class for ransac robust algorithm
*/

#ifndef vpRANSAC_HH
#define vpRANSAC_HH

#include <visp/vpColVector.h>
#include <visp/vpMath.h>

/*!
  \class vpRansac

  \brief this class is a generic implementation of the Ransac algorithm.
  it cannot be used alone.

  Creation: june, 15 2005

  RANSAC is described in

  M.A. Fishler and  R.C. Boles. "Random sample concensus: A paradigm
  for model fitting with applications to image analysis and automated
  cartography". Comm. Assoc. Comp, Mach., Vol 24, No 6, pp 381-395, 1981

  Richard Hartley and Andrew Zisserman. "Multiple View Geometry in
  Computer Vision". pp 101-113. Cambridge University Press, 2001

  the code of this class is inspired by :
  Peter Kovesi
  School of Computer Science & Software Engineering
  The University of Western Australia
  pk at csse uwa edu au
  http://www.csse.uwa.edu.au/~pk


  \sa vpHomography


 */



template <class vpTransformation>
class vpRansac
{
public:
  static  void ransac(int n,
		      vpColVector &x,
		      int s, double t,
		      vpColVector &model,
		      vpColVector &inliers,
		      int consensus = 1000) ;
};


#include <visp/vpRansac.t.cpp>

#endif
