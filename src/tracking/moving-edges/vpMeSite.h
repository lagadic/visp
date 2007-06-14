/****************************************************************************
 *
 * $Id: vpMeSite.h,v 1.8 2007-06-14 12:04:22 marchand Exp $
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
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 * Andrew Comport
 *
 *****************************************************************************/

/*!
	\file vpMeSite.h
	\brief Moving edges
*/



#ifndef vpMeSite_H
#define vpMeSite_H

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpMe.h>

// ====================================================================
/*!
 * \class vpMeSite
 * \brief Performs search in a given direction(normal) for a given distance(pixels)
 * \n for a given 'site'. Gives the most likely site given the probablility
 * \n from an ME mask
 * \n
 * \n - Bug fix rewrote application of masks to use the temporal information
 * \n instead of applying both temporal masks to the same image.
 * \n ie: spacial -> spatio/temporal
 * \n - Added new tracking function to choose the most similar edge amongst
 * \n - all edges found.
 * \n sample step.
 */
// ====================================================================


class VISP_EXPORT vpMeSite
{
public:
  int i,j ;
  int i_1, j_1 ;
  double ifloat, jfloat ;
  unsigned char v ;
  int mask_sign ;
public:
  // Angle of tangent at site
  double alpha;

  // Convolution of Site in previous image
  double convlt ;
 // Convolution of Site in previous image
  double normGradient ;

  //! Flag to indicate whether point is rejected or not
  //! 1 = contrast, 2 = threshold, 3 = M-estimator, 0 = nosupp
  int suppress;

  // Uncertainty of point given as a probability between 0 and 1
  double weight;

public:
  void init() ;
  void init(double ip, double jp, double alphap) ;
  void init(double ip, double jp, double alphap, double convltp) ;
  void init(double ip, double jp, double alphap, double convltp, int sign) ;

  vpMeSite () ;
  vpMeSite(double ip, double jp) ;
  virtual ~vpMeSite() {} ;

  vpMeSite &operator=(const vpMeSite &m) ;
  int operator!=(const vpMeSite  &m) ;
  
  // Needed in order to use it in vpList
  friend std::ostream& operator<<(std::ostream& os, vpMeSite& vpMeS);

  void getSign(vpImage<unsigned char> &I, const int range) ;
  double convolution(vpImage<unsigned char>& ima, const vpMe *me) ;

  vpMeSite *getQueryList(vpImage<unsigned char> &I, const int range) ;

  void track(vpImage<unsigned char>& im,
	     const vpMe *me,
	     const  bool test_contraste=true);

private:
  int selectDisplay ;
  enum displayEnum
    {
      NONE,
      RANGE,
      RESULT,
      RANGE_RESULT
    } ;
public:
  void setDisplay(int select) { selectDisplay = select ; }
} ;

#endif
