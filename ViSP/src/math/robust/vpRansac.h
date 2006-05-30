/****************************************************************************
 *
 * $Id: vpRansac.h,v 1.3 2006-05-30 08:40:43 fspindle Exp $
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
 * Ransac robust algorithm.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpRansac.cpp

  template class for
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
