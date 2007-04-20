/****************************************************************************
 *
 * $Id: vpRotationVector.h,v 1.4 2007-04-20 14:22:16 asaunier Exp $
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
 * Generic rotation vector (cannot be used as is !).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpRotationVECTOR_H
#define vpRotationVECTOR_H

/*!
  \file vpRotationVector.h
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/


#include <stdio.h>
#include <iostream>


#include <math.h>

#include <visp/vpConfig.h>
#include <visp/vpMath.h>


//#include <vpRotationMatrix.h>

/*!
  \class vpRotationVector
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/

class VISP_EXPORT vpRotationVector
{
  friend class vpRotationMatrix;

protected:
  double r[3] ;


public:
  //! constructor
  vpRotationVector() { ; }


  //! constructor from 3 angles (in radian)
  vpRotationVector(const double phi, const double theta, const double psi) ;

  //! convert a rotation matrix into Ryp vector
  //  virtual vpRotationVector buildFrom(const vpRotationMatrix& R) =0 ;


  //! Access  r[i] = x
  inline double &operator [](int n) {  return *(r + n);  }
  //! Access x = r[i]
  inline const double &operator [](int n) const { return *(r+n);  }

 //---------------------------------
  // Printing
  //---------------------------------
  friend VISP_EXPORT std::ostream &operator << (std::ostream &s,const vpRotationVector &m);

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
