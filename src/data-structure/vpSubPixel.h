/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Sub pixel manipulation.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpSubPixel_h
#define vpSubPixel_h

#include "visp/vpConfig.h"

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

/*!
  \file vpSubPixel.h
  \brief Class that defines what is a sub-pixel.

*/


/*!
  \class vpSubPixel
  \brief Class that defines what is a sub-pixel.

  \deprecated This class is deprecated. You should use vpImagePoint instead.

  A sub-pixel is considered here as an image pixel with non-integer
  (u,v) coordinates.

  - Coordinate u stands for a position along the image horizontal axis.
  - Coordinate v stands for a position along the image vertical axis.

*/
class VISP_EXPORT vpSubPixel
{

 public:
  vp_deprecated vpSubPixel();
  vp_deprecated vpSubPixel(const double &u, const double &v);
  vp_deprecated vpSubPixel(const vpSubPixel &p);
  virtual ~vpSubPixel() {}

  /*!

    Copy operator.

   */
  vp_deprecated const vpSubPixel& operator=(const vpSubPixel &p) {
    this->u = p.u;
    this->v = p.v;
    return *this;
  }

  /*!
    Set the sub-pixel coordinate along the image horizontal axis.

    \sa set_v(const double &)
  */
  vp_deprecated inline void set_u(const double &u) {this->u = u;}
  /*!
    Set the sub-pixel coordinate along the image vertical axis.

    \sa set_u(const double &)
  */
  vp_deprecated inline void set_v(const double &v) {this->v = v;}
  
  /*!
    Get the sub-pixel coordinate along the image horizontal axis.

    \sa get_v()
  */
  vp_deprecated inline double get_u() const {return u;}
  /*!
    Get the sub-pixel coordinate along the image vertival axis.

    \sa get_u()
  */
  vp_deprecated inline double get_v() const {return v;}
  
  // Printing
  vp_deprecated friend VISP_EXPORT std::ostream &operator << (std::ostream &s,
						const vpSubPixel &p);

 private:
  double u; // Sub pixel coordinate along the horizontal axis
  double v; // Sub pixel coordinate along the vertical axis
  
} ;

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
