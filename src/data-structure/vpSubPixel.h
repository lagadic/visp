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
