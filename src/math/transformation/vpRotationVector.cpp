/****************************************************************************
 *
 * $Id: vpRotationVector.cpp,v 1.4 2007-04-20 14:22:16 asaunier Exp $
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

#include <math.h>
#include <visp/vpRotationVector.h>

/*!
  \file vpRotationVector.cpp
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/


//! constructor from 3 angles (in radian)
vpRotationVector::vpRotationVector(const double phi,
			     const double theta,
			     const double psi)
{
    r[0] = phi ;
    r[1] = theta ;
    r[2] = psi ;
}


/*!
  \brief std::cout a rotation vector
*/
std::ostream &operator <<(std::ostream &s,const vpRotationVector &m)
{
  int i;

  s.precision(5) ;
  s.width(6) ;
  s.precision(10) ;

  for (i=0;i<3;i++)       s <<  m.
			    r[i] << "  ";

  s << std::endl;

  return s;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
