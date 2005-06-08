
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRotationVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpRotationVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpRotationVector.cpp,v 1.1.1.1 2005-06-08 07:08:06 fspindle Exp $
 *
 * Description
 * ============
 *    class that consider the case of a generic rotation vector
 *    (cannot be used as is !)
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <math.h>
#include <vpRotationVector.h>

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
  \brief cout a rotation vector
*/
ostream &operator <<(ostream &s,const vpRotationVector &m)
{
  int i;

  s.precision(5) ;
  s.width(6) ;
  s.precision(10) ;

  for (i=0;i<3;i++)       s <<  m.
			    r[i] << "  ";

  s << endl;

  return s;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
