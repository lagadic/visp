/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Operation on row vectors.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/




#ifndef vpRowVector_H
#define vpRowVector_H


#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>

class vpMatrix;

/*!
  \file vpRowVector.h
  \brief definition of row vector class as well
  as a set of operations on these vector
*/

/*!
  \class vpRowVector

  \ingroup Matrix
  \brief Definition of the row vector class.

  vpRowVector class provides a data structure for the row vectors as well
  as a set of operations on these vectors

  \author Eric Marchand (IRISA - INRIA Rennes)

  \warning Note the vector in the class (*this) will be noted A in the comment

  \ingroup libmath
*/
class VISP_EXPORT vpRowVector : public vpMatrix
{
  friend class vpMatrix;


protected:
  //! Constructor  (Take line i of matrix m)
  vpRowVector(vpMatrix &m, int i);

public:
  //! basic constructor
  vpRowVector() : vpMatrix() {};
  //! constructor of vector of size n
  vpRowVector(int nn) : vpMatrix(1,nn){};
  //! copy constructor
  vpRowVector(const vpRowVector &v);

  //! Set the size of the Row vector
  inline void resize(int i)      {   vpMatrix::resize(1, i) ;  }
  //! Access  V[i] = x
  inline double &operator [](int n)             { return *(data+n); }
  //! Access x = V[i]
  inline const double &operator [](int n) const { return *(data+n) ; }

  //! Copy operator.   Allow operation such as A = v
  vpRowVector &operator=(const vpRowVector &v);
  //! copy from a matrix
  vpRowVector & operator=(const vpMatrix &m) ;

  //!operator dot product
  double  operator*(const vpColVector &x) const;
  //!operator dot product
  vpRowVector operator*(const vpMatrix &A) const;
  
  //! initialisation each element of the vector is x
  vpRowVector& operator=(const double x);

  //! Transpose the vector
  vpColVector t() const;

  //! normalise the vector
  vpRowVector &normalize() ;
  //! normalise the vector
  vpRowVector &normalize(vpRowVector &x) const ;


};


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
