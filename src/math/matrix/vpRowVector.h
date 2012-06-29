/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Operation on row vectors.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/




#ifndef vpRowVector_H
#define vpRowVector_H

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
  vpRowVector(vpMatrix &m, unsigned int i);

public:
  //! basic constructor
  vpRowVector() : vpMatrix() {};
  //! constructor of vector of size n
  vpRowVector(unsigned int nn) : vpMatrix(1,nn){};
  //! copy constructor
  vpRowVector(const vpRowVector &v);

  //! Set the size of the Row vector
  inline void resize(unsigned int i)      {   vpMatrix::resize(1, i) ;  }
  //! Access  V[i] = x
  inline double &operator [](unsigned int n)             { return *(data+n); }
  //! Access x = V[i]
  inline const double &operator [](unsigned int n) const { return *(data+n) ; }

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

  //! Reshape methods
  void reshape(vpMatrix & m,const unsigned int &nrows,const unsigned int &ncols);
  vpMatrix reshape(const unsigned int &nrows,const unsigned int &ncols);
  
  //! Transpose the vector
  vpColVector t() const;

  //! normalise the vector
  vpRowVector &normalize() ;
  //! normalise the vector
  vpRowVector &normalize(vpRowVector &x) const ;

  /*!

    Return the size of the vector in term of number of columns.

  */
  inline unsigned int size() const
  {
    return getCols();
  }

};


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
