/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Provide some simple operation on column vectors.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpColVector_H
#define vpColVector_H

#include <visp/vpMatrix.h>
#include <visp/vpRowVector.h>
#include <visp/vpMath.h>

class vpMatrix;
class vpRotationVector;

/*!
  \file vpColVector.h
  \brief definition of column vector class as well
  as a set of operations on these vector
*/

/*!
  \class vpColVector
  \ingroup Matrix
  \brief Class that provides a data structure for the column vectors
  as well as a set of operations on these vectors.

  the vpColVector is derived from vpMatrix.

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  \warning Note the vector in the class (*this) will be noted A in the comment
*/
class VISP_EXPORT vpColVector : public vpMatrix
{
   friend class vpMatrix;
protected:
  vpColVector (const vpMatrix &m, unsigned int j);
  vpColVector (const vpColVector &m, unsigned int r, unsigned int nrows) ;

public:

  //! Basic constructor.
  vpColVector() : vpMatrix() {};
  //! Constructor of vector of size n. Each element is set to 0.
  vpColVector(unsigned int n) : vpMatrix(n,1){};
  //! Constructor of vector of size n. Each element is set to \e val.
  vpColVector(unsigned int n, double val) : vpMatrix(n, 1, val){};
  //! Copy constructor.
  vpColVector (const vpColVector &v);
  //! Constructor that initialize a vpColVector from a vpRotationVector.
  vpColVector (const vpRotationVector &v);

  void insert(unsigned int i, const vpColVector &v);

  /*! Set the size of the column vector.
    \param i : Column vector size.
    \param flagNullify : If true, set the data to zero.
   */
  void resize(const unsigned int i, const bool flagNullify = true)
  {
    vpMatrix::resize(i, 1, flagNullify);
  }

  //! Access  V[i] = x
  inline double &operator [](unsigned int n) {  return *(data + n);  }
  //! Access x = V[i]
  inline const double &operator [](unsigned int n) const { return *(data+n);  }
  //! Copy operator.   Allow operation such as A = v
  vpColVector &operator=(const vpColVector &v);
  // Copy from a matrix.
  vpColVector &operator=(const vpMatrix &m);
  //! Initialize each element of the vector to x
  vpColVector &operator=(double x);
  // Copy operator.   Allow operation such as A << v
  vpColVector &operator<<(const vpColVector &v);
  // Assigment operator.   Allow operation such as A = *v
  vpColVector &operator<<(double *);

  //! Addition of two vectors V = A+v
  vpColVector operator+(const vpColVector &v) const;
  //! Substraction of two vectors V = A-v
  vpColVector operator-(const vpColVector &v) const;
  //! Operator A = -A
  vpColVector operator-() const;
  //! Dot product
  double  operator*(const vpColVector &x) const;
  //! Dot product
  vpMatrix  operator*(const vpRowVector &x) const;
  //! Multiplication by a scalar V =  A * x
  vpColVector operator*(const double x) const;

  vpColVector rows(unsigned int first_row, unsigned int last_row) const
  { 
    return vpColVector(*this, first_row-1, last_row-first_row+1);
  }

  //! Reshape methods
  void reshape(vpMatrix & m,
	       const unsigned int &nrows, const unsigned int &ncols);
  vpMatrix reshape(const unsigned int &nrows, const unsigned int &ncols);

  void stack(const double &b);
  void stack(const vpColVector &B);
  static vpColVector stack(const vpColVector &A, const vpColVector &B);
  static void stack(const vpColVector &A, const vpColVector &B, vpColVector &C);

  //! Transpose of a vector
  vpRowVector t() const;

  //! Normalise the vector
  vpColVector &normalize() ;
  // normalise the vector
  //  vpColVector &normalize(vpColVector &x) const ;

  //! Compute the cross product of two vectors C = a x b
  static vpColVector crossProd(const vpColVector &a, const vpColVector &b)  ;
  
  // Compute the cross product of two vectors C = a x b
  inline static vpColVector cross(const vpColVector &a, const vpColVector &b){
                                  return crossProd(a,b);}
  
  // Compute the skew matrix [v]x
  static vpMatrix skew(const vpColVector &v);
  //! Dot Product
  
  static double dotProd(const vpColVector &a, const vpColVector &b)  ;
 
  //! sort the elements of vector v
  static vpColVector  sort(const vpColVector &v)  ;
  //! reverse sorting of the elements of vector v
  static vpColVector invSort(const vpColVector &v)  ;
  //! compute the median
  static double median(const vpColVector &v) ;
  //! compute the mean
  static double mean(const vpColVector &v)  ;

  /*!

    Convert a column vector containing angles in radians into
    degrees.

  */
  inline void rad2deg() {
    double r2d = 180.0/M_PI;

    for (unsigned int i=0; i < rowNum; i++)
      (*this)[i] *= r2d;
  }
  /*!
    Convert a column vector containing angles in degrees into radians.

  */
  inline void deg2rad() {
    double d2r = M_PI/180.0;

    for (unsigned int i=0; i < rowNum; i++)
      (*this)[i] *= d2r;
  }

  /*!

    Return the size of the vector in term of number of rows.

  */
  inline unsigned int size() const
  {
    return getRows();
  }

};


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
