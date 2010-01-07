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
 * Provide some simple operation on column vectors.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



#ifndef vpColVector_H
#define vpColVector_H

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpRowVector.h>
#include <visp/vpRotationVector.h>

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
  vpColVector (vpMatrix &m, int j);
  vpColVector (vpColVector &m, int r, int nrows) ;

public:

  //! basic constructor
  vpColVector() : vpMatrix() {};
  //! constructor of vector of size n
  vpColVector(int nn) : vpMatrix(nn,1){};
  //! copy constructor
  vpColVector (const vpColVector &v);
  //! constructor initialize a vpColVector from a vpRotationVector
  vpColVector (const vpRotationVector &v);

  /*! Set the size of the column vector.
    \param i : Column vector size.
    \param flagNullify : If true, set the data to zero.
   */
  void resize(const int i, const bool flagNullify = true)
  {  vpMatrix::resize(i, 1, flagNullify); }


  //! Access  V[i] = x
  inline double &operator [](int n) {  return *(data + n);  }
  //! Access x = V[i]
  inline const double &operator [](int n) const { return *(data+n);  }
  //! Copy operator.   Allow operation such as A = v
  vpColVector &operator=(const vpColVector &v);
  //! Copy operator.   Allow operation such as A = v
  vpColVector &operator<<(const vpColVector &v);
  //! Assigment operator.   Allow operation such as A = *v
  vpColVector &operator<<(double *);
  //! copy from a matrix
  vpColVector &operator=(const vpMatrix &m);
  //! initialisation each element of the vector is x
  vpColVector &operator=(double x);

  //! operator addition of two vectors V = A+v
  vpColVector operator+(const vpColVector &v) const;
  //! operator substraction of two vectors V = A-v
  vpColVector operator-(const vpColVector &v) const;
  //! operator dot product
  double  operator*(const vpColVector &x) const;
  //! operator dot product
  vpMatrix  operator*(const vpRowVector &x) const;
  //! operator multiplication by a scalar V =  A * x
  vpColVector operator*(const double x) const;
  //! operator A = -A
  vpColVector operator-() ;

  vpColVector rows(int first_row, int last_row)
  { return vpColVector(*this, first_row-1, last_row-first_row+1); }

  //! transpose of Vector
  vpRowVector t() const;

  //! normalise the vector
  vpColVector &normalize() ;
  //! normalise the vector
  //  vpColVector &normalize(vpColVector &x) const ;

  //! compute the cross product of two vectors C = a x b
  static vpColVector crossProd(const vpColVector &a, const vpColVector &b)  ;
  
  // compute the cross product of two vectors C = a x b
  inline static vpColVector cross(const vpColVector &a, const vpColVector &b){
                                  return crossProd(a,b);}
  
  //! compute the skew matrix [a]x
  static vpMatrix skew(const vpColVector &a);
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
    double rad2deg = 180.0/M_PI;

    for (int i=0; i < rowNum; i++)
      (*this)[i] *= rad2deg;
  }
  /*!
    Convert a column vector containing angles in degrees into radians.

  */
  inline void deg2rad() {
    double deg2rad = M_PI/180.0;

    for (int i=0; i < rowNum; i++)
      (*this)[i] *= deg2rad;
  }
};


#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
