
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpColVector.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpColVector.h,v 1.2 2005-08-26 08:33:33 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a column vector
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpColVector_H
#define vpColVector_H


#include <visp/vpMatrix.h>
#include <visp/vpRowVector.h>

/*!
  \file vpColVector.h
  \brief definition of column vector class as well
  as a set of operations on these vector
*/

/*!
  \class vpColVector

  \brief
  Class that provides a data structure for the column vectors as well
  as a set of operations on these vectors.

  the vpColVector is derived from vpMatrix.

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  \warning Note the vector in the class (*this) will be noted A in the comment
*/
class vpColVector : public vpMatrix
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

  //! Set the size of the Column vector
  void resize(int i) {  vpMatrix::resize(i, 1); }


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
  static vpColVector cross(const vpColVector &a, const vpColVector &b)  ;
  //! compute the skew matrix [a]x
  static vpMatrix skew(const vpColVector &a)  ;

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
};



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
