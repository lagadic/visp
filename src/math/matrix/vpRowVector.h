
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRowVector.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpRowVector.h,v 1.2 2005-11-16 09:44:07 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a row vector
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



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

  \brief Definition of the vpRowVector class.

  vpRowVector class provides a data structure for the row vectors as well
  as a set of operations on these vectors

  \author Eric Marchand (IRISA - INRIA Rennes)

  \warning Note the vector in the class (*this) will be noted A in the comment

  \ingroup libmath
*/
class vpRowVector : public vpMatrix
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

  //! initialisation each element of the vector is x
  vpRowVector &operator=(const double x);

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
