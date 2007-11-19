/****************************************************************************
 *
 * $Id: vpColVector.cpp,v 1.10 2007-11-19 15:47:06 asaunier Exp $
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


/*!
  \file vpColVector.cpp
  \brief  Class that provides a data structure for the column vectors as well
  as a set of operations on these vectors
*/


#include <visp/vpColVector.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>



  //! operator addition of two vectors
vpColVector
vpColVector::operator+(const vpColVector &m) const
{
  vpColVector v ;

  try {
    v.resize(rowNum) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }




  double *vd = v.data ;   double *d = data ;  double *md = m.data ;

  for (int i=0;i<rowNum;i++)
      *(vd++) = *(d++) + *(md++);

  return v;
}

//! operator dot product
double
vpColVector::operator*(const vpColVector &m) const
{
  double v = 0 ;



  for (int i=0;i<rowNum;i++)
      v += (*this)[i] * m[i];
  return v;
}

//! operator dot product
vpMatrix vpColVector::operator*(const vpRowVector &m) const
{

  return (vpMatrix)*this*(vpMatrix)m;
}

  //! operator substraction of two vectors V = A-v
vpColVector vpColVector::operator-(const vpColVector &m) const
{
  vpColVector v ;

  try {
    v.resize(rowNum) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  for (int i=0;i<rowNum;i++)
      v[i] = (*this)[i] - m[i];
  return v;
}

vpColVector::vpColVector (vpColVector &m, int r, int nrows)
{
  if (r<0)
  {
    vpERROR_TRACE("\n\t\t Illegal subMatrix operation") ;
    throw(vpMatrixException(vpMatrixException::subMatrixError,
			    "\n\t\t Illegal subMatrix operation")) ;
  }
  if ( (r+nrows) > m.getRows() )
  {
    vpERROR_TRACE("\n\t\t SubvpMatrix larger than vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::subMatrixError,
			    "\n\t\t SubvpMatrix larger than vpColVector")) ;
  }
  init(m, r, 0, nrows, 1);
}


vpColVector::vpColVector (const vpRotationVector &v){
    resize(3);
    memcpy(data, v.r, 3*sizeof(double)) ;
}

  
 //! operator A = -A
vpColVector vpColVector::operator-()
{
  vpColVector A ;
  try {
    A.resize(rowNum)  ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }


  double *vd = A.data ;   double *d = data ;

  for (int i=0; i<rowNum; i++)
    *(vd++)= - (*d++);

  return A;
}

//! operator multiplication by a scalar V =  A * x
vpColVector vpColVector::operator*(double x) const
{
  vpColVector v;
  try {
    v.resize(rowNum)  ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }



  double *vd = v.data ;   double *d = data ;

  for (int i=0;i<rowNum;i++)
     *(vd++)=  (*d++) * x;
  return v;
}

/*!
  \brief copy from a matrix
  \warning  Handled with care m should be a 1 row matrix
*/
vpColVector &vpColVector::operator=(const vpMatrix &m)
{


  if (m.getCols() !=1)
  {
    vpTRACE(" m should be a 1 cols matrix ") ;
    throw (vpException(vpException::dimensionError)," m should be a 1 cols matrix ");
  }

  try {
    resize(m.getRows());
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

 //



  memcpy(data, m.data, rowNum*sizeof(double)) ;


  /*
    double *md = m.data ;   double *d = data ;
    for (int i=0;i<rowNum;i++)
    *(d++)=  *(md++) ;
    */
  /*
  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = m.rowPtrs[i][j];
    }
    }*/
  return *this;
}

 //! Copy operator.   Allow operation such as A = v
vpColVector &vpColVector::operator=(const vpColVector &v)
{

  int k = v.rowNum ;
  if (rowNum != k){
    try {
      resize(k);
    }
    catch(vpException me)
    {
      vpERROR_TRACE("Error caught") ;
      throw ;
    }
  }
  //

  memcpy(data, v.data, rowNum*sizeof(double)) ;
  /*
    double *vd = m.data ;   double *d = data ;
    for (int i=0;i<rowNum;i++)
    *(d++)=  *(vd++) ;


    for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
    rowPtrs[i][j] = v.rowPtrs[i][j];
    }
    }
*/
  return *this;
}

//! Copy operator.   Allow operation such as A = v
vpColVector & vpColVector::operator<<(const vpColVector &v)
{
  try {
    resize(v.getRows());
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = v.rowPtrs[i][j];
    }
  }
  return *this;
}

//! Assigment operator.   Allow operation such as A = *v
vpColVector & vpColVector::operator<<( double *x )
{


  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}

//! initialisation each element of the vector is x
vpColVector & vpColVector::operator=(double x)
{

  double *d = data ;

  for (int i=0;i<rowNum;i++)
    *(d++)=  x ;
  /*
  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = x;
    }
    }*/
  return *this;
}

/*
  \brief Transpose the row vector A

  A is defined inside the class

  \return  A^T
*/
vpRowVector vpColVector::t() const
{
  vpRowVector v ;
  try {
    v.resize(rowNum);
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  memcpy(v.data, data, rowNum*sizeof(double)) ;
  /*
    // premiere optimisation
    double *vd = m.data ;   double *d = data ;
    for (int i=0;i<rowNum;i++)
    *(d++)=  *(vd++) ;
    */

  /*
    // solution brute
    for (int i=0;i<rowNum;i++)
      v[i] = (*this)[i];
  */
  return v;
}
/*!
  \relates vpColVector
  \brief  multiplication by a scalar Ci = x*Bi
*/
vpColVector operator*(const double &x,const vpColVector &B)
{
  vpColVector v1 ;
  v1 = B*x ;
  return v1 ;
}

vpColVector::vpColVector (const vpColVector &v) : vpMatrix(v)
{
}

vpColVector::vpColVector (vpMatrix &m, int j) : vpMatrix(m, 0, j, m.getRows(), 1)
{
}


/*!
  \relates vpColVector
  \brief compute  the dot product of two vectors C = a.b
*/
double
vpColVector::dotProd(const vpColVector &a, const vpColVector &b)
{
  if (a.data==NULL)
  {
    vpERROR_TRACE("vpColVector a non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
  if (b.data==NULL)
  {
    vpERROR_TRACE("vpColVector b non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
  double *ad = a.data ;   double *bd = b.data ;

  double c = 0 ;
  for (int i=0 ; i < a.getRows() ; i++)
    c += *(ad++)* *(bd++) ;
  //  vpMatrix c = (a.t() * b);
  //  return c[0][0];
  return c ;
}

/*!
  \relates vpColVector
  \brief normalise the vector

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
// vpColVector &vpColVector::normalize(vpColVector &x) const
// {
//   x = x/sqrt(x.sumSquare());

//   return x;
// }


/*!
  \relates vpColVector
  \brief normalise the vector

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
vpColVector &vpColVector::normalize()
{

  double sum = sumSquare() ;

  if (sum != 0.0)
    *this /= sqrt(sum) ;

  // If sum = 0, we have a nul vector. So we return just.
  return *this;
}



vpColVector
vpColVector::invSort(const vpColVector &v)
{
 if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
  vpColVector tab ;
  tab = v ;
  int nb_permutation = 1 ;
  int i = 0 ;
  while (nb_permutation !=0 )
  {
    nb_permutation = 0 ;
    for (int j =v.getRows()-1 ; j >= i+1 ; j--)
    {
      if ((tab[j]>tab[j-1]))
      {
	double tmp = tab[j] ;
	tab[j] = tab[j-1] ;
	tab[j-1] = tmp ;
	nb_permutation++ ;
      }
    }
    i++ ;
  }

  return tab ;
}

vpColVector
vpColVector::sort(const vpColVector &v)
{
 if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector a non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
 vpColVector tab ;
  tab = v ;
  int nb_permutation = 1 ;
  int i = 0 ;
  while (nb_permutation !=0 )
  {
    nb_permutation = 0 ;
    for (int j =v.getRows()-1 ; j >= i+1 ; j--)
    {
      if ((tab[j]<tab[j-1]))
      {
	double tmp = tab[j] ;
	tab[j] = tab[j-1] ;
	tab[j-1] = tmp ;
	nb_permutation++ ;
      }
    }
    i++ ;
  }

  return tab ;
}

/*!
  \brief Compute the mean value of all the element of the vector
*/
double vpColVector::mean(const vpColVector &v)
{
 if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
 double mean = 0 ;
  double *vd = v.data ;
  for (int i=0 ; i < v.getRows() ; i++)
    mean += *(vd++) ;

  /*
  for (int i=0 ; i < v.getRows() ; i++)
  {
    mean += v[i] ;
  }
   mean /= v.rowNum ;
   return mean;
  */
  return mean/v.getRows();
}

/*!
  Compute the median value of all the element of the vector
*/
double
vpColVector::median(const vpColVector &v)
{
  if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }

  int i,j;
  int inf, sup;
  int n = v.getRows() ;
  vpColVector infsup(n) ;

  for (i=0;i<v.getRows();i++)
  {
    // We compute the number of element gretear (sup) than the current
    // value and the number of element smaller (inf) than the current
    // value
    inf = sup = 0;
    for (j=0;j<v.getRows();j++)
    {
      if (i != j)
      {
        if (v[i] <= v[j]) inf++;
        if (v[i] >= v[j]) sup++;
      }
    }
    // We compute then difference between inf and sup
    // the median should be for |inf-sup| = 0 (1 if an even number of element)
    // which means that there are the same number of element in the array
    // that are greater and smaller that this value.
    infsup[i] = abs(inf-sup);
  }

  // seek for the smaller value of |inf-sup| (should be 0 or 1)
  int imin=0 ; // index of the median in the array
  // min cannot be greater than the number of element
  double  min = v.getRows();
  for (i=0;i<v.getRows();i++)
    if (infsup[i] < min)
    {
      min = infsup[i];
      imin = i ;
    }

  // return the median
  return(v[imin]);

}

/*!
  \brief Compute the skew symmetric matrix of vector v (matrice de pre-produit
  vectoriel)

  \f[ \mbox{if} \quad  {\bf V} =  \left( \begin{array}{c} x \\ y \\  z
  \end{array}\right), \quad \mbox{then} \qquad
  skew(\bf V) = \left( \begin{array}{ccc}
  0 & -z & y \\
  z & 0 & -x \\
  -y & x & 0
  \end{array}\right)
  \f]

  \param v : vpColVector
*/
vpMatrix
vpColVector::skew(const vpColVector &v)
{

  vpMatrix M ;
  if (v.getRows() != 3)
  {
    vpERROR_TRACE("Cannot compute skew kinematic matrix,"
		"v has not 3 rows") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t Cannot compute skew kinematic matrix"
			    "v has not 3 rows")) ;
  }


  M.resize(3,3) ;
  M[0][0] = 0 ;     M[0][1] = -v[2] ;  M[0][2] = v[1] ;
  M[1][0] = v[2] ;  M[1][1] = 0 ;      M[1][2] = -v[0] ;
  M[2][0] = -v[1] ; M[2][1] = v[0] ;   M[2][2] = 0 ;


  return M ;
}

/*!
  \brief Compute the cross product of two vectors C = a x b

  \param a : vpColVector
  \param b : vpColVector
*/
vpColVector vpColVector::crossProd(const vpColVector &a, const vpColVector &b)
{

  if (a.getRows() != 3)
  {
    vpERROR_TRACE("Cannot compute cross product,"
		"a has not 3 rows") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t Cannot compute  cross product"
			    "a has not 3 rows")) ;
  }
  if (b.getRows() != 3)
  {

    vpERROR_TRACE("Cannot compute cross product,"
		"b has not 3 rows") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t Cannot compute  cross product"
			    "b has not 3 rows")) ;;
  }

  return vpColVector::skew(a) * b;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
