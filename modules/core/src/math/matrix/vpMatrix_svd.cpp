/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Matrix SVD decomposition.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpDebug.h>


#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#include <iostream>

/*---------------------------------------------------------------------

SVD related functions

---------------------------------------------------------------------*/


static double pythag(double a, double b)
{
  double absa, absb;
  absa = fabs(a);
  absb = fabs(b);
  if (absa > absb) return absa*sqrt(1.0+vpMath::sqr(absb/absa));
  //else return (absb == 0.0 ? 0.0 : absb*sqrt(1.0+vpMath::sqr(absa/absb)));
  else return (std::fabs(absb) <= std::numeric_limits<double>::epsilon() ? 0.0 : absb*sqrt(1.0+vpMath::sqr(absa/absb)));
}

#ifdef SIGN
#undef SIGN
#endif
#define SIGN(a,b) ((b) >= 0.0 ? fabs(a) : -fabs(a))

/*!
  \brief Singular value decomposition

  Given a matrix A (m x n) this routine compute its sngular value decomposition
  A = U W V^T. The matrice U replace A on output. the diagonal matrix of
  singular value is output as a vector W (n).  The matrix V (not the transpose
  V^T) is output as V (n x n)


  \warning Destructive wrt A
  \warning


  \sa SVD for a more intuitive use


  This function is extracted from the NRC

*/

#define  MAX_ITER_SVD 50
void vpMatrix::svdNr(vpColVector& W, vpMatrix& V)
{

  unsigned int m = rowNum;
  unsigned int n = colNum;
  double epsilon = 10*std::numeric_limits<double>::epsilon();

  unsigned int flag,i,its,j,jj,k,l=0,nm=0;
  double c,f,h,s,x,y,z;
  double anorm=0.0,g=0.0,scale=0.0;

  // So that the original NRC code (using 1..n indexing) can be used
  // This should be considered as a temporary fix.
  double **a = new double*[m+1];
  double **v = new double*[n+1];
  //  double **w = W.rowPtrs;
  //  w--;

  double *w = new double[n+1];
  for (i=0;i<n;i++) w[i+1] = 0.0;

  for (i=1;i<=m;i++) {
    a[i] = this->rowPtrs[i-1]-1;
  }
  for (i=1;i<=n;i++) {
    v[i] = V.rowPtrs[i-1]-1;
  }

  if (m < n)
  {
    delete[] w;
    delete[] a;
    delete[] v;
    vpERROR_TRACE("\n\t\tSVDcmp: You must augment A with extra zero rows") ;
    throw(vpMatrixException(vpMatrixException::matrixError,
			    "\n\t\tSVDcmp: You must augment A with "
			    "extra zero rows")) ;
  }
  double* rv1=new double[n+1];

  for (i=1;i<=n;i++) {
    l=i+1;
    rv1[i]=scale*g;
    g=s=scale=0.0;
    if (i <= m) {
      for (k=i;k<=m;k++) scale += fabs(a[k][i]);
      //if ((scale != 0.0) || (fabs(scale) > EPS_SVD)) {
      if ((std::fabs(scale) > epsilon)/* || (fabs(scale) > EPS_SVD)*/) {
	for (k=i;k<=m;k++) {
	  a[k][i] /= scale;
	  s += a[k][i]*a[k][i];
	}
	f=a[i][i];
	g = -SIGN(sqrt(s),f);
	h=f*g-s;
	a[i][i]=f-g;
	if (i != n) {
	  for (j=l;j<=n;j++) {
	    for (s=0.0,k=i;k<=m;k++) s += a[k][i]*a[k][j];
	    f=s/h;
	    for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
	  }
	}
	for (k=i;k<=m;k++) a[k][i] *= scale;
      }
    }
    w[i]=scale*g;
    g=s=scale=0.0;
    if (i <= m && i != n) {
      for (k=l;k<=n;k++) scale += fabs(a[i][k]);
      //if ((scale != 0.0) || (fabs(scale) > EPS_SVD)) {
      if ((std::fabs(scale) > epsilon) /*|| (fabs(scale) > EPS_SVD)*/) {
	for (k=l;k<=n;k++) {
	  a[i][k] /= scale;
	  s += a[i][k]*a[i][k];
	}
	f=a[i][l];
	g = -SIGN(sqrt(s),f);
	h=f*g-s;
	a[i][l]=f-g;
	for (k=l;k<=n;k++) rv1[k]=a[i][k]/h;
	if (i != m) {
	  for (j=l;j<=m;j++) {
	    for (s=0.0,k=l;k<=n;k++) s += a[j][k]*a[i][k];
	    for (k=l;k<=n;k++) a[j][k] += s*rv1[k];
	  }
	}
	for (k=l;k<=n;k++) a[i][k] *= scale;
      }
    }
    anorm=vpMath::maximum(anorm,(fabs(w[i])+fabs(rv1[i])));
  }
  for (i=n;i>=1;i--) {
    if (i < n) {
      //if ((g) || (fabs(g) > EPS_SVD)) {
      if ((std::fabs(g) > epsilon) /*|| (fabs(g) > EPS_SVD)*/) {
	for (j=l;j<=n;j++)
	  v[j][i]=(a[i][j]/a[i][l])/g;
	for (j=l;j<=n;j++) {
	  for (s=0.0,k=l;k<=n;k++) s += a[i][k]*v[k][j];
	  for (k=l;k<=n;k++) v[k][j] += s*v[k][i];
	}
      }
      for (j=l;j<=n;j++) v[i][j]=v[j][i]=0.0;
    }
    v[i][i]=1.0;
    g=rv1[i];
    l=i;
  }
  for (i=n;i>=1;i--) {
    l=i+1;
    g=w[i];
    if (i < n)
      for (j=l;j<=n;j++) a[i][j]=0.0;
    //if ((g != 0.0) || (fabs(g) > EPS_SVD)) {
    if ((std::fabs(g) > epsilon) /*|| (fabs(g) > EPS_SVD)*/) {
      g=1.0/g;
      if (i != n) {
	for (j=l;j<=n;j++) {
	  for (s=0.0,k=l;k<=m;k++) s += a[k][i]*a[k][j];
	  f=(s/a[i][i])*g;
	  for (k=i;k<=m;k++) a[k][j] += f*a[k][i];
	}
      }
      for (j=i;j<=m;j++) a[j][i] *= g;
    } else {
      for (j=i;j<=m;j++) a[j][i]=0.0;
    }
    ++a[i][i];
  }
  for (k=n;k>=1;k--) {
    for (its=1;its<=MAX_ITER_SVD;its++) {
      flag=1;
      for (l=k;l>=1;l--) {
	nm=l-1;
	//if ((fabs(rv1[l])+anorm == anorm) || (fabs(rv1[l]) <= EPS_SVD)) {
        if ((std::fabs(rv1[l]) <= epsilon) /*|| (fabs(rv1[l]) <= EPS_SVD)*/) {
	  flag=0;
	  break;
	}
	//if ((fabs(w[nm])+anorm == anorm) || (fabs(w[nm]) <= EPS_SVD)) break;
        if ((std::fabs(w[nm]) <= epsilon) /*|| (fabs(w[nm]) <= EPS_SVD)*/) break;
      }
      if (flag) {
	c=0.0;
	s=1.0;
	for (i=l;i<=k;i++) {
	  f=s*rv1[i];
	  //if ((fabs(f)+anorm != anorm)  || (fabs(f) <= EPS_SVD)) {
          if ((std::fabs(f) > epsilon)  /*|| (fabs(f) <= EPS_SVD)*/) {
	    g=w[i];
	    h=pythag(f,g);
	    w[i]=h;
	    h=1.0/h;
	    c=g*h;
	    s=(-f*h);
	    for (j=1;j<=m;j++) {
	      y=a[j][nm];
	      z=a[j][i];
	      a[j][nm]=y*c+z*s;
	      a[j][i]=z*c-y*s;
	    }
	  }
	}
      }
      z=w[k];
      if (l == k) {
	if (z < 0.0) {
	  w[k] = -z;
	  for (j=1;j<=n;j++) v[j][k]=(-v[j][k]);
	}
	break;
      }
      if (its == MAX_ITER_SVD)
      {
	for (i=0;i<n;i++) W[i] = w[i+1];

	vpERROR_TRACE("\n\t\t No convergence in  SVDcmp ") ;
	std::cout << *this <<std::endl ;
	//	throw(vpMatrixException(vpMatrixException::matrixError,
	//				"\n\t\t No convergence in  SVDcmp ")) ;
      }
      x=w[l];
      nm=k-1;
      y=w[nm];
      g=rv1[nm];
      h=rv1[k];
      f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
      g=pythag(f,1.0);
      f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
      c=s=1.0;
      for (j=l;j<=nm;j++) {
	i=j+1;
	g=rv1[i];
	y=w[i];
	h=s*g;
	g=c*g;
	z=pythag(f,h);
	rv1[j]=z;
  if ((std::fabs(z) > epsilon) /*|| (fabs(z) > EPS_SVD)*/) {
    c=f/z;
    s=h/z;
  }
	f=x*c+g*s;
	g=g*c-x*s;
	h=y*s;
	y=y*c;
	for (jj=1;jj<=n;jj++) {
	  x=v[jj][j];
	  z=v[jj][i];
	  v[jj][j]=x*c+z*s;
	  v[jj][i]=z*c-x*s;
	}
	z=pythag(f,h);
	w[j]=z;
	//if ((z != 0.0) || (fabs(z) > EPS_SVD)) {
        if ((std::fabs(z) > epsilon) /*|| (fabs(z) > EPS_SVD)*/) {
	  z=1.0/z;
	  c=f*z;
	  s=h*z;
	}
	f=(c*g)+(s*y);
	x=(c*y)-(s*g);
	for (jj=1;jj<=m;jj++) {
	  y=a[jj][j];
	  z=a[jj][i];
	  a[jj][j]=y*c+z*s;
	  a[jj][i]=z*c-y*s;
	}
      }
      rv1[l]=0.0;
      rv1[k]=f;
      w[k]=x;
    }
  }
  for (i=0;i<n;i++) W[i] = w[i+1];


  delete[] w;
  delete[] rv1;
  delete[] a;
  delete[] v;

}

#undef SIGN
#undef PYTHAG

/*!
  \brief solve a linear system AX = B using an SVD decomposition

  Solves AX = B for a vector X, where A is am matrix m x n, w a vector (n) and
  v a matrix (n x n) as returned by SVDcmp.  m and n are the dimensions of A,
  and will be equal for square matrices. b (m) is the input right-hand
  side. x (n) is the output solution vector. No input quantities are
  destroyed, so the routine may be called sequentially with different b's.

  \warning not to be used directly

  \sa to be used with svd first

  \sa solveBySVD and  SVDsolve for a more intuitive solution of AX=B problem
*/


void vpMatrix::SVBksb( const vpColVector& w,
		       const vpMatrix& v,
		       const vpColVector& b, vpColVector& x)
{
  unsigned int m = this->rowNum;
  unsigned int n = this->colNum;
  double** u = rowPtrs;

  unsigned int jj,j,i;
  double s,*tmp;

  tmp=new double[n];
  for (j=0;j<n;j++) {
    s=0.0;
    //if (w[j])
    if (std::fabs(w[j]) > std::numeric_limits<double>::epsilon())
    {
      for (i=0;i<m;i++) s += u[i][j]*b[i];
      s /= w[j];
    }
    tmp[j]=s;
  }
  for (j=0;j<n;j++) {
    s=0.0;
    for (jj=0;jj<n;jj++) s += v[j][jj]*tmp[jj];
    x[j]=s;
  }
  delete [] tmp;
}

#define TOL 1.0e-5

/*!
  \brief Compute the SVD decomposition

  Computes the singular value decomposition of the matrix, U.
  The contents of U are replaced such that A = U*S*V' where A represents
  the initial value of U.
  S is understood to have only room for ncol elements.
  The matrix V may be NULL, in which case, no values are returned for V.

  This SVD routine is based on pgs 30-48 of "Compact Numerical Methods
  for Computers" by J.C. Nash (1990), used to compute the pseudoinverse.

  Gary William Flake
  http://research.yahoo.com/~flakeg/nodelib/html/
  http://www.neci.nec.com/homepages/flake/nodelib/html/svd.html (not valid)

  \sa SVDcmp and SVDksb
*/

#define TOLERANCE 1.0e-7

static
void svd_internal_use(double *U, double *S, double *V,
		      unsigned int nRow, unsigned int nCol)
{
  unsigned int i, j, k, EstColRank, RotCount, SweepCount, slimit;
  double eps, e2, tol, vt, p, x0, y0, q, r, c0, s0, d1, d2;

  eps = TOLERANCE;
  slimit = nCol / 4;
  if (slimit < 6.0)
    slimit = 6;
  SweepCount = 0;
  e2 = 10.0 * nRow * eps * eps;
  tol = eps * .1;
  EstColRank = nCol;
  if(V)
    for (i = 0; i < nCol; i++)
      for (j = 0; j < nCol; j++) {
	V[nCol * i + j] = 0.0;
	V[nCol * i + i] = 1.0;
      }
  RotCount = EstColRank * (EstColRank - 1) / 2;
  while (RotCount != 0 && SweepCount <= slimit) {
    RotCount = EstColRank * (EstColRank - 1) / 2;
    SweepCount++;
    for (j = 0; j < EstColRank - 1; j++) {
      for (k = j + 1; k < EstColRank; k++) {
	p = q = r = 0.0;
	for (i = 0; i < nRow; i++) {
	  x0 = U[nCol * i + j];
	  y0 = U[nCol * i + k];
	  p += x0 * y0;
	  q += x0 * x0;
	  r += y0 * y0;
	}
	S[j] = q;
	S[k] = r;
	if (q >= r) {
	  if (q <= e2 * S[0] || fabs(p) <= tol * q)
	    RotCount--;
	  else {
	    p /= q;
	    r = 1 - r / q;
	    vt = sqrt(4 * p * p + r * r);
	    c0 = sqrt(fabs(.5 * (1 + r / vt)));
	    s0 = p / (vt * c0);
	    for (i = 0; i < nRow; i++) {
	      d1 = U[nCol * i + j];
	      d2 = U[nCol * i + k];
	      U[nCol * i + j] = d1 * c0 + d2 * s0;
	      U[nCol * i + k] = -d1 * s0 + d2 * c0;
	    }
	    if(V)
	      for (i = 0; i < nCol; i++) {
		d1 = V[nCol * i + j];
		d2 = V[nCol * i + k];
		V[nCol * i + j] = d1 * c0 + d2 * s0;
		V[nCol * i + k] = -d1 * s0 + d2 * c0;
	      }
	  }
	}
	else {
	  p /= r;
	  q = q / r - 1;
	  vt = sqrt(4 * p * p + q * q);
	  s0 = sqrt(fabs(.5 * (1 - q / vt)));
	  if (p < 0)
	    s0 = -s0;
	  c0 = p / (vt * s0);
	  for (i = 0; i < nRow; i++) {
	    d1 = U[nCol * i + j];
	    d2 = U[nCol * i + k];
	    U[nCol * i + j] = d1 * c0 + d2 * s0;
	    U[nCol * i + k] = -d1 * s0 + d2 * c0;
	  }
	  if(V)
	    for (i = 0; i < nCol; i++) {
	      d1 = V[nCol * i + j];
	      d2 = V[nCol * i + k];
	      V[nCol * i + j] = d1 * c0 + d2 * s0;
	      V[nCol * i + k] = -d1 * s0 + d2 * c0;
	    }
	}
      }
    }
    while (EstColRank >= 3 && S[(EstColRank - 1)] <= S[0] * tol + tol * tol)
      EstColRank--;
  }
  for(i = 0; i < nCol; i++)
    S[i] = sqrt(S[i]);
  for(i = 0; i < nCol; i++)
    for(j = 0; j < nRow; j++)
      U[nCol * j + i] = U[nCol * j + i] / S[i];
}

/*!
  \brief Singular value decomposition (other function)

  Given a matrix A (m x n) this routine compute its singular value decomposition
  A = U W V^T. The matrice U replace A on output. the diagonal matrix of
  singular value is output as a vector W (n).  The matrix V (not the transpose
  V^T) is output as V (n x n)


  \warning Destructive wrt A
  \warning


  \sa SVD for a more intuitive use

  This SVD routine is based on pgs 30-48 of "Compact Numerical Methods
  for Computers" by J.C. Nash (1990), used to compute the pseudoinverse.

  http://www.neci.nec.com/homepages/flake/nodelib/html/svd.html
  http://labs.yahoo.com/~flakeg/nodelib/html/svd.html

  \sa SVDcmp and SVDksb

*/

void vpMatrix::svdFlake(vpColVector &W, vpMatrix &V)
{


  svd_internal_use(data, W.data, V.data, getRows(), getCols());
}


#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#    include <opencv2/core/core.hpp>
void vpMatrix::svdOpenCV(vpColVector& w, vpMatrix& v){
  int rows = (int)this->getRows();
  int cols = (int)this->getCols();
  cv::Mat m(rows, cols, CV_64F, this->data);
  cv::SVD opencvSVD(m);
  cv::Mat opencvV = opencvSVD.vt;
  cv::Mat opencvW = opencvSVD.w;
  v.resize((unsigned int)opencvV.rows, (unsigned int)opencvV.cols);
  w.resize((unsigned int)(opencvW.rows*opencvW.cols));
  
  memcpy(v.data, opencvV.data, (size_t)(8*opencvV.rows*opencvV.cols));
  v=v.transpose();
  memcpy(w.data, opencvW.data, (size_t)(8*opencvW.rows*opencvW.cols));
  this->resize((unsigned int)opencvSVD.u.rows, (unsigned int)opencvSVD.u.cols);
  memcpy(this->data,opencvSVD.u.data, (size_t)(8*opencvSVD.u.rows*opencvSVD.u.cols));
}

#endif

#ifdef VISP_HAVE_LAPACK_C
extern "C" int dgesdd_(char *jobz, int *m, int *n, double *a, int *lda, double *s, double *u, int *ldu, double *vt, int *ldvt, double *work, int *lwork, int *iwork, int *info);
#include <stdio.h>
#include <string.h>

void vpMatrix::svdLapack(vpColVector& W, vpMatrix& V){
  /* unsigned */ int m = static_cast<int>(this->getCols()), n = static_cast<int>(this->getRows()), lda = m, ldu = m, ldvt = std::min(m,n);
  int info, lwork;

  double wkopt;
  double* work;

  int* iwork = new int[8*static_cast<unsigned int>(std::min(n,m))];

  double *s = W.data;
  double* a = new double[static_cast<unsigned int>(lda*n)];
  memcpy(a,this->data,this->getRows()*this->getCols()*sizeof(double));
  double* u = V.data;
  double* vt = this->data;



  lwork = -1;
  dgesdd_( (char*)"S", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, &wkopt, &lwork, iwork, &info );
  lwork = (int)wkopt;
  work = new double[static_cast<unsigned int>(lwork)];

  dgesdd_( (char*)"S", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, work, &lwork, iwork, &info );

  if( info > 0 ) {
   vpTRACE("The algorithm computing SVD failed to converge.");
   throw(vpMatrixException(vpMatrixException::fatalError,
         "The algorithm computing SVD failed to converge.")) ;

  }

  V=V.transpose();
  delete[] work;
  delete[] iwork;
  delete[] a;
}
#endif

#ifdef VISP_HAVE_GSL
#include <gsl/gsl_linalg.h>

void
vpMatrix::svdGsl(vpColVector& w, vpMatrix& v)
{
  
#if 0 
  // premier test avec la gsl 1. on recopie...
  int i,j ;

  int nc = getCols() ;
  int nr = getRows() ;
  gsl_matrix *A = gsl_matrix_alloc(nr, nc) ;

  int Atda = A->tda ;
  for (i=0 ; i < nr ; i++)
  {
    int k = i*Atda ;
    for (j=0 ; j < nc ; j++)
      A->data[k+j] = (*this)[i][j] ;
  }
  // gsl_matrix_set(A,i,j,(*this)[i][j]) ;

  gsl_matrix *V = gsl_matrix_alloc(nc, nc) ;
  gsl_vector *S = gsl_vector_alloc(nc) ;
  gsl_vector *work = gsl_vector_alloc(nc) ;

  gsl_linalg_SV_decomp(A,V,S, work) ;
//  gsl_linalg_SV_decomp_jacobi(A,V,S) ;


  //l'acces par gsl_matrix_get est tres lourd, voir si on peut pas faire
  // autremement (surement !)

  Atda = A->tda ;
  for (i=0 ; i < nr ; i++)
    for (j=0 ; j < nc ; j++)
      (*this)[i][j] =  gsl_matrix_get(A,i,j) ;

  int Vtda = V->tda ;
  for (i=0 ; i < nc ; i++)
  {
    int k = i*Vtda ;
    for (j=0 ; j < nc ; j++)
      v[i][j] = V->data[k+j] ;
  }

  for (j=0 ; j < nc ; j++)
    w[j] = gsl_vector_get(S,j) ;


  gsl_matrix_free(V) ;
  gsl_matrix_free(A) ;
  gsl_vector_free(S) ;
  gsl_vector_free(work) ;

#else //optimisation Anthony 20/03/2008
  
  unsigned int nc = getCols() ;
  unsigned int nr = getRows() ;
  gsl_vector *work = gsl_vector_alloc(nc) ;

//  gsl_linalg_SV_decomp_jacobi(A,V,S) ;


  //l'acces par gsl_matrix_get est tres lourd, voir si on peut pas faire
  // autremement (surement !)

  gsl_matrix A;
  A.size1 = nr;
  A.size2 = nc;
  A.tda = A.size2;
  A.data = this->data;
  A.owner = 0;
  A.block = 0;
  
  gsl_matrix V;
  V.size1 = nc;
  V.size2 = nc;
  V.tda = V.size2;
  V.data = v.data;
  V.owner = 0;
  V.block = 0;
  
  gsl_vector S;
  S.size = nc;
  S.stride = 1;
  S.data = w.data;
  S.owner = 0;
  S.block = 0;
  
  gsl_linalg_SV_decomp(&A,&V,&S, work) ;
  
  gsl_vector_free(work) ;

#endif  
}
#endif // # #GSL


#undef TOL
#undef TOLERANCE

#undef MAX_ITER_SVD

#endif // doxygen should skip this
