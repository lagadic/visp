
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_svd.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: test_svd.cpp,v 1.2 2005-06-29 12:20:01 fspindle Exp $
 *
 * Description
 * ============
 *  test various svd decomposition
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example test_svd.cpp
  \brief  test various svd decomposition
*/


#include <visp/vpTime.h>

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

int
main()
{
  int i,j ;
  vpMatrix L(60000,6), Ls ;
  for (i=0 ; i < L.getRows() ; i++)
    for  (j=0 ; j < L.getCols() ; j++)
      L[i][j] = 2*i+j + cos((double)(i+j))+((double)(i)) ;
  //  cout << L << endl ;
  Ls = L ;
  cout << "--------------------------------------"<<endl ;

  vpColVector W(L.getCols()) ;
  vpMatrix V(L.getCols(), L.getCols()) ;

  long t = vpTime::measureTimeMs() ;
  L.svdNr(W,V) ;
  t = vpTime::measureTimeMs() -t ;

  cout <<"svdNr Numerical recipes \n time " <<t << endl;
    cout << W.t() ;
  cout << "--------------------------------------"<<endl ;

#if (defined HAVE_LIBGSL) && (defined HAVE_LIBGSLCBLAS)
  L = Ls ;
  t = vpTime::measureTimeMs() ;
  L.svdGsl(W,V) ;
  t = vpTime::measureTimeMs() -t ;
  cout <<"svdGsl_mod \n time " <<t << endl;
    cout << W.t() ;

  cout << "--------------------------------------"<<endl ;
#endif

  L = Ls ;
  t = vpTime::measureTimeMs() ;
  L.svdFlake(W,V) ;
  t = vpTime::measureTimeMs() -t ;
  cout <<"svdFlake\n time " <<t << endl;
    cout << W.t() ;



}

