

/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_matrix_exception.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: test_matrix_exception.cpp,v 1.3 2006-06-23 14:45:07 brenier Exp $
 *
 * Description
 * ============
 *   tests matrix_exceptions
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \example test_matrix_exception.cpp
  \brief tests matrix exception
*/


#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>

int
main()
{

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Tests  matrix exception " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;

  vpMatrix M ;
  vpMatrix M1(2,3) ;
  vpMatrix M2(3,3) ;
  vpMatrix M3(2,2) ;

  vpTRACE("test matrix size in multiply") ;

  try
  {
    M = M1*M3 ;
  }
  catch (vpMatrixException me)
  {
    vpCTRACE ;
    cout << me << endl ;
  }


  vpTRACE("test matrix size in addition") ;

  try
  {
    M = M1+M3 ;
  }
  catch (vpMatrixException me)
  {
    vpCTRACE ;
    cout << me << endl ;
  }


  vpCTRACE ;
}
