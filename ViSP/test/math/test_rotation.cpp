/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      test_rotation.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: test_rotation.cpp,v 1.1.1.1 2005-06-08 07:08:14 fspindle Exp $
 *
 * Description
 * ============
 *   tests transformation from various representations of rotation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file test_rotation.cpp
  \brief tests transformation within various representations of rotation
*/


#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpEulerVector.h>

int
main()
{

  cout << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << " Tests program for vpRotationMatrix, vpThetaUVector, "  <<endl ;
  cout << " and vpEulerVector " << endl ;
  cout << "-------------------------------------------------------" << endl ;
  cout << endl ;

  vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45)) ;

  cout << "Initialization " <<endl ;
  cout << tu << endl ;


  cout << "From vpThetaUVector to vpRotationMatrix " << endl ;
  vpRotationMatrix R(tu)  ;

  cout << "Matrix R" ;
  if (R.isARotationMatrix()==1) cout <<" is a rotation matrix " << endl ;
  else cout <<" is not a rotation matrix " << endl ;

  cout << R << endl ;

  cout << "From vpRotationMatrix to vpRxyzVector " << endl ;
  vpRxyzVector RxyzBuildFromR(R) ;
  cout <<  RxyzBuildFromR <<endl ;



  cout << "From vpRxyzVector to vpThetaUVector " << endl ;
  cout << "  use From vpRxyzVector to vpRotationMatrix " << endl ;
  cout << "  use From vpRotationMatrix to vpThetaUVector " << endl ;


  vpThetaUVector tuBuildFromEu ;
  tuBuildFromEu.buildFrom(RxyzBuildFromR) ;

  cout << endl ;
  cout <<  "result : should equivalent to the first one " << endl ;
  cout << tuBuildFromEu << endl ;


  vpRzyzVector rzyz(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45)) ;
  cout << "Initialization vpRzyzVector " <<endl ;
  cout << rzyz << endl ;
  cout << "From vpRzyzVector to vpRotationMatrix  " << endl ;
  R.buildFrom(rzyz) ;
  cout << "From vpRotationMatrix to vpRzyzVector " << endl ;
  vpRzyzVector rzyz_final ;
  rzyz_final.buildFrom(R) ;
  cout << rzyz_final << endl ;


  vpRzyxVector rzyx(vpMath::rad(180), vpMath::rad(120), vpMath::rad(45)) ;
  cout << "Initialization vpRzyxVector " <<endl ;
  cout << rzyx << endl ;
  cout << "From vpRzyxVector to vpRotationMatrix  " << endl ;
  R.buildFrom(rzyx) ;
  cout << R << endl ;
  cout << "From vpRotationMatrix to vpRzyxVector " << endl ;
  vpRzyxVector rzyx_final ;
  rzyx_final.buildFrom(R) ;
  cout << rzyx_final << endl ;


}
