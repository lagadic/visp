

/*!
  \file test_homography.cpp
  \brief tests transformation within various representations of rotation
*/


#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomography.h>
#include <visp/vpDebug.h>
#include <visp/vpThetaUVector.h>

int
main()
{



  vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45)) ;

  cout << "Initialization " <<endl ;
  // cout << tu << endl ;


  cout << "From vpThetaUVector to vpRotationMatrix " << endl ;
  vpRotationMatrix R(tu)  ;


  // pure rotation
  vpHomogeneousMatrix M ;
  M.insert(R) ;

  M[2][3] = 1 ;

  cout << M << endl ;
  vpPlane p(0,0,1,1) ;

  vpHomography H(M,p) ;

  TRACE(" ") ;
  cout << H << endl ;

  TRACE(" ") ;
  H.extract() ;
  TRACE(" ") ;

}
