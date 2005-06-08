

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



  {
    vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45)) ;

    cout << "Initialization " <<endl ;
    // cout << tu << endl ;


    cout << "From vpThetaUVector to vpRotationMatrix " << endl ;
    vpRotationMatrix R(tu)  ;


    // pure rotation
    vpHomogeneousMatrix M ;
    M.insert(R) ;



    cout << "M" <<endl <<M << endl ;
    vpPlane p(0,0,1,1) ;

    vpHomography H(M,p) ;

    TRACE(" ") ;
    cout << "H" <<endl <<H << endl ;


    TRACE(" ") ;

    vpColVector n ;
    vpTranslationVector T ;

    H.computeDisplacement(R,T,n) ;

    cout << "R" <<endl << R ;
    cout << "T" <<endl << T.t()  ;
    cout << "n" <<endl << n.t()  ;
    TRACE(" ") ;
    TRACE(" ") ;
  }
  cout <<"------------------------------------------------------" << endl ;

  {
    vpThetaUVector tu(vpMath::rad(90), vpMath::rad(120), vpMath::rad(45)) ;

    cout << "Initialization " <<endl ;
    // cout << tu << endl ;


    cout << "From vpThetaUVector to vpRotationMatrix " << endl ;
    vpRotationMatrix R(tu)  ;


    // pure rotation
    vpHomogeneousMatrix M ;
    M.insert(R) ;


    M[0][3] = 0.21 ;
    M[1][3] = 0.31 ;
    M[2][3] = 0.5 ;


    cout << "M" <<endl <<M << endl ;
    vpPlane p(0,0,1,1) ;

    vpHomography H(M,p) ;

    TRACE(" ") ;
    cout << "H" <<endl <<H << endl ;


    TRACE(" ") ;

    vpColVector n ;
    vpTranslationVector T ;

    H.computeDisplacement(R,T,n) ;

    cout << "R" <<endl << R ;
    cout << "T" <<endl << T.t()  ;
    cout << "n" <<endl << n.t()  ;
    TRACE(" ") ;
    TRACE(" ") ;
  }

  cout <<"------------------------------------------------------" << endl ;
  {
    vpThetaUVector  tu(vpMath::rad(-190), vpMath::rad(12), vpMath::rad(-45)) ;

    vpRotationMatrix R(tu)  ;


    // pure rotation
    vpHomogeneousMatrix M ;
    M.insert(R) ;

    M[0][3] = 0.21 ;
    M[1][3] =- 0.31 ;
    M[2][3] = 0.5 ;

    cout << "M" <<endl <<M << endl ;
    vpPlane p(0.4,-0.5,0.5,1) ;

    vpHomography H(M,p) ;

    TRACE(" ") ;
    cout << "H" <<endl <<H << endl ;

    TRACE(" ") ;
    vpColVector n ;
    vpTranslationVector T ;
    H.computeDisplacement(R,T,n) ;

    cout << "R" <<endl << R ;
    cout << "T" <<endl << T.t()  ;
    cout << "n" <<endl << n.t()  ;

    vpPlane p1(n[0],n[1],n[2],1.0) ;
    H.buildFrom(R,T,p1) ;
    cout << "H" <<endl <<H << endl ;

    TRACE(" ") ;


  }
}
