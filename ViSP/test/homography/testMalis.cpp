

/*!
  \file testMalis.cpp
  \brief tests transformation within various representations of rotation
*/


#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpHomography.h>
#include <visp/vpDebug.h>
#include <visp/vpThetaUVector.h>

#include <visp/vpPoint.h>
#include <visp/vpMath.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpDebug.h>

#define L 0.1
#define nbpt 5
/*!
  \example testMalis.cpp

  Test the Malis homography estimation algorithm

*/
int
main()
{
  int i ;

  vpPoint P[nbpt]  ;  //  Point to be tracked
  double xa[nbpt], ya[nbpt] ;
  double xb[nbpt], yb[nbpt] ;

  vpPoint aP[nbpt]  ;  //  Point to be tracked
  vpPoint bP[nbpt]  ;  //  Point to be tracked

  P[0].setWorldCoordinates(-L,-L, 0 ) ;
  P[1].setWorldCoordinates(2*L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,3*L, 0 ) ;
  P[4].setWorldCoordinates(0,0, 0 ) ;
  /*
    P[5].setWorldCoordinates(10,20, 0 ) ;
    P[6].setWorldCoordinates(-10,12, 0 ) ;
  */
  vpHomogeneousMatrix bMo(0,0,1, 0,0,0) ;
  vpHomogeneousMatrix aMb(1,0,0.0,vpMath::rad(10),0,vpMath::rad(40)) ;
  vpHomogeneousMatrix aMo =aMb*bMo ;
  for(i=0 ; i < nbpt ; i++)
  {
    P[i].project(aMo) ;
    aP[i] = P[i] ;
    xa[i] = P[i].get_x() ;
    ya[i] = P[i].get_y() ;
  }

  for(i=0 ; i < nbpt ; i++)
  {
    P[i].project(bMo) ;
    bP[i] = P[i] ;
    xb[i] = P[i].get_x() ;
    yb[i] = P[i].get_y() ;
  }
  cout << "-------------------------------" <<endl ;
  cout << "aMb "<<endl <<aMb << endl ;
  cout << "-------------------------------" <<endl ;
  vpHomography aHb ;

  vpHomography::HLM(nbpt,xb,yb,xa,ya,true, aHb) ;

  TRACE("aHb computed using the Malis paralax  algorithm") ;
  aHb /= aHb[2][2] ;
  cout << endl << aHb<<  endl ;

  vpRotationMatrix aRb  ;
  vpTranslationVector aTb ;
  vpColVector n ;

  cout << "-------------------------------" <<endl ;
  TRACE("extract R, T and n ") ;
  aHb.computeDisplacement(aRb, aTb, n) ;
  cout << "Rotation: aRb" <<endl ;
  cout << aRb << endl ;
  cout << "Translation: aTb" <<endl;
  cout << (aTb).t() <<endl   ;
  cout << "Normal to the plane: n" <<endl;
  cout << (n).t() <<endl ;


  cout << "-------------------------------" <<endl ;
  TRACE("Compare with built homography H = R + t/d ") ;
  vpPlane bp(0,0,1,1) ;
  vpHomography aHb_built(aMb,bp) ;
  TRACE( "aHb built from the displacement ") ;
  cout <<  endl <<aHb_built/aHb_built[2][2] <<  endl ;

  aHb_built.computeDisplacement(aRb, aTb, n) ;
  cout << "Rotation: aRb" <<endl ;
  cout << aRb << endl ;
  cout << "Translation: aTb" <<endl;
  cout << (aTb).t() <<endl ;
  cout << "Normal to the plane: n" <<endl;
  cout << (n).t() <<endl ;

  cout << "-------------------------------" <<endl ;
  TRACE("test if ap = aHb bp") ;

  for(i=0 ; i < nbpt ; i++)
  {
    cout << "Point "<< i<< endl ;
    vpPoint p ;
    cout << "(" ;
    cout << aP[i].get_x()/aP[i].get_w()<<", "<< aP[i].get_y()/aP[i].get_w() ;
    cout <<") =  (" ;
    p = aHb*bP[i] ;
    cout << p.get_x() /p.get_w()<<",  "<< p.get_y()/ p.get_w() <<")"<<endl ;
  }

  cout << "-------------------------------" <<endl ;
  TRACE("test displacement") ;

  vpList<vpRotationMatrix> laRb ;
  vpList<vpTranslationVector> laTb ;
  vpList<vpColVector> lnb ;

  vpHomography::computeDisplacement(aHb,bP[0].get_x(),bP[0].get_y(),
				    laRb, laTb, lnb) ;

  laRb.front() ;
  laTb.front() ;
  lnb.front() ;
  int k =1 ;
  while (!lnb.outside())
  {
    cout << "Solution " << k++ << endl ;

    aRb = laRb.value() ;
    aTb = laTb.value() ;
    n = lnb.value() ;
    cout << "Rotation: aRb" <<endl ;
    cout << aRb << endl ;
    cout << "Translation: aTb" <<endl;
    cout << (aTb).t() <<endl ;
    cout << "Normal to the plane: n" <<endl;
    cout << (n).t() <<endl ;

    laRb.next() ;
    laTb.next() ;
    lnb.next() ;
  }

}
