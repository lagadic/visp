


#ifndef vpMeLine_HH
#define vpMeLine_HH


#include <math.h>
#include <iostream>


#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpMeTracker.h>



class vpMeLine : public vpMeTracker
{
private:
  vpMeSite PExt[2] ;

  double rho, theta ;
  double delta ;

public:
  double a, b, c ; // ai + bj + c = 0


public:
  vpMeLine() ;
  ~vpMeLine() ;

  void display(vpImage<unsigned char>& I,  int col) ;

  void track(vpImage<unsigned char>& Im);

  void sample(vpImage<unsigned char>&image);
  void reSample(vpImage<unsigned char> &I) ;
  void leastSquare(vpImage<unsigned char> &I) ;
  void updateDelta();
  void setExtremities() ;
  void seekExtremities(vpImage<unsigned char> &I) ;
  void suppressPoints() ;

  void initTracking(vpImage<unsigned char> &I) ;
  void initTracking(vpImage<unsigned char> &I, int i1,int j1, int i2, int j2) ;

  void computeRhoTheta(vpImage<unsigned char> &I) ;
  double getRho() const ;
  double getTheta() const ;
};




#endif


