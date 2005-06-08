

#ifndef vpMeEllipse_HH
#define vpMeEllipse_HH


#include <math.h>



#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

#include <visp/vpMeTracker.h>
#include <visp/vpMeSite.h>

#include <visp/vpImage.h>
#include <visp/vpColor.h>


class vpMeEllipse : public vpMeTracker
{
public:
  vpMeSite PExt[2] ;

  double theta ;
  //! vecteur de parametres de la quadrique
  //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  vpColVector K ;

  double ic, jc, e, a, b ;
  double ce, se ;
  int i1,j1, i2, j2 ;
  double alpha1 ;
  double alpha2 ;
private:
  //! seek extremities (in degree)
  double seek ;

public:
  int sample_step ;

public:
  vpMeEllipse() ;
  ~vpMeEllipse() ;


  void setSeekExtremities(double _seek) {seek = _seek ; }
  void display(vpImage<unsigned char>&I,  int col) ;

private:
  void computeAngle(int ip1, int jp1,int ip2, int jp2) ;
  void computeAngle(int ip1, int jp1, double &alpha1,
	     int ip2, int jp2, double &alpha2) ;

  void sample(vpImage<unsigned char>&image);
  void reSample(vpImage<unsigned char> &I) ;
  void leastSquare(vpImage<unsigned char> &I) ;
  void updateTheta();
  void suppressPoints() ;
  void seekExtremities(vpImage<unsigned char> &I) ;

public:

  void getParameters() ;
  void printParameters() ;

  void track(vpImage<unsigned char>& Im);

  void initTracking(vpImage<unsigned char> &I) ;
  void initTracking(vpImage<unsigned char> &I, int n, int *i, int *j) ;

private:
  bool circle ;
public:
  //! set to true if we are sure to track a circle and that this very
  //! unlikely to append in perspective projection, nevertherless for
  //! omnidirectional camera, this can be useful
  //!
  //! in that case
  //! i^2 + K0 j^2 + 2 K1 i j + 2 K2 i + 2 K3 j + K4
  //! K0 = 1
  //! K1 = 0
  void setCircle(bool _circle) { circle = _circle ; }
};




#endif


