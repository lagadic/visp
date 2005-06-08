/*!
	\file vpSiteEcm.h
	\name Element de Contour en Mouvement
*/

// ====================================================================
/*!
 * \class vpSiteMe
 * \brief Performs search in a given direction(normal) for a given distance(pixels)
 * \n for a given 'site'. Gives the most likely site given the probablility
 * \n from an ME mask
 * \author Andrew Comport - adapted from vpSiteMe ViSP and added functionality :
 * \n - Bug fix rewrote application of masks to use the temporal information
 * \n instead of applying both temporal masks to the same image.
 * \n ie: spacial -> spatio/temporal
 * \n - Added new tracking function to choose the most similar edge amongst
 * \n - all edges found.
 * \n sample step.
 * \date 3/3/03
*/
// ====================================================================




#ifndef SITE_HH
#define SITE_HH

#include <visp/vpMatrix.h>
#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpMe.h>

class vpMeSite // : public CPixel<double>
{
public:
  int i,j ;
  int i_1, j_1 ;
  double ifloat, jfloat ;
  unsigned char v ;
  int mask_sign ;
public:
  // Angle of tangent at site
  double alpha;

  // Convolution of Site in previous image
  double convlt ;

  //! Flag to indicate whether point is rejected or not
  //! 1 = contrast, 2 = threshold, 3 = M-estimator, 0 = nosupp
  int suppress;

  // Uncertainty of point given as a probability between 0 and 1
  double weight;

public:
  void init() ;
  void init(double ip, double jp, double alphap) ;
  void init(double ip, double jp, double alphap, double convltp) ;
  void init(double ip, double jp, double alphap, double convltp, int sign) ;

  vpMeSite () ;
  vpMeSite(double ip, double jp) ;
  ~vpMeSite() {} ;

  vpMeSite &operator=(const vpMeSite &m) ;
  int operator!=(const vpMeSite  &m) ;

  void getSign(vpImage<unsigned char> &I, const int range) ;
  double convolution(vpImage<unsigned char>& ima, const vpMe *me) ;

  vpMeSite *getQueryList(vpImage<unsigned char> &I, const int range) ;

  void track(vpImage<unsigned char>& im,
	     const vpMe *me,
	     const  bool test_contraste=true);

private:
  int selectDisplay ;
  enum displayEnum
    {
      NONE,
      RANGE,
      RESULT,
      RANGE_RESULT
    } ;
public:
  void setDisplay(int select) { selectDisplay = select ; }
} ;

#endif
