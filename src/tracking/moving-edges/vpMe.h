/*!
	\file vpMe.h
	\name Element de Contour en Mouvement
*/

// ====================================================================
/*!
 * \class vpMe
 * \brief Contains predetermined masks for sites, holds ecm
 * \n			tracking parameters
 * \author Adapted from vpMe ViSP,
 * \n Andrew Comport added functionality - replaced points_to_track with
 * \n sample step.
 * \date 4/7/03
*/
// ====================================================================

#ifndef ECM_HH
#define ECM_HH

#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpImage.h>


class vpMe
{
public:
  double threshold ;// likelihood ratio threshold
  double mu1; //contrast continuity parameter(left boundary)
  double mu2; //contrast continuity parameter(right boundary)
  double min_samplestep;
  int anglestep;
	// convolution masks' size in pixels (masks are square)
  int mask_size;
  // the number of convolution masks available for tracking ;
	// defines resolution
  int n_mask;
  int mask_sign;
  int range; //seek range - on both sides of the reference pixel
  double sample_step; // Distance between sampled points (in pixels)
  int ntotal_sample;
  int points_to_track;

  //strip: defines a "security strip" such that Seek_Extremities()
  //cannot return a new extremity which is too close to the
  //frame borders which may cause Get_Sampling_Grid to refuse
  //the that extremity
  int strip;
  double aberration;
  double init_aberration;
  //int graph ;
  vpMatrix *mask ;

  vpMe() ;
  ~vpMe() ;

  void initMask() ;// convolution masks - offset computation
  void print( ) ;

  void setThreshold(double lambda) { threshold = lambda ; }
  void setPointsToTrack(int number) { points_to_track = number ; }
  void setAngleStep(int a) { anglestep =a  ; }
  void setRange(int a) { range =a  ; }
  void setMu1(double a) { mu1 =a  ; }
  void setMu2(double a) { mu2 =a  ; }
  void setSizeMask(int a) ;
  void setNumberMask(int a) ;
  void setMaskSign(int a){mask_sign = a ; }
  double GetSampleStep() { return sample_step ; }
  void setSampleStep(double a) { sample_step = a ; }
  void setStrip(int a) { strip = a ; }
  // in CPixel.convolution() : avoids to get points (In Appariement()
  // and SeekExtremities()) that Get_Sampling_Grid()
  // would reject since too close to the frame borders ;
  void setMinSamplestep(double a) { min_samplestep = a ; }
  //sets the minimum samplestep in pixels ;

  void setAberration( double a) { aberration = a ; }
  void setInitAberration(double a ) { init_aberration = a ; }
  void checkSamplestep(double &a) { if(a < min_samplestep) a = min_samplestep ; }
};


#endif


