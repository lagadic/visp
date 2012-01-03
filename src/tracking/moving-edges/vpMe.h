/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 * Andrew Comport
 *
 *****************************************************************************/

/*!
	\file vpMe.h
	\brief Moving edges
*/

// ====================================================================
/*!
  \class vpMe
  \ingroup TrackingImageME
  
  \brief Contains predetermined masks for sites and holds moving edges
  tracking parameters.
   
*/
// ====================================================================
//  Andrew Comport added functionality - replaced points_to_track with
//  sample step.

#ifndef vpMe_H
#define vpMe_H

#include <visp/vpMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpImage.h>


class VISP_EXPORT vpMe
{
public:
  double threshold ;//! Likelihood ratio threshold
  double mu1; //! Contrast continuity parameter (left boundary)
  double mu2; //! Contrast continuity parameter (right boundary)
  double min_samplestep;
  unsigned int anglestep;
  int mask_sign;
  unsigned int range; //! Seek range - on both sides of the reference pixel
  double sample_step; //! Distance between sampled points (in pixels)
  int ntotal_sample;
  int points_to_track;
  //! convolution masks' size in pixels (masks are square), \warning should not be public, use setMaskSize() and getMaskSize() instead (kept public for compatibility reasons).
  unsigned int mask_size;
  //! the number of convolution masks available for tracking ; defines resolution. \warning Should not be public, use setMaskNumber() and getMaskNumber() instead (kept public for compatibility reasons).
  unsigned int n_mask;

  //strip: defines a "security strip" such that Seek_Extremities()
  //cannot return a new extremity which is too close to the
  //frame borders which may cause Get_Sampling_Grid to refuse
  //the that extremity
  int strip;
  double aberration;
  double init_aberration;
  //int graph ;
  //! Array of matrices defining the different masks (one for every angle step).
  vpMatrix *mask ;

  vpMe() ;
  vpMe(const vpMe &me) ;
  virtual ~vpMe() ;
  
  const vpMe& operator=(const vpMe &me);


  void initMask() ;// convolution masks - offset computation
  void print( ) ;

  /*!
    Set the likelihood threshold.
   */
  void setThreshold(const double t) { threshold = t ; }
  /*! 
    Set the number of points to track.

    \warning This method is useful only for the vpMeNurbsTracker.  
   */
  void setPointsToTrack(const int n) { points_to_track = n; }
  void setAngleStep(const unsigned int a) { anglestep = a ; }
  //! Set the seek range on on both sides of the reference pixel.
  void setRange(const unsigned int r) { range = r  ; }
  void setMu1(const double mu1) { this->mu1 = mu1  ; }
  void setMu2(const double mu2) { this->mu2 = mu2  ; }
  void setMaskNumber(const unsigned int a) ;
  void setMaskSign(const int a){mask_sign = a ; }
  void setMaskSize(const unsigned int a) ;
  double getSampleStep() const { return sample_step ; }
  void setSampleStep(const double s) { sample_step = s ; }
  void setStrip(const int a) { strip = a ; }
  // in CPixel.convolution() : avoids to get points (In Appariement()
  // and SeekExtremities()) that Get_Sampling_Grid()
  // would reject since too close to the frame borders ;
  void setMinSamplestep(const double a) { min_samplestep = a ; }
  //sets the minimum samplestep in pixels ;

  /*!
    Return the actual mask size. The mask size defines the size of the
    convolution mask used to detect an edge.

    \return the current mask size.
  */
  unsigned int getMaskSize() const {
    return mask_size;
  }

  /*!
    Return the number of mask. The number of mask determines the precision of
    the normal of the edge for every sample. If precision is 2deg, then there
    are 360/2 = 180 masks.

    \return the current number of mask.
  */
  unsigned int getMaskNumber() const{
    return n_mask;
  }

  void setAberration( double a) { aberration = a ; }
  void setInitAberration(double a ) { init_aberration = a ; }
  void checkSamplestep(double &a) { if(a < min_samplestep) a = min_samplestep ; }

};


#endif


