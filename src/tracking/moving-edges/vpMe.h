/****************************************************************************
 *
 * $Id: vpMe.h,v 1.5 2007-05-02 13:29:41 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
 * \class vpMe
 * \brief Contains predetermined masks for sites, holds ecm
 * \n			tracking parameters
 * \author Adapted from vpMe ViSP,
 * \n Andrew Comport added functionality - replaced points_to_track with
 * \n sample step.
 * \date 4/7/03
*/
// ====================================================================

#ifndef vpMe_H
#define vpMe_H

#include <visp/vpConfig.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpImage.h>


class VISP_EXPORT vpMe
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
  virtual ~vpMe() ;

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


