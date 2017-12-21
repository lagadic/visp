/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Moving edges.
 *
 * Authors:
 * Eric Marchand
 * Andrew Comport
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
        \file vpMe.h
        \brief Moving edges
*/

#ifndef vpMe_H
#define vpMe_H

#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

/*!
  \class vpMe
  \ingroup module_me

  This class defines predetermined masks for sites and holds moving edges
  tracking parameters.
 */
class VISP_EXPORT vpMe
{
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
#else
private:
#endif
  double threshold; //! Likelihood ratio threshold
  double mu1;       //! Contrast continuity parameter (left boundary)
  double mu2;       //! Contrast continuity parameter (right boundary)
  double min_samplestep;
  unsigned int anglestep;
  int mask_sign;
  unsigned int range; //! Seek range - on both sides of the reference pixel
  double sample_step; //! Distance between sampled points (in pixels)
  int ntotal_sample;
  int points_to_track;
  //! convolution masks' size in pixels (masks are square), \warning should
  //! not be public, use setMaskSize() and getMaskSize() instead (kept public
  //! for compatibility reasons).
  unsigned int mask_size;
  //! the number of convolution masks available for tracking ; defines
  //! resolution. \warning Should not be public, use setMaskNumber() and
  //! getMaskNumber() instead (kept public for compatibility reasons).
  unsigned int n_mask;
  // strip: defines a "security strip" such that Seek_Extremities()
  // cannot return a new extremity which is too close to the
  // frame borders which may cause Get_Sampling_Grid to refuse
  // the that extremity
  int strip;
  // int graph ;
  vpMatrix *mask; //! Array of matrices defining the different masks (one for
                  //! every angle step).

public:
  vpMe();
  vpMe(const vpMe &me);
  virtual ~vpMe();

  vpMe &operator=(const vpMe &me);
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  vpMe &operator=(const vpMe &&me);
#endif

  void checkSamplestep(double &a)
  {
    if (a < min_samplestep)
      a = min_samplestep;
  }
  /*!
    Return the angle step.

    \return Value of anglestep.
  */
  inline unsigned int getAngleStep() const { return anglestep; }
  /*!
    Get the matrix of the mask.

    \return the value of mask.
  */
  inline vpMatrix *getMask() const { return mask; }
  /*!
    Return the number of mask  applied to determine the object contour. The
    number of mask determines the precision of the normal of the edge for
    every sample. If precision is 2deg, then there are 360/2 = 180 masks.

    \return the current number of mask.
  */
  inline unsigned int getMaskNumber() const { return n_mask; }
  /*!
    Return the mask sign.

    \return Value of mask_sign.
  */
  inline int getMaskSign() const { return mask_sign; }
  /*!
    Return the actual mask size (in pixel) used to compute the image gradient
    and determine the object contour. The mask size defines the size of the
    convolution mask used to detect an edge.

    \return the current mask size.
  */
  inline unsigned int getMaskSize() const { return mask_size; }
  /*!
    Get the minimum allowed sample step. Useful to specify a lower bound when
    the sample step is changed.

    \return Value of min_samplestep.
  */
  inline double getMinSampleStep() const { return min_samplestep; }
  /*!
    Get the minimum image contrast allowed to detect a contour.

    \return Value of mu1.
  */
  inline double getMu1() const { return mu1; }
  /*!
    Get the maximum image contrast allowed to detect a contour.

    \return Value of mu2.
  */
  inline double getMu2() const { return mu2; }
  /*!
    Get how many discretizied points are used to track the feature.

    \return Value of ntotal_sample.
  */
  inline int getNbTotalSample() const { return ntotal_sample; }
  /*!
    Return the number of points to track.

    \return Value of points_to_track.
  */
  inline int getPointsToTrack() const { return points_to_track; }
  /*!
    Return the seek range on both sides of the reference pixel.

    \return Value of range.
  */
  inline unsigned int getRange() const { return range; }
  /*!
    Get the number of pixels that are ignored around the image borders.

    \return the value of strip.
  */
  inline int getStrip() const { return strip; }

  /*!
    Return the likelihood threshold used to determined if the moving edge is
    valid or not.

    \return Value of threshold.
  */
  inline double getThreshold() const { return threshold; }

  void initMask(); // convolution masks - offset computation
  void print();

  /*!
    Set the angle step.

    \param a : new angle step.
  */
  void setAngleStep(const unsigned int &a) { anglestep = a; }
  /*!
    Set the number of mask applied to determine the object contour. The number
    of mask determines the precision of the normal of the edge for every
    sample. If precision is 2deg, then there are 360/2 = 180 masks.

    \param a : the number of mask.
  */
  void setMaskNumber(const unsigned int &a);

  /*!
    Set the mask sign.

    \param a : new mask sign.
  */
  void setMaskSign(const int &a) { mask_sign = a; }

  /*!
    Set the mask size (in pixel) used to compute the image gradient and
    determine the object contour. The mask size defines the size of the
    convolution mask used to detect an edge.

    \param a : new mask size.
  */
  void setMaskSize(const unsigned int &a);
  /*!
    Set the minimum allowed sample step. Useful to specify a lower bound when
    the sample step is changed.

    \param min : new minimum sample step.
  */
  void setMinSampleStep(const double &min) { min_samplestep = min; }

  /*!
    Set the minimum image contrast allowed to detect a contour.

    \param mu_1 : new mu1.
  */
  void setMu1(const double &mu_1) { this->mu1 = mu_1; }

  /*!
    Set the maximum image contrast allowed to detect a contour.

    \param mu_2 : new mu2.
  */
  void setMu2(const double &mu_2) { this->mu2 = mu_2; }

  /*!
    Set how many discretizied points are used to track the feature.

    \param nb : new total number of sample.
  */
  void setNbTotalSample(const int &nb) { ntotal_sample = nb; }

  /*!
    Set the number of points to track.

    \param n : new number of points to track.

    \warning This method is useful only for the vpMeNurbsTracker.
  */
  void setPointsToTrack(const int &n) { points_to_track = n; }

  /*!
    Set the seek range on both sides of the reference pixel.

    \param r : new range.
  */
  void setRange(const unsigned int &r) { range = r; }

  /*!
    Set the minimum distance in pixel between two discretized points.

    \param s : new sample_step.
  */
  void setSampleStep(const double &s) { sample_step = s; }

  /*!
    Get the minimum distance in pixel between two discretized points.

    \return Value of sample_step.
  */
  inline double getSampleStep() const { return sample_step; }

  /*!
    Set the number of pixels that are ignored around the image borders.

    \param a : new strip.
  */
  void setStrip(const int &a) { strip = a; }

  /*!
    Set the likelihood threshold used to determined if the moving edge is
    valid or not.

    \param t : new threshold.
  */
  void setThreshold(const double &t) { threshold = t; }
};

#endif
