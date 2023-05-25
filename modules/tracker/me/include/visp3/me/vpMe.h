/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

  <b>JSON serialization</b>

  Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpMe.
  The following sample code shows how to save moving-edges settings in a file named `me.json`
  and reload the values from this JSON file.
  \code
  #include <visp3/me/vpMe.h>

  int main()
  {
  #if defined(VISP_HAVE_NLOHMANN_JSON)
    std::string filename = "me.json";
    {
      vpMe me;
      me.setThreshold(10000);
      me.setMaskNumber(180);
      me.setMaskSign(0);
      me.setMu1(0.5);
      me.setMu2(0.5);
      me.setNbTotalSample(0);
      me.setPointsToTrack(200);
      me.setRange(5);
      me.setStrip(2);

      std::ofstream file(filename);
      const nlohmann::json j = me;
      file << j;
      file.close();
    }
    {
      std::ifstream file(filename);
      const nlohmann::json j = nlohmann::json::parse(file);
      vpMe me;
      me = j;
      file.close();
      std::cout << "Read moving-edges settings from " << filename << ":" << std::endl;
      me.print();
    }
  #endif
  }
  \endcode
  If you build and execute the sample code, it will produce the following output:
  \code{.unparsed}
  Read moving-edges settings from me.json:

  Moving edges settings

   Size of the convolution masks....5x5 pixels
   Number of masks..................180
   Query range +/- J................5 pixels
   Likelihood test ratio............10000
   Contrast tolerance +/-...........50% and 50%
   Sample step......................10 pixels
   Strip............................2 pixels
   Min_Samplestep...................4 pixels
  \endcode

  The content of the `me.json` file is the following:
  \code{.unparsed}
  $ cat me.json
  {"maskSign":0,"maskSize":5,"minSampleStep":4.0,"mu":[0.5,0.5],"nMask":180,"ntotalSample":0,"pointsToTrack":200,"range":5,"sampleStep":10.0,"strip":2,"threshold":10000.0}
  \endcode
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
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
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
    Return the likelihood threshold used to determine if the moving edge is valid or not.

    \return Value of threshold.

    \sa setThreshold()
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
    Set the likelihood threshold used to determined if the moving edge is valid or not.

    When the contrast is high between the two sides of the ellipse (for example white/black or black/white)
    a threshold of 1500 is sufficient. On the other hand, when the contrast is less strong, it is necessary
    to increase this threshold according to the images until 20000.

    \param t : new threshold.

    \sa getThreshold()
  */
  void setThreshold(const double &t) { threshold = t; }

#ifdef VISP_HAVE_NLOHMANN_JSON
  friend void to_json(nlohmann::json &j, const vpMe &me);
  friend void from_json(const nlohmann::json &j, vpMe &me);
#endif
};


#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>

/**
 * @brief Convert a vpMe object to a JSON representation
 *
 * @param j resulting json object
 * @param me the object to convert
 */
inline void to_json(nlohmann::json &j, const vpMe &me)
{
  j = {
    {"threshold", me.threshold},
    {"mu", {me.mu1, me.mu2}},
    {"minSampleStep", me.min_samplestep},
    {"sampleStep", me.sample_step},
    {"range", me.range},
    {"ntotalSample", me.ntotal_sample},
    {"pointsToTrack", me.points_to_track},
    {"maskSize", me.mask_size},
    {"nMask", me.n_mask},
    {"maskSign", me.mask_sign},
    {"strip", me.strip}
  };
}

/**
 * @brief Retrieve a vpMe object from a JSON representation
 *
 * JSON content (key: type):
 *  - threshold: double, vpMe::setThreshold
 *  - mu : [double, double], vpMe::setMu1, vpMe::setMu2
 *  - minSampleStep: double, vpMe::setMinSampleStep
 *  - angleStep: double, vpMe::setAngleStep
 *  - sampleStep: double, vpMe::setSampleStep
 *  - range: int, vpMe::setRange
 *  - ntotal_sample: int, vpMe::setNbTotalSample
 *  - pointsToTrack: int, vpMe::setPointsToTrack
 *  - maskSize: int, vpMe::setMaskSize
 *  - nMask: int, vpMe::setMaskNumber
 *  - maskSign: int, vpMe::setMaskSign
 *  - strip: int, vpMe::setStrip
 *
 * Example:
 * \code{.json}
 * {
 *  "angleStep": 1,
    "maskSign": 0,
    "maskSize": 5,
    "minSampleStep": 4.0,
    "mu": [
        0.5,
        0.5
    ],
    "nMask": 180,
    "ntotal_sample": 0,
    "pointsToTrack": 500,
    "range": 7,
    "sampleStep": 4.0,
    "strip": 2,
    "threshold": 5000.0
  }
 * \endcode
 *
 * @param j JSON representation to convert
 * @param me converted object
 */
inline void from_json(const nlohmann::json &j, vpMe &me)
{
  me.threshold = j.value("threshold", me.threshold);

  if (j.contains("mu")) {
    std::vector<double> mus = j.at("mu").get<std::vector<double>>();
    assert((mus.size() == 2));
    me.setMu1(mus[0]);
    me.setMu2(mus[1]);
  }
  me.min_samplestep = j.value("minSampleStep", me.min_samplestep);

  me.range = j.value("range", me.range);
  me.ntotal_sample = j.value("ntotalSample", me.ntotal_sample);
  me.points_to_track = j.value("pointsToTrack", me.points_to_track);
  me.mask_size = j.value("maskSize", me.mask_size);
  me.mask_sign = j.value("maskSign", me.mask_sign);
  me.strip = j.value("strip", me.strip);
  if (j.contains("angleStep") && j.contains("nMask")) {
    std::cerr << "both angle step and number of masks are defined, number of masks will take precedence" << std::endl;
    me.setMaskNumber(j["nMask"]);
  }
  else if (j.contains("angleStep")) {
    me.setAngleStep(j["angleStep"]);
  }
  else if (j.contains("nMask")) {
    me.setMaskNumber(j["nMask"]);
  }
  me.initMask();
}


#endif

#endif
