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
 * See https://visp.inria.fr for more information.
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

#ifndef _vpMe_h_
#define _vpMe_h_

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
      me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
      me.setThreshold(20);    // Value in range [0 ; 255]
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
   Likelihood threshold type........normalized
   Likelihood threshold.............20
   Contrast tolerance +/-...........50% and 50%
   Sample step......................10 pixels
   Strip............................2 pixels
   Min sample step..................4 pixels
  \endcode

  The content of the `me.json` file is the following:
  \code{.unparsed}
  $ cat me.json
  {"maskSign":0,"maskSize":5,"minSampleStep":4.0,"mu":[0.5,0.5],"nMask":180,"ntotalSample":0,"pointsToTrack":200,"range":5,"sampleStep":10.0,"strip":2,"threshold":20.0,"thresholdType":1}
  \endcode
 */
class VISP_EXPORT vpMe
{
public:
  typedef enum
  {
    OLD_THRESHOLD = 0,        /*!< Old likelihood ratio threshold (to be avoided). */
    NORMALIZED_THRESHOLD = 1, /*!< Easy-to-use normalized likelihood threshold corresponding to the minimal luminance contrast to consider with values in [0 ; 255]. */
  } vpLikelihoodThresholdType;

private:
  vpLikelihoodThresholdType m_likelihood_threshold_type;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
public:
#else
private:
#endif
  double threshold; //! Old likelihood ratio threshold (to be avoided) or easy-to-use normalized threshold: minimal contrast
  double mu1;       //! Contrast continuity parameter (left boundary)
  double mu2;       //! Contrast continuity parameter (right boundary)
  double min_samplestep;
  unsigned int anglestep;
  int mask_sign;
  unsigned int range; //! Seek range - on both sides of the reference pixel
  double sample_step; //! Distance between sampled points in pixels
  int ntotal_sample;
  int points_to_track;
  //! Convolution masks' size in pixels (masks are square), \warning should
  //! not be public, use setMaskSize() and getMaskSize() instead (kept public
  //! for compatibility reasons).
  unsigned int mask_size;
  //! The number of convolution masks available for tracking ; defines
  //! resolution. \warning Should not be public, use setMaskNumber() and
  //! getMaskNumber() instead (kept public for compatibility reasons).
  unsigned int n_mask;
  // Strip: defines a "security strip" such that Seek_Extremities()
  // cannot return a new extremity which is too close to the
  // frame borders which may cause Get_Sampling_Grid to refuse
  // the that extremity
  int strip;
  vpMatrix *mask; //! Array of matrices defining the different masks (one for every angle step).

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

    \return Value of the likelihood threshold.

    \sa setThreshold(), getLikelihoodThresholdType(), setLikelihoodThresholdType()
  */
  inline double getThreshold() const { return threshold; }

  /*!
    Return the selected choice for the likelihood threshold.

    \return The likelihood threshold type to consider.

    \sa setLikelihoodThresholdType(), setThreshold(), getThreshold()
  */
  inline vpLikelihoodThresholdType getLikelihoodThresholdType() const { return m_likelihood_threshold_type; }

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

    \param t : Threshold to consider. Two different cases need to be considered depending on the likelihood threshold type that
    can be set using setLikelihoodThresholdType() or get using getLikelihoodThresholdType(). The default likelihood threshold type
    is set to OLD_THRESHOLD to keep compatibility with ViSP previous releases, but it is recommended to use rather the NORMALIZED_THRESHOLD
    type like in the following sample code. When doing so, the threshold is more easy to set since it corresponds to the minimal luminance
    contrast to consider with values in range [0 ; 255].

    \code
    vpMe me;
    me.setLikelihoodThresholdType(NORMALIZED_THRESHOLD);
    me.setThreshold(20); // Value in range [0 ; 255]
    \endcode

    When the likelihood threshold type is set by default to OLD_THRESHOLD like in the next example, values of the likelihood threshold
    depends on the minimal luminance contrast to consider and the mask size that can be set using setMaskSize() and retrieved using getMaskSize().
    \code
    vpMe me;                // By default the constructor set the threshold type to OLD_THRESHOLD
    me.setThreshold(10000); // Value that depends on the minimal luminance contrast to consider and the mask size.
    \endcode
    The previous sample code is similar to the next one:
    \code
    vpMe me;
    me.setLikelihoodThresholdType(OLD_THRESHOLD);
    me.setThreshold(10000); // Value that depends on the minimal luminance contrast to consider and the mask size.
    \endcode
    \sa getThreshold(), getLikelihoodThresholdType()
  */
  void setThreshold(const double &t) { threshold = t; }

  /*!
    Set the likelihood threshold type used to determine if the moving edge is valid or not.

    \param likelihood_threshold_type : Likelihood threshold type. It is recommended to use NORMALIZED_THRESHOLD and set the threshold
    using setThreshold() with a value corresponding to the minimal luminance contrast to consider that can handle values in range [0 ; 255].

    \sa setThreshold()
  */
  void setLikelihoodThresholdType(const vpLikelihoodThresholdType likelihood_threshold_type) { m_likelihood_threshold_type = likelihood_threshold_type; }

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
    {"thresholdType", me.getLikelihoodThresholdType()},
    {"threshold", me.getThreshold()},
    {"mu", {me.getMu1(), me.getMu2()}},
    {"minSampleStep", me.getMinSampleStep()},
    {"sampleStep", me.getSampleStep()},
    {"range", me.getRange()},
    {"ntotalSample", me.getNbTotalSample()},
    {"pointsToTrack", me.getPointsToTrack()},
    {"maskSize", me.getMaskSize()},
    {"nMask", me.getMaskNumber()},
    {"maskSign", me.getMaskSign()},
    {"strip", me.getStrip()}
  };
}

/**
 * @brief Retrieve a vpMe object from a JSON representation
 *
 * JSON content (key: type):
 *  - thresholdType: int, vpMe::getLikelihoodThresholdType()
 *  - threshold: double, vpMe::setThreshold()
 *  - mu : [double, double], vpMe::setMu1, vpMe::setMu2()
 *  - minSampleStep: double, vpMe::setMinSampleStep()
 *  - angleStep: double, vpMe::setAngleStep()
 *  - sampleStep: double, vpMe::setSampleStep()
 *  - range: int, vpMe::setRange()
 *  - ntotal_sample: int, vpMe::setNbTotalSample()
 *  - pointsToTrack: int, vpMe::setPointsToTrack()
 *  - maskSize: int, vpMe::setMaskSize()
 *  - nMask: int, vpMe::setMaskNumber()
 *  - maskSign: int, vpMe::setMaskSign()
 *  - strip: int, vpMe::setStrip()
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
    "thresholdType": 1
    "threshold": 20.0
  }
 * \endcode
 *
 * @param j JSON representation to convert
 * @param me converted object
 */
inline void from_json(const nlohmann::json &j, vpMe &me)
{
  if (j.contains("thresholdType")) {
    me.setLikelihoodThresholdType(j.value("thresholdType", me.getLikelihoodThresholdType()));
  }
  me.setThreshold(j.value("threshold", me.getThreshold()));

  if (j.contains("mu")) {
    std::vector<double> mus = j.at("mu").get<std::vector<double>>();
    assert((mus.size() == 2));
    me.setMu1(mus[0]);
    me.setMu2(mus[1]);
  }
  me.setMinSampleStep(j.value("minSampleStep", me.getMinSampleStep()));

  me.setRange(j.value("range", me.getRange()));
  me.setNbTotalSample(j.value("ntotalSample", me.getNbTotalSample()));
  me.setPointsToTrack(j.value("pointsToTrack", me.getPointsToTrack()));
  me.setMaskSize(j.value("maskSize", me.getMaskSize()));
  me.setMaskSign(j.value("maskSign", me.getMaskSign()));
  me.setStrip(j.value("strip", me.getStrip()));
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
