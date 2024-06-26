/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 */

/*!
 * \file vpMe.h
 * \brief Moving edges
 */

#ifndef VP_ME_H
#define VP_ME_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE
/*!
 * \class vpMe
 * \ingroup module_me
 *
 * This class defines predetermined masks for sites and holds moving edges
 * tracking parameters.
 *
 * <b>JSON serialization</b>
 *
 * Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpMe.
 * The following sample code shows how to save moving-edges settings in a file named `me.json`
 * and reload the values from this JSON file.
 * \code
 * #include <visp3/me/vpMe.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined(VISP_HAVE_NLOHMANN_JSON)
 *   std::string filename = "me.json";
 *   {
 *     vpMe me;
 *     me.setLikelihoodThresholdType(vpMe::NORMALIZED_THRESHOLD);
 *     me.setThreshold(20);    // Value in range [0 ; 255]
 *     me.setThresholdMarginRatio(-1.); // Deactivate automatic thresholding
 *     me.setMinThreshold(-1.); // Deactivate automatic thresholding
 *     me.setMaskNumber(180);
 *     me.setMaskSign(0);
 *     me.setMu1(0.5);
 *     me.setMu2(0.5);
 *     me.setNbTotalSample(0);
 *     me.setPointsToTrack(200);
 *     me.setRange(5);
 *     me.setStrip(2);
 *
 *     std::ofstream file(filename);
 *     const nlohmann::json j = me;
 *     file << j;
 *     file.close();
 *   }
 *   {
 *     std::ifstream file(filename);
 *     const nlohmann::json j = nlohmann::json::parse(file);
 *     vpMe me;
 *     me = j;
 *     file.close();
 *     std::cout << "Read moving-edges settings from " << filename << ":" << std::endl;
 *     me.print();
 *   }
 * #endif
 * }
 * \endcode
 * If you build and execute the sample code, it will produce the following output:
 * \code{.unparsed}
 * Read moving-edges settings from me.json:
 *
 * Moving edges settings
 *
 *  Size of the convolution masks....5x5 pixels
 *  Number of masks..................180
 *  Query range +/- J................5 pixels
 *  Likelihood threshold type........normalized
 *  Likelihood threshold.............20
 *  Likelihood margin ratio..........unused
 *  Minimum likelihood threshold.....unused
 *  Contrast tolerance +/-...........50% and 50%
 *  Sample step......................10 pixels
 *  Strip............................2 pixels
 *  Min sample step..................4 pixels
 * \endcode
 *
 * The content of the `me.json` file is the following:
 * \code{.unparsed}
 * $ cat me.json
 * {"maskSign":0,"maskSize":5,"minSampleStep":4.0,"mu":[0.5,0.5],"nMask":180,"ntotalSample":0,"pointsToTrack":200,
 *  "range":5,"sampleStep":10.0,"strip":2,"threshold":20.0,"thresholdMarginRatio":-1.0,"minThreshold":-1.0,"thresholdType":"normalized"}
 * \endcode
*/
class VISP_EXPORT vpMe
{
public:
  /*!
   * Type of likelihood threshold to use.
   */
  typedef enum
  {
    //! Old likelihood ratio threshold (to be avoided).
    OLD_THRESHOLD = 0,
    //! Easy-to-use normalized likelihood threshold corresponding to the minimal luminance contrast to consider
    //! with values in [0 ; 255].
    NORMALIZED_THRESHOLD = 1
  } vpLikelihoodThresholdType;

public:
  /*!
   * Default constructor.
   */
  vpMe();

  /*!
   * Copy constructor.
   */
  vpMe(const vpMe &me);

  /*!
   * Destructor.
   */
  virtual ~vpMe();

  /*!
   * Copy operator.
   */
  vpMe &operator=(const vpMe &me);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  /*!
   * Move operator.
   */
  vpMe &operator=(const vpMe &&me);
#endif

  /*!
   * Check sample step wrt min value.
   * \param[inout] sample_step : When this value is lower than the min sample step value,
   * it is modified to the min sample step value.
   */
  void checkSamplestep(double &sample_step)
  {
    if (sample_step < m_min_samplestep) {
      sample_step = m_min_samplestep;
    }
  }

  /*!
   * Return the angle step.
   *
   * \return Value of angle step.
   */
  inline unsigned int getAngleStep() const { return m_anglestep; }

  /*!
   * Get the matrix of the mask.
   *
   * \return the value of mask.
   */
  inline vpMatrix *getMask() const { return m_mask; }

  /*!
   * Return the number of mask  applied to determine the object contour. The
   * number of mask determines the precision of the normal of the edge for
   * every sample. If precision is 2deg, then there are 360/2 = 180 masks.
   *
   * \return the current number of mask.
   */
  inline unsigned int getMaskNumber() const { return m_mask_number; }

  /*!
   * Return the mask sign.
   *
   * \return Value of mask_sign.
   */
  inline int getMaskSign() const { return m_mask_sign; }

  /*!
   * Return the actual mask size (in pixel) used to compute the image gradient
   * and determine the object contour. The mask size defines the size of the
   * convolution mask used to detect an edge.
   *
   * \return the current mask size.
   */
  inline unsigned int getMaskSize() const { return m_mask_size; }

  /*!
   * Get the minimum allowed sample step. Useful to specify a lower bound when
   * the sample step is changed.
   *
   * \return Value of min sample step.
   */
  inline double getMinSampleStep() const { return m_min_samplestep; }

  /*!
   * Get the minimum image contrast allowed to detect a contour.
   *
   * \return Value of mu1.
   */
  inline double getMu1() const { return m_mu1; }

  /*!
   * Get the maximum image contrast allowed to detect a contour.
   *
   * \return Value of mu2.
   */
  inline double getMu2() const { return m_mu2; }

  /*!
   * Get how many discretized points are used to track the feature.
   *
   * \return Value of ntotal_sample.
   */
  inline int getNbTotalSample() const { return m_ntotal_sample; }

  /*!
   * Return the number of points to track.
   *
   * \return Value of points_to_track.
   */
  inline int getPointsToTrack() const { return m_points_to_track; }

  /*!
   * Return the seek range on both sides of the reference pixel.
   *
   * \return Value of range.
   */
  inline unsigned int getRange() const { return m_range; }

  /*!
   * Get the minimum distance in pixel between two discretized points.
   *
   * \return Value of sample_step.
   */
  inline double getSampleStep() const { return m_sample_step; }

  /*!
   * Get the number of pixels that are ignored around the image borders.
   *
   * \return the value of strip.
   */
  inline int getStrip() const { return m_strip; }

  /*!
   * Return the likelihood threshold used to determine if the moving edge is valid or not.
   *
   * \return Value of the likelihood threshold.
   *
   * \sa setThreshold(), getLikelihoodThresholdType(), setLikelihoodThresholdType()
   */
  inline double getThreshold() const { return m_threshold; }

  /*!
   * Return the ratio of the initial contrast to use to initialize the contrast threshold of the \b vpMeSite.
   *
   * \return Value of the likelihood threshold ratio, between 0 and 1.
   *
   * \sa setThresholdMarginRatio(), setMinThreshold(), getMinThreshold(), getLikelihoodThresholdType(), setLikelihoodThresholdType()
   */
  inline double getThresholdMarginRatio() const { return m_thresholdMarginRatio; }

  /*!
   * Return the minimum contrast threshold of the \b vpMeSite that can be used when using the
   * automatic threshold computation.
   *
   * \return Value of the minimum contrast threshold.
   *
   * \sa setThresholdMarginRatio(), getThresholdMarginRatio(), setMinThreshold(), getLikelihoodThresholdType(), setLikelihoodThresholdType()
   */
  inline double getMinThreshold() const { return m_minThreshold; }

  /*!
   * \brief Indicates if the contrast threshold of the vpMeSite is automatically computed.
   *
   * \return true The contrast threshold of the vpMeSite is automatically computed.
   * \return false The vpMe::m_threshold is used as a global threshold.
   */
  inline bool getUseAutomaticThreshold() const { return m_useAutomaticThreshold; }

  /*!
   * Return the selected choice for the likelihood threshold.
   *
   * \return The likelihood threshold type to consider.
   *
   * \sa setLikelihoodThresholdType(), setThreshold(), getThreshold()
   */
  inline vpLikelihoodThresholdType getLikelihoodThresholdType() const { return m_likelihood_threshold_type; }

  /*!
   * Initialise the array of matrices with the defined size and the number of
   * matrices to create.
   */
  void initMask(); // convolution masks - offset computation

  /*!
   * Print using std::cout moving edges settings.
   */
  void print();

  /*!
   * Set the angle step.
   *
   * \param anglestep : New angle step value.
   */
  void setAngleStep(const unsigned int &anglestep) { m_anglestep = anglestep; }

  /*!
   * Set the number of mask applied to determine the object contour. The number
   * of mask determines the precision of the normal of the edge for every
   * sample. If precision is 2deg, then there are 360/2 = 180 masks.
   *
   * \param mask_number : The number of mask.
   */
  void setMaskNumber(const unsigned int &mask_number);

  /*!
   * Set the mask sign.
   *
   * \param mask_sign : New mask sign.
   */
  void setMaskSign(const int &mask_sign) { m_mask_sign = mask_sign; }

  /*!
   * Set the mask size (in pixel) used to compute the image gradient and
   * determine the object contour. The mask size defines the size of the
   * convolution mask used to detect an edge.
   *
   * \param mask_size : New mask size.
   */
  void setMaskSize(const unsigned int &mask_size);

  /*!
   * Set the minimum allowed sample step. Useful to specify a lower bound when
   * the sample step is changed.
   *
   * \param min_samplestep : New minimum sample step.
   */
  void setMinSampleStep(const double &min_samplestep) { m_min_samplestep = min_samplestep; }

  /*!
   * Set the minimum image contrast allowed to detect a contour.
   *
   * \param mu_1 : new mu1.
   */
  void setMu1(const double &mu_1) { this->m_mu1 = mu_1; }

  /*!
   * Set the maximum image contrast allowed to detect a contour.
   *
   * \param mu_2 : New mu2.
   */
  void setMu2(const double &mu_2) { this->m_mu2 = mu_2; }

  /*!
   * Set how many discretized points are used to track the feature.
   *
   * \param ntotal_sample : New total number of sample.
   */
  void setNbTotalSample(const int &ntotal_sample) { m_ntotal_sample = ntotal_sample; }

  /*!
   * Set the number of points to track.
   *
   * \param points_to_track : New number of points to track.
   *
   * \warning This method is useful only for the vpMeNurbsTracker.
   */
  void setPointsToTrack(const int &points_to_track) { m_points_to_track = points_to_track; }

  /*!
   * Set the seek range on both sides of the reference pixel.
   *
   * \param range : New range.
   */
  void setRange(const unsigned int &range) { m_range = range; }

  /*!
   * Set the minimum distance in pixel between two discretized points.
   *
   * \param sample_step : New sample_step.
   */
  void setSampleStep(const double &sample_step) { m_sample_step = sample_step; }

  /*!
   * Set the number of pixels that are ignored around the image borders.
   *
   * \param strip : New strip.
   */
  void setStrip(const int &strip) { m_strip = strip; }

  /*!
   * Set the likelihood threshold used to determined if the moving edge is valid or not.
   *
   * \param threshold : Threshold to consider. Two different cases need to be considered depending on the likelihood threshold type that
   * can be set using setLikelihoodThresholdType() or get using getLikelihoodThresholdType(). The default likelihood threshold type
   * is set to OLD_THRESHOLD to keep compatibility with ViSP previous releases, but it is recommended to use rather the NORMALIZED_THRESHOLD
   * type like in the following sample code. When doing so, the threshold is more easy to set since it corresponds to the minimal luminance
   * contrast to consider with values in range [0 ; 255].
   *
   * \code
   * vpMe me;
   * me.setLikelihoodThresholdType(NORMALIZED_THRESHOLD);
   * me.setThreshold(20); // Value in range [0 ; 255]
   * me.setThresholdMarginRatio(-1.); // Deactivate automatic thresholding
   * me.setMinThreshold(-1.); // Deactivate automatic thresholding
   * \endcode
   *
   * When the likelihood threshold type is set by default to OLD_THRESHOLD like in the next example, values of the likelihood threshold
   * depends on the minimal luminance contrast to consider and the mask size that can be set using setMaskSize() and retrieved using getMaskSize().
   * \code
   * vpMe me;                // By default the constructor set the threshold type to OLD_THRESHOLD
   * me.setThreshold(10000); // Value that depends on the minimal luminance contrast to consider and the mask size.
   * me.setThresholdMarginRatio(-1.); // Deactivate automatic thresholding
   * me.setMinThreshold(-1.); // Deactivate automatic thresholding
   * \endcode
   * The previous sample code is similar to the next one:
   * \code
   * vpMe me;
   * me.setLikelihoodThresholdType(OLD_THRESHOLD);
   * me.setThreshold(10000); // Value that depends on the minimal luminance contrast to consider and the mask size.
   * me.setThresholdMarginRatio(-1.); // Deactivate automatic thresholding
   * me.setMinThreshold(-1.); // Deactivate automatic thresholding
   * \endcode
   * \sa getThreshold(), getLikelihoodThresholdType()
   */
  void setThreshold(const double &threshold) { m_threshold = threshold; }

  /*!
   * Set the the ratio of the initial contrast to use to initialize the contrast threshold of the \b vpMeSite.
   *
   * \param thresholdMarginRatio Value of the likelihood threshold ratio, between 0 and 1.
   *
   * \sa getThresholdMarginRatio(), setMinThreshold(), getMinThreshold(), getLikelihoodThresholdType(), setLikelihoodThresholdType()
   */
  inline void setThresholdMarginRatio(const double &thresholdMarginRatio)
  {
    if (thresholdMarginRatio > 1.) {
      throw(vpException(vpException::badValue, "Threshold margin ratio must be between 0 and 1 if you want to use automatic threshold computation, or negative otherwise"));
    }
    m_thresholdMarginRatio = thresholdMarginRatio;
    m_useAutomaticThreshold = (m_thresholdMarginRatio > 0) && (m_minThreshold > 0);
  }

  /*!
   * Set the minimum value of the contrast threshold of the \b vpMeSite.
   *
   * \param minThreshold Minimum value of the contrast threshold.
   *
   * \sa getMinThreshold(), setThresholdMarginRatio(), getThresholdMarginRatio(), getLikelihoodThresholdType(), setLikelihoodThresholdType()
   */
  inline void setMinThreshold(const double &minThreshold)
  {
    m_minThreshold = minThreshold;
    m_useAutomaticThreshold = (m_thresholdMarginRatio > 0) && (m_minThreshold > 0);
  }

  /*!
   * Set the likelihood threshold type used to determine if the moving edge is valid or not.
   *
   * \param likelihood_threshold_type : Likelihood threshold type. It is recommended to use NORMALIZED_THRESHOLD and set the threshold
   * using setThreshold() with a value corresponding to the minimal luminance contrast to consider that can handle values in range [0 ; 255].
   *
   * \sa setThreshold()
   */
  void setLikelihoodThresholdType(const vpLikelihoodThresholdType likelihood_threshold_type) { m_likelihood_threshold_type = likelihood_threshold_type; }

private:
  vpLikelihoodThresholdType m_likelihood_threshold_type; //!< Likelihood threshold type
  //! Old likelihood ratio threshold (to be avoided) or easy-to-use normalized threshold: minimal contrast
  double m_threshold;
  double m_thresholdMarginRatio; //!< The ratio of the initial contrast to use to initialize the contrast threshold of the vpMeSite.
  double m_minThreshold; //!< The minimum moving-edge threshold in grey level used when the contrast threshold of the vpMeSites is automatically computed.
  bool m_useAutomaticThreshold; //!< Set to true if the user wants to automatically compute the vpMeSite contrast thresholds, false if the user wants to use a global threshold.
  double m_mu1;       //!< Contrast continuity parameter (left boundary)
  double m_mu2;       //!< Contrast continuity parameter (right boundary)
  double m_min_samplestep;
  unsigned int m_anglestep;
  int m_mask_sign;
  unsigned int m_range; //! Seek range - on both sides of the reference pixel
  double m_sample_step; //! Distance between sampled points in pixels
  int m_ntotal_sample;
  int m_points_to_track; //!< Expected number of points to track
  //! Convolution masks' size in pixels (masks are square)
  unsigned int m_mask_size;
  //! The number of convolution masks available for tracking ; defines resolution
  unsigned int m_mask_number;
  //! Strip: defines a "security strip" such that when seeking extremities
  //! cannot return a new extremity which is too close to the frame borders
  int m_strip;
  vpMatrix *m_mask; //!< Array of matrices defining the different masks (one for every angle step).

#ifdef VISP_HAVE_NLOHMANN_JSON
  /*!
   * @brief Convert a vpMe object to a JSON representation.
   *
   * @param j : Resulting json object.
   * @param me : The object to convert.
   */
  friend void to_json(nlohmann::json &j, const vpMe &me);

  /**
   * @brief Retrieve a vpMe object from a JSON representation
   *
   * JSON content (key: type):
   *  - thresholdType: either "old" or "normalized", vpMe::getLikelihoodThresholdType()
   *  - threshold: double, vpMe::setThreshold()
   *  - thresholdMarginRatio: double, vpMe::setThresholdMarginRatio()
   *  - minThreshold: double, vpMe::setMinThreshold()
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
   *   "maskSign": 0,
   *   "maskSize": 5,
   *   "minSampleStep": 4.0,
   *   "mu": [
   *       0.5,
   *       0.5
   *   ],
   *   "nMask": 180,
   *   "ntotal_sample": 0,
   *   "pointsToTrack": 500,
   *   "range": 7,
   *   "sampleStep": 4.0,
   *   "strip": 2,
   *   "thresholdType": "normalized",
   *   "threshold": 20.0,
   *   "thresholdMarginRatio": 0.75,
   *   "minThreshold": 20.0,
   * }
   * \endcode
   *
   * @param j JSON representation to convert
   * @param me converted object
   */
  friend void from_json(const nlohmann::json &j, vpMe &me);
#endif
};

#ifdef VISP_HAVE_NLOHMANN_JSON
NLOHMANN_JSON_SERIALIZE_ENUM(vpMe::vpLikelihoodThresholdType, {
  {vpMe::vpLikelihoodThresholdType::OLD_THRESHOLD, "old"},
  {vpMe::vpLikelihoodThresholdType::NORMALIZED_THRESHOLD, "normalized"}
});

inline void to_json(nlohmann::json &j, const vpMe &me)
{
  j = {
    {"thresholdType", me.getLikelihoodThresholdType()},
    {"threshold", me.getThreshold()},
    {"thresholdMarginRatio", me.getThresholdMarginRatio()},
    {"minThreshold", me.getMinThreshold()},
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

inline void from_json(const nlohmann::json &j, vpMe &me)
{
  if (j.contains("thresholdType")) {
    me.setLikelihoodThresholdType(j.value("thresholdType", me.getLikelihoodThresholdType()));
  }
  me.setThreshold(j.value("threshold", me.getThreshold()));
  me.setThresholdMarginRatio(j.value("thresholdMarginRatio", me.getThresholdMarginRatio()));
  me.setMinThreshold(j.value("minThreshold", me.getMinThreshold()));

  if (j.contains("mu")) {
    std::vector<double> mus = j.at("mu").get<std::vector<double>>();
    assert((mus.size() == 2));
    me.setMu1(mus[0]);
    me.setMu2(mus[1]);
  }
  me.setMinSampleStep(j.value("minSampleStep", me.getMinSampleStep()));
  me.setSampleStep(j.value("sampleStep", me.getSampleStep()));
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

END_VISP_NAMESPACE
#endif
