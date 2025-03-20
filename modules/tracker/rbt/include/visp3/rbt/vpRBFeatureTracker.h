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
 */

/*!
  \file vpRBFeatureTracker.h
  \brief Base class for features that can be tracked using vpRBTracker
*/
#ifndef VP_RB_FEATURE_TRACKER_H
#define VP_RB_FEATURE_TRACKER_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpRect.h>

#include <visp3/rbt/vpRBFeatureTrackerInput.h>
#include <visp3/rbt/vpRBSilhouettePoint.h>
#include <visp3/rbt/vpRBJsonParsable.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif


BEGIN_VISP_NAMESPACE
/**
 * \brief A base class for all features that can be used and tracked in the vpRBTracker
 *
 * \ingroup group_rbt_trackers
*/
class VISP_EXPORT vpRBFeatureTracker : public vpRBJsonParsable
{
public:

  vpRBFeatureTracker();

  /**
   * \brief Return the type of feature that is used by this tracker
   *
   * \return vpRBFeatureType
   */

  /**
   * \brief Get the number of features used to compute the pose update
   *
   */
  unsigned getNumFeatures() const { return m_numFeatures; }

  /**
   * \name Required inputs
   * @{
   */

  /**
   * \brief Whether this tracker requires RGB image to extract features
   *
   * \return true if the tracker requires an RGB image
   * \return false otherwise
   */
  virtual bool requiresRGB() const = 0;

  /**
   * \brief Whether this tracker requires depth image to extract features
   *
   */
  virtual bool requiresDepth() const = 0;

  /**
   * \brief Whether this tracker requires Silhouette candidates
   */
  virtual bool requiresSilhouetteCandidates() const = 0;
  /**
   * @}
   */

  /**
   * \name Core Tracking methods
   * @{
   */

  /**
  * \brief Method called when starting a tracking iteration
  *
  */
  virtual void onTrackingIterStart(const vpHomogeneousMatrix &cMo) = 0;

  /**
   * \brief Method called after the tracking iteration has finished
   *
   */
  virtual void onTrackingIterEnd(const vpHomogeneousMatrix &cMo) = 0;

  /**
   * \brief Extract features from the frame data and the current pose estimate
   *
   */
  virtual void extractFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) = 0;

  /**
   * \brief Track the features
   */
  virtual void trackFeatures(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) = 0;

  virtual void initVVS(const vpRBFeatureTrackerInput &frame, const vpRBFeatureTrackerInput &previousFrame, const vpHomogeneousMatrix &cMo) = 0;
  virtual void computeVVSIter(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cMo, unsigned int iteration) = 0;

  /**
   * @}
   */

  /**
   * \name Display
   * @{
   */

  bool featuresShouldBeDisplayed() const { return m_enableDisplay; }
  void setFeaturesShouldBeDisplayed(bool enableDisplay) { m_enableDisplay = enableDisplay; }

  virtual void display(
    const vpCameraParameters &cam,
    const vpImage<unsigned char> &I,
    const vpImage<vpRGBa> &IRGB,
    const vpImage<unsigned char> &depth) const = 0;

  /**
   * @}
   */


  /**
   * \name Covariance computation
   * @{
   */
  /**
   * \brief Retrieve the 6 x 6 pose covariance matrix, computed from the weights associated to each feature.
   *
   * The updateCovariance method should have been called before
   */
  virtual const vpMatrix getCovariance() const { return m_cov; }
  /**
   * \brief Update the covariance matrix
   *
   * \param lambda the visual servoing gain
   */
  virtual void updateCovariance(const double lambda);
  /**
   * @}
   */

  /**
   * Returns whether the tracker is considered as having converged to the desired pose.
   */
  bool vvsHasConverged() const { return m_vvsConverged; }

  /**
   * \brief Get the importance of this tracker in the optimization step.
   *  The default computation is the following:
   * \f$ w / N \f$, where \f$ w\f$ is the weight defined by setTrackerWeight, and  \f$ N \f$ is the number of features.
   */
  virtual double getVVSTrackerWeight() const { return m_userVvsWeight / m_numFeatures; }
  void setTrackerWeight(double weight) { m_userVvsWeight = weight; }

  /**
   * \brief Get the left-side term of the Gauss-Newton optimization term
   */
  virtual vpMatrix getLTL() const { return m_LTL; }

  /**
   * \brief Get the right-side term of the Gauss-Newton optimization term
   */
  virtual vpColVector getLTR() const { return m_LTR; }

  /**
   * \brief Get a weighted version of the error vector.
   * This should not include the userVVSWeight, but may include reweighting to remove outliers, occlusions, etc.
   */
  const vpColVector &getWeightedError() const { return m_weighted_error; }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  virtual void loadJsonConfiguration(const nlohmann::json &j) VP_OVERRIDE
  {
    m_userVvsWeight = j.at("weight");
    m_enableDisplay = j.value("display", m_enableDisplay);
  }
  virtual nlohmann::json explain() const VP_OVERRIDE
  {
    return flipToDict(
      {
        vpRBJsonParsable::parameter("weight", "Feature weight in the optimization step", true, m_userVvsWeight),
        vpRBJsonParsable::parameter("display", "Whether visual feature representations should be displayed when calling vpRBTracker::display()", true, m_enableDisplay),
      }
      );
  }
#endif

  static void computeJTR(const vpMatrix &interaction, const vpColVector &error, vpColVector &JTR);
  static vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &b, const vpMatrix &W);

protected:


  vpMatrix m_L; //! Error jacobian (In VS terms, the interaction matrix)
  vpMatrix m_LTL;  //! Left side of the Gauss newton minimization
  vpColVector m_LTR; //! Right side of the Gauss Newton minimization
  vpMatrix m_cov; //! Covariance matrix
  vpColVector m_covWeightDiag;

  vpColVector m_error; //! Raw VS Error vector
  vpColVector m_weighted_error; //! Weighted VS error
  vpColVector m_weights; //! Error weights


  unsigned m_numFeatures; //! Number of considered features
  double m_userVvsWeight; //! User-defined weight for this specific type of feature

  bool m_vvsConverged; //! Whether VVS has converged, should be updated every VVS iteration

  bool m_enableDisplay; //! Whether the tracked features should be displayed.

};

END_VISP_NAMESPACE

#endif
