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
  \file vpRBProbabilistic3DDriftDetector.h
  \brief Drift detection through 3D point statistical description for the render-based tracker
*/

#ifndef VP_RB_PROBABILISTIC_3D_DRIFT_DETECTOR_H
#define VP_RB_PROBABILISTIC_3D_DRIFT_DETECTOR_H

#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpRGBf.h>
#include <visp3/core/vpRGBa.h>

#include <visp3/rbt/vpRBDriftDetector.h>

#include <array>

BEGIN_VISP_NAMESPACE

template <typename T> class vpImage;

/**
 * \ingroup group_rbt_drift
 *
 * \brief Algorithm that uses tracks object surface points in order to estimate the probability that tracking is
 * successful.
 *
 * Given a set of surface points \f$ \mathbf{X}_0, ..., \mathbf{X}_N\f$, each point \f$\mathbf{X}_i\f$ being associated
 * to:
 *
 * - a color distribution \f$\mathcal{N}(\mathbf{\bar c_i}, \mathbf{\Sigma_{c_i}^2})\f$,
 * - its distance to the camera being \f$Z_i\f$,
 * - its projection in the current color and depth images \f$\mathbf{I_c}, \mathbf{I_Z}\f$ having coordinates
 *   \f$u_i, v_i\f$.
 * - its visibility \f$V(\mathbf{X_i})\f$, which is 1 if \f$u_i, v_i\f$ lie in the image,
 *   \f$Z_i\f$ is close to the rendered depth value
 *   and the normal at the surface marks the point as visible from the camera's point of view.
 *
 * We compute the probability that tracking is successful for a given pose \f$^{c}\mathbf{T}_o\f$ as:
 *
 * \f[ p(^{c}\mathbf{T}_o) = \frac{1}{\sum_{i=0}^N w_i \cdot V(\mathbf{X_i})}\sum_{i=0}^N w_i \cdot V(\mathbf{X_i}) \cdot p(\mathbf{X_i}) \f]
 *
 * with \f[
 * \begin{aligned}
 * p(\mathbf{X_i}) &=  p(\mathbf{I_c}(u_i, v_i)|\mathcal{N}(\mathbf{\bar c_i}, \mathbf{\Sigma_{c_i}^2})) \cdot p(\mathbf{I_Z}(u_i, v_i) | \mathcal{N}(Z_i, \sigma_Z^2)) \\
 * p(\mathbf{I_c}(u_i, v_i) | \mathcal{N}(\mathbf{\bar c_i}, \mathbf{\Sigma_{c_i}^2})) &= erfc(\frac{1}{\sqrt{2}}\lVert \frac{\mathbf{I_c}(u_i, v_i) - \mathbf{\bar c_i}}{diag(\mathbf{\Sigma_{c_i}})} \rVert_2) \\
 * p(\mathbf{I_Z}(u_i, v_i) | \mathcal{N}(Z_i, \sigma_Z^2)) &= erfc(\frac{1}{\sigma_Z \sqrt{2}}\mathbf{I_Z}(u_i, v_i) - Z_i)
 * \end{aligned}
 * \f]
 *
 * if the depth is unavailable, then we set \f$p(\mathbf{I_Z}(u_i, v_i) | \mathcal{N}(Z_i, \sigma_Z^2)) = 1\f$
 *
 * Here, the color distribution is estimated online for each point separately using exponential moving average/variance
 * techniques. For each point the update step is computed as
 * \f$p(\mathbf{I_Z}(u_i, v_i) | \mathcal{N}(Z_i, \sigma_Z^2))\cdot \alpha\f$ where \f$\alpha\f$ is a fixed parameter.
 * Larger values will lead to faster update rates and may be more beneficial for non lambertian materials.
 *
 * For the depth, \f$\sigma_Z\f$ is a fixed parameter to be tweaked by the user.
 *
 * Every time update() is called, the set of points \f$ \mathbf{X}_0, ..., \mathbf{X}_N, \f$ may grow larger.
 * If a new candidate point is visible and is far enough from points already in the set, it is added to it.
 */
class VISP_EXPORT vpRBProbabilistic3DDriftDetector : public vpRBDriftDetector
{
private:

  struct vpStored3DSurfaceColorPoint
  {

    /**
     * \brief Online estimation of a Gaussian color distribution
     * \f$\mathcal{N}(\mathbf{\bar c}, \mathbf{\Sigma_c^2})\f$, Where \f$\mathbf{\Sigma_c^2}\f$ is a diagonal variance
     * matrix \f$diag(\sigma_r^2, \sigma_g^2, \sigma_b^2)\f$.
     *
     * This class uses exponential moving average and variance estimation to approximage the distribution of the
     * different RGB color components.
     *
     * It does not estimate the full covariance matrix, but rather the variance of the individual RGB components.
     */
    struct ColorStatistics
    {
      ColorStatistics() = default;

      void init(const vpRGBf &c, const vpRGBf &var)
      {
        mean = c;
        variance = var;
        computeStddev();
      }

      /**
       * \brief Update the color distribution with a new sample c.
       *
       * \param c
       * \param weight The importance of c (between 0 and 1) in the distribution update (see Exponential moving
       * average). A high value prioritizes the last seen values.
       */
      void update(const vpRGBf &c, float weight)
      {
        const vpRGBf diff(c.R - mean.R, c.G - mean.G, c.B - mean.B);
        vpRGBf diffSqr(std::pow(diff.R, 2), std::pow(diff.G, 2), std::pow(diff.B, 2));
        mean = mean + weight * diff;
        variance = variance + weight * diffSqr;
        computeStddev();
      }

      /**
       * \brief Computes the probability that the input color was sampled from the estimated distribution.
       *
       * \param c
       * \return the probability \f$ p(\mathbf{c} | \mathcal{N}(\mathbf{\bar c}, \mathbf{\Sigma_c})) \f$
       */
      double probability(const vpRGBf &c)
      {

        const double dist = sqrt(
          std::pow((c.R - mean.R) / (standardDev.R), 2) +
          std::pow((c.G - mean.G) / (standardDev.G), 2) +
          std::pow((c.B - mean.B) / (standardDev.B), 2));

        const double proba = 1.0 - erf(dist / sqrt(2));

        return proba;
      }

      double trace()
      {
        return static_cast<double>(variance.R + variance.G + variance.B);
      }

      void computeStddev()
      {
        standardDev.R = sqrt(variance.R);
        standardDev.G = sqrt(variance.G);
        standardDev.B = sqrt(variance.B);
      }

      vpRGBf mean;
      vpRGBf variance;
      vpRGBf standardDev;
    };

    inline void update(const vpHomogeneousMatrix &cTo, const vpHomogeneousMatrix &cprevTo, const vpHomogeneousMatrix &renderTo, const vpCameraParameters &cam)
    {
      fastProjection(cTo, cam, currX, projCurr, projCurrPx);
      fastProjection(cprevTo, cam, prevX, projPrev, projPrevPx);
      fastProjection(renderTo, cam, renderX, projRender, projRenderPx);
    }

    inline double squaredDist(const std::array<double, 3> &p) const
    {
      return std::pow(p[0] - X[0], 2) + std::pow(p[1] - X[1], 2) + std::pow(p[2] - X[2], 2);
    }

    inline void fastProjection(const vpHomogeneousMatrix &cTo, const vpCameraParameters &cam,
      std::array<double, 3> &pC, std::array<double, 2> &proj, std::array<int, 2> &px)
    {
      const double *T = cTo.data;
      pC[0] = (T[0] * X[0] + T[1] * X[1] + T[2] * X[2] + T[3]);
      pC[1] = (T[4] * X[0] + T[5] * X[1] + T[6] * X[2] + T[7]);
      pC[2] = (T[8] * X[0] + T[9] * X[1] + T[10] * X[2] + T[11]);
      proj[0] = pC[0] / pC[2];
      proj[1] = pC[1] / pC[2];
      px[0] = static_cast<int>((proj[0] * cam.get_px()) + cam.get_u0());
      px[1] = static_cast<int>((proj[1] * cam.get_py()) + cam.get_v0());
    }

    void updateColor(const vpRGBf &currentColor, float updateRate)
    {
      stats.update(currentColor, updateRate);
    }

    vpRGBa getDisplayColor() const
    {
      return vpRGBa(static_cast<unsigned int>(stats.mean.R), static_cast<unsigned int>(stats.mean.G), static_cast<unsigned int>(stats.mean.B));
    }

    std::array<double, 3> X; // Point position in object frame
    ColorStatistics stats; //! Color statistics associated to this point
    std::array<double, 3> currX, prevX, renderX; //! Point position in the current and previous camera frames
    std::array<double, 2> projCurr, projPrev, projRender; // Projection in camera normalized coordinates of the point for the current and previous camera poses.
    std::array<int, 2> projCurrPx, projPrevPx, projRenderPx; // Projection in pixels of the point for the current and previous camera poses.
    bool visible; // Whether the point is visible
  };

public:

  vpRBProbabilistic3DDriftDetector() : m_colorUpdateRate(0.2), m_initialColorSigma(25.0), m_depthSigma(0.04), m_maxError3D(0.001), m_minDist3DNewPoint(0.003), m_sampleStep(4)
  { }

  void update(const vpRBFeatureTrackerInput &previousFrame, const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cTo, const vpHomogeneousMatrix &cprevTo) VP_OVERRIDE;

  /**
   * \brief Returns the probability [0, 1] that tracking is successful.
   *
   */
  double getScore() const VP_OVERRIDE;

  bool hasDiverged() const VP_OVERRIDE;

  void display(const vpImage<vpRGBa> &I) VP_OVERRIDE;

  /**
   * \name Settings
   * @{
   */

  /**
   * \brief Get the minimum distance criterion (in meters) that is used
   * when trying to add new points to track for the drift detection.
   *
   * A candidate surface point is compared to all the currently tracked surface point and
   * if any of these points is below the minimum distance, the candidate is rejected.
   *
   * \return the minimum distance, in meters
   */
  double getMinDistForNew3DPoints() const { return m_minDist3DNewPoint; }

  void setMinDistForNew3DPoints(double distance)
  {
    if (distance <= 0.0) {
      throw vpException(vpException::badValue, "Distance criterion for candidate rejection should be greater than 0.");
    }
    m_minDist3DNewPoint = distance;
  }

  /**
   * \brief Returns the maximum 3D distance (in meters) above which a tracked surface point is rejected for the drift
   * estimation step.
   *
   * The surface point's distance to the camera is compared to rendered depth. If the difference between the two is
   * too great, it is rejected.
   *
   * This is mainly used to handle self occlusions.
   */
  double getFilteringMax3DError() const { return m_maxError3D; }

  void setFilteringMax3DError(double maxError)
  {
    if (maxError <= 0.0) {
      throw vpException(vpException::badValue, "Maximum 3D error for rejection should be greater than 0.");
    }
    m_maxError3D = maxError;
  }

  /**
   * \brief Get the standard deviation that is used when computing
   *  the probability that the observed depth Z is the correct depth given the rendered depth at the same pixel.
   *
   */
  double getDepthStandardDeviation() const { return m_depthSigma; }
  void setDepthStandardDeviation(double sigma)
  {
    if (sigma < 0.0) {
      throw vpException(vpException::badValue, "Depth standard deviation should be greater than 0");
    }
    m_depthSigma = sigma;
  }

  /**
   * \brief Get the standard deviation that is used to initialize the color distribution when adding a new surface point.
   * This standard deviation is applied on all color components.
   *
   */
  double getInitialColorStandardDeviation() const { return m_depthSigma; }
  void setInitialColorStandardDeviation(double sigma)
  {
    if (sigma < 0.0) {
      throw vpException(vpException::badValue, "Initial color standard deviation should be greater than 0");
    }
    m_initialColorSigma = sigma;
  }

  /**
   * \brief Get the rate at which the colors of surface points are updated.
   *
   * Note that if depth is available, this component is further multiplied by the probability of depth being correct
   * for a given point.
   *
   * A high value will lead to a fast update rate (short term memory), while a lower one will update slower.
   * A slower update may lead to a more stable tracking score. A higher value may be better suited to non isotropic
   * materials.
   */
  double getColorUpdateRate() const { return m_colorUpdateRate; }

  /**
   * \brief Set the update rate for the color distribution. It should be between 0 and 1.
   *
   * \param updateRate the update rate
   */
  void setColorUpdateRate(double updateRate)
  {
    if (updateRate < 0.0 || updateRate > 1.f) {
      throw vpException(vpException::badValue, "Color update rate should be between 0 and 1");
    }
    m_colorUpdateRate = updateRate;
  }

  unsigned int getSampleStep() const { return m_sampleStep; }
  void setSampleStep(unsigned int sampleStep)
  {
    if (sampleStep == 0) {
      throw vpException(vpException::badValue, "Image sample step should be greater than 0");
    }
    m_sampleStep = sampleStep;
  }

#if defined(VISP_HAVE_NLOHMANN_JSON)
  void loadJsonConfiguration(const nlohmann::json &) VP_OVERRIDE;
#endif

/**
 * @}
 * End settings
 */

private:
  double m_colorUpdateRate;
  double m_initialColorSigma;
  double m_depthSigma;
  double m_maxError3D;
  double m_minDist3DNewPoint;
  unsigned int m_sampleStep;

  double m_score;

  std::vector<vpStored3DSurfaceColorPoint> m_points;
};

END_VISP_NAMESPACE

#endif
