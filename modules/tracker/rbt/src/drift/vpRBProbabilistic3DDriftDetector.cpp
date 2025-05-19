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

#include <visp3/rbt/vpRBProbabilistic3DDriftDetector.h>

#include <visp3/core/vpRect.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpDisplay.h>

#include <visp3/rbt/vpRBFeatureTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include VISP_NLOHMANN_JSON(json.hpp)
#endif

BEGIN_VISP_NAMESPACE

double vpRBProbabilistic3DDriftDetector::score(const vpRBFeatureTrackerInput &frame, const vpHomogeneousMatrix &cTo)
{
  double score = 0.0;
  const vpTranslationVector t = frame.renders.cMo.getTranslationVector();
  if (m_points.size() == 0) {
    return 1.0;
  }
    // Step 0: project all points
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
  for (vpStored3DSurfaceColorPoint &p : m_points) {
    p.update(cTo, frame.renders.cMo, frame.cam);
  }




  // Step 1: gather points visible in both images and in render

  std::vector<vpStored3DSurfaceColorPoint *> visiblePoints;
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
  {
    std::vector<vpStored3DSurfaceColorPoint *> visiblePointsLocal;
#ifdef VISP_HAVE_OPENMP
#pragma omp for
#endif
    for (vpStored3DSurfaceColorPoint &p : m_points) {
      p.visible = true;
      if (
        p.projRenderPx[0] < 2 || static_cast<unsigned int>(p.projRenderPx[0]) >= frame.IRGB.getWidth() - 2
      || p.projRenderPx[1] < 2 || static_cast<unsigned int>(p.projRenderPx[1]) >= frame.IRGB.getHeight() - 2
      || p.projCurrPx[0] < 2 || static_cast<unsigned int>(p.projCurrPx[0]) >= frame.IRGB.getWidth() - 2
      || p.projCurrPx[1] < 2 || static_cast<unsigned int>(p.projCurrPx[1]) >= frame.IRGB.getHeight() - 2) {
        p.visible = false; // Point is outside of either current or previous image, ignore it
        continue;
      }

      float ZrenderMap = frame.renders.depth[p.projRenderPx[1]][p.projRenderPx[0]];
      // Version 2: compare previous projection with render, this does not filter occlusions
      if (ZrenderMap == 0.f || fabs(p.renderX[2] - ZrenderMap) > m_maxError3D) {
        p.visible = false;
        continue;
      }
      // Filter occlusions if depth is available
      bool validZMap = frame.hasDepth() && frame.depth[p.projCurrPx[1]][p.projCurrPx[0]] > 0.f;

      if (validZMap) {
        float actualZ = frame.depth[p.projCurrPx[1]][p.projCurrPx[0]];
        // Filter against the specified Z distribution: depth that is too close to the camera wrt to the object render
        // and is not in the Z distribution can be considered as an occlusion
        if (p.currX[2] - actualZ > 3.f * m_depthSigma) {
          p.visible = false;
          continue;
        }
      }

      // vpRGBf normalObject = frame.renders.normals[p.projRenderPx[1]][p.projRenderPx[0]];

      // vpColVector cameraRay({ t[0] - p.X[0], t[1] - p.X[1], t[2] - p.X[2] });

      // cameraRay.normalize();
      // double angle = acos(vpColVector::dotProd(vpColVector({ normalObject.R, normalObject.G, normalObject.B }).normalize(), cameraRay));
      // if (angle > vpMath::rad(85)) {
      //   p.visible = false;
      //   continue;
      // }

      // Filter points that are too close to the silhouette edges
      if (frame.silhouettePoints.size() > 0) {
        for (const vpRBSilhouettePoint &sp: frame.silhouettePoints) {
          if (std::pow(static_cast<double>(sp.i) - p.projRenderPx[1], 2) + std::pow(static_cast<double>(sp.j) - p.projRenderPx[0], 2) < vpMath::sqr(2)) {
            p.visible = false;
            break;
          }
        }
      }
      // Version 3: could be using version 1 and 2. If 1 is wrong but 2 is ok, then there is an issue that is not self occlusion
      // We could reweigh the error by the number of problematic points
      // ...

      if (p.visible) {
        visiblePointsLocal.push_back(&p);
      }
    }
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif
    {
      visiblePoints.insert(visiblePoints.end(), visiblePointsLocal.begin(), visiblePointsLocal.end());
    }
  }
  if (visiblePoints.size() > 0) {
    bool useMedian = false;

    std::vector<double> scores;
    scores.reserve(visiblePoints.size());
    score = 0.0;
    double weightSum = 0.0;

#ifdef VISP_HAVE_OPENMP
#pragma omp parallel
#endif
    {
      std::vector<double> scoresLocal;
      double weightSumLocal = 0.0;
      double scoreLocal = 0.0;
#ifdef VISP_HAVE_OPENMP
#pragma omp for
#endif
      for (vpStored3DSurfaceColorPoint *p : visiblePoints) {

        const bool hasCorrectDepth = frame.hasDepth() && frame.depth[p->projCurrPx[1]][p->projCurrPx[0]] > 0.f;
        const double Z = hasCorrectDepth ? frame.depth[p->projCurrPx[1]][p->projCurrPx[0]] : 0.0;
        double depthError = Z > 0 ? fabs(p->currX[2] - Z) : 0.0;
        double probaDepth = 1.0;
        double scaleFactor = p->stats.covarianceScaleFactor();
        double weight = 1.0 - std::min(1.0, scaleFactor / std::pow((m_initialColorSigma + 20.0), 2));
        weightSumLocal += weight;


        if (hasCorrectDepth) {
          probaDepth = 1.0 - erf((depthError) / (m_depthSigma * sqrt(2.0)));
        }

        vpRGBf averageColor(0.f, 0.f, 0.f);

        for (int i = -1; i < 2; ++i) {
          for (int j = -1; j < 2; ++j) {
            const vpRGBa currentColor = frame.IRGB[p->projCurrPx[1] + i][p->projCurrPx[0] + j];
            averageColor.R += static_cast<float>(currentColor.R);
            averageColor.G += static_cast<float>(currentColor.G);
            averageColor.B += static_cast<float>(currentColor.B);

            // const vpRGBf c(static_cast<float>(currentColor.R), static_cast<float>(currentColor.G), static_cast<float>(currentColor.B));
            // double probaColor = p->stats.probability(c);
            // if (probaColor > bestColorProba) {
            //   bestColorProba = probaColor;
            //   bestColor = c;
            // }
          }
        }
        averageColor = averageColor * (1.f / 9.f);

        const double proba = p->stats.probability(averageColor) * probaDepth;

        scoresLocal.push_back(proba); // Keep only the weight
        scoreLocal += proba * weight;
        p->updateColor(averageColor, m_colorUpdateRate * probaDepth);
      }
#ifdef VISP_HAVE_OPENMP
#pragma omp critical
#endif
      {
        scores.insert(scores.end(), scoresLocal.begin(), scoresLocal.end());
        score += scoreLocal;
        weightSum += weightSumLocal;
      }
    }
    if (!useMedian) {
      // Use average score, may be more sensitive to outliers
      if (weightSum > 0.0) {
        score /= weightSum;
      }
      else {
        score = vpMath::getMean(scores);
      }
      return score;
    }
    else {
      return vpMath::getMedian(scores);
    }
  }
  else {
    return 0.0;
  }
}


void vpRBProbabilistic3DDriftDetector::update(const vpRBFeatureTrackerInput &previousFrame,
                                              const vpRBFeatureTrackerInput &frame,
                                              const vpHomogeneousMatrix &cTo, const vpHomogeneousMatrix &cprevTo)
{
  // const vpHomogeneousMatrix &cprevTo = frame.renders.cMo;
  m_score = score(frame, cTo);

  // Step 4: Sample bb to add new visible points
  const vpHomogeneousMatrix oMcprev = cprevTo.inverse();
  vpColVector cX(4, 1.0), renderX(4, 1.0);
  vpColVector oX(4, 1.0);

  const unsigned int h = frame.renders.depth.getHeight();
  const unsigned int w = frame.renders.depth.getWidth();


  const vpHomogeneousMatrix cprevTrender = cprevTo * frame.renders.cMo.inverse();

  const unsigned int top = static_cast<unsigned int>(std::max(0.0, frame.renders.boundingBox.getTop()));
  const unsigned int left = static_cast<unsigned int>(std::max(0.0, frame.renders.boundingBox.getLeft()));
  const unsigned int bottom = std::min(h, static_cast<unsigned int>(frame.renders.boundingBox.getBottom()));
  const unsigned int right = std::min(w, static_cast<unsigned int>(frame.renders.boundingBox.getRight()));

  for (unsigned int i = top; i < bottom; i += m_sampleStep) {
    for (unsigned int j = fleft; j < right; j += m_sampleStep) {
      double u = static_cast<double>(j), v = static_cast<double>(i);
      double x = 0.0, y = 0.0;
      double Z = static_cast<double>(frame.renders.depth[i][j]);

      if (Z > 0.0) {
        vpPixelMeterConversion::convertPoint(frame.cam, u, v, x, y);
        renderX[0] = x * Z;
        renderX[1] = y * Z;
        renderX[2] = Z;
        cX = cprevTrender * renderX;
        x = cX[0] / cX[2], y = cX[1] / cX[2];
        vpMeterPixelConversion::convertPoint(frame.cam, x, y, u, v);
        int prevI = static_cast<int>(v), prevJ = static_cast<int>(u);
        if (prevI < 0 || prevI >= static_cast<int>(previousFrame.IRGB.getHeight()) || prevJ < 0 || prevJ >= static_cast<int>(previousFrame.IRGB.getWidth())) {
          continue;
        }
        oX = oMcprev * cX;
        vpStored3DSurfaceColorPoint newPoint;
        newPoint.X[0] = oX[0] / oX[3];
        newPoint.X[1] = oX[1] / oX[3];
        newPoint.X[2] = oX[2] / oX[3];
        const vpRGBa &c = previousFrame.IRGB[prevI][prevJ];
        const float colorVariance = std::pow(static_cast<float>(m_initialColorSigma), 2);
        newPoint.stats.init(vpRGBf(c.R, c.G, c.B), vpRGBf(colorVariance));
        bool canAdd = true;
        for (const vpStored3DSurfaceColorPoint &p : m_points) {
          if (p.squaredDist(newPoint.X) < vpMath::sqr(m_minDist3DNewPoint)) {
            canAdd = false;
            break;
          }
        }
        if (canAdd) {
          m_points.push_back(newPoint);
        }
      }
    }
  }

}

void vpRBProbabilistic3DDriftDetector::display(const vpImage<vpRGBa> &I)
{
  for (const vpStored3DSurfaceColorPoint &p : m_points) {
    if (p.visible) {
      const vpRGBf color = p.stats.mean;
      vpDisplay::displayPoint(I, p.projCurrPx[1], p.projCurrPx[0],
        vpColor(static_cast<unsigned char>(color.R), static_cast<unsigned char>(color.G), static_cast<unsigned char>(color.B)) 3);
    }
  }
}

double vpRBProbabilistic3DDriftDetector::getScore() const
{
  return m_score;
}

bool vpRBProbabilistic3DDriftDetector::hasDiverged() const
{
  return m_score < 0.2;
}

#if defined(VISP_HAVE_NLOHMANN_JSON)
void vpRBProbabilistic3DDriftDetector::loadJsonConfiguration(const nlohmann::json &j)
{
  setColorUpdateRate(j.value("colorUpdateRate", m_colorUpdateRate));
  setDepthStandardDeviation(j.value("depthSigma", m_depthSigma));
  setFilteringMax3DError(j.value("filteringMaxDistance", m_maxError3D));
  setInitialColorStandardDeviation(j.value("initialColorSigma", m_initialColorSigma));
  setMinDistForNew3DPoints(j.value("minDistanceNewPoints", m_minDist3DNewPoint));
  setSampleStep(j.value("sampleStep", m_sampleStep));
}

void vpRBProbabilistic3DDriftDetector::loadRepresentation(const std::string &filename)
{
  std::ifstream f(filename);
  if (!f.good()) {
    throw vpException(vpException::ioError, "Could not open file %s", filename.c_str());
  }

  nlohmann::json j = nlohmann::json::parse(f);
  f.close();
  m_points = j;
}
void vpRBProbabilistic3DDriftDetector::saveRepresentation(const std::string &filename) const
{
  nlohmann::json j = m_points;
  std::ofstream f(filename);
  if (!f.good()) {
    throw vpException(vpException::ioError, "Could not open file %s", filename.c_str());
  }
  f << j.dump(2);
  f.close();
}

#endif

END_VISP_NAMESPACE
