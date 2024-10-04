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
#include <visp3/core/vpDisplay.h>

#include <visp3/rbt/vpRBFeatureTracker.h>

#if defined(VISP_HAVE_NLOHMANN_JSON)
#include <nlohmann/json.hpp>
#endif

BEGIN_VISP_NAMESPACE

void vpRBProbabilistic3DDriftDetector::update(const vpRBFeatureTrackerInput &previousFrame,
                                              const vpRBFeatureTrackerInput &frame,
                                              const vpHomogeneousMatrix &cTo, const vpHomogeneousMatrix &cprevTo)
{
  const vpTranslationVector t = cprevTo.inverse().getTranslationVector();

  if (m_points.size() > 0) {
    // Step 0: project all points
#ifdef VISP_HAVE_OPENMP
#pragma omp parallel for
#endif
    for (vpStored3DSurfaceColorPoint &p : m_points) {
      p.update(cTo, cprevTo, frame.cam);
    }

    // Step 1: gather points visible in both images and in render
    std::vector<vpStored3DSurfaceColorPoint *> visiblePoints;
    for (vpStored3DSurfaceColorPoint &p : m_points) {
      p.visible = true;
      if (
        p.projPrevPx[0] < 2 || static_cast<unsigned int>(p.projPrevPx[0]) >= frame.IRGB.getWidth() - 2
      || p.projPrevPx[1] < 2 || static_cast<unsigned int>(p.projPrevPx[1]) >= frame.IRGB.getHeight() - 2
      || p.projCurrPx[0] < 2 || static_cast<unsigned int>(p.projCurrPx[0]) >= frame.IRGB.getWidth() - 2
      || p.projCurrPx[1] < 2 || static_cast<unsigned int>(p.projCurrPx[1]) >= frame.IRGB.getHeight() - 2) {
        p.visible = false; // Point is outside of either current or previous image, ignore it
        continue;
      }

      // if (fabs(p.currX[2] - ZcurrMap) > m_maxError3D || fabs(p.prevX[2] - ZprevMap) > m_maxError3D) {
      //   continue; // Depth is wrong:  probable occlusion, ignore it
      // }

      float ZrenderMap = frame.renders.depth[p.projPrevPx[1]][p.projPrevPx[0]];
      // Version 2: compare previous projection with render, this does not filter occlusions
      if (ZrenderMap == 0.f || fabs(p.prevX[2] - ZrenderMap) > m_maxError3D) {
        p.visible = false;
        continue;
      }

      vpRGBf normalObject = frame.renders.normals[p.projPrevPx[1]][p.projPrevPx[0]];

      vpColVector vector({ t[0] - p.X[0], t[1] - p.X[1], t[2] - p.X[2] });

      vector.normalize();
      double angle = acos(vpColVector::dotProd(vpColVector({ normalObject.R, normalObject.G, normalObject.B }).normalize(), vector));
      if (angle > vpMath::rad(75)) {
        p.visible = false;
        continue;
      }

      if (frame.silhouettePoints.size() > 0) {
        for (const vpRBSilhouettePoint &sp: frame.silhouettePoints) {
          if (std::pow(static_cast<double>(sp.i) - p.projPrevPx[1], 2) + std::pow(static_cast<double>(sp.j) - p.projPrevPx[0], 2) < vpMath::sqr(5)) {
            p.visible = false;
            break;
          }
        }
      }
      // Version 3: could be using version 1 and 2. If 1 is wrong but 2 is ok, then there is an issue that is not self occlusion
      // We could reweigh the error by the number of problematic points
      // ...

      if (p.visible) {
        visiblePoints.push_back(&p);
      }
    }

    if (visiblePoints.size() > 0) {
      //   // Compute sample weight
      //   double maxTrace = 0.0;

      //   for (vpStored3DSurfaceColorPoint *p : visiblePoints) {
      //     double trace = p->stats.trace();
      //     if (trace > maxTrace) {
      //       maxTrace = trace;
      //     }
      //   }
      //   maxTrace = std::max(maxTrace, 80.0);
      double weightSum = 0.0;
      m_score = 0.0;
      for (vpStored3DSurfaceColorPoint *p : visiblePoints) {
        double maxProba = 0.0;
        vpRGBf minColor;
        const bool hasCorrectDepth = frame.hasDepth() && frame.depth[p->projPrevPx[1]][p->projPrevPx[0]] > 0.f;
        const double Z = hasCorrectDepth ? frame.depth[p->projPrevPx[1]][p->projPrevPx[0]] : 0.0;
        double depthError = Z > 0 ? fabs(p->prevX[2] - Z) : 0.0;
        double probaDepth = 1.0;
        if (hasCorrectDepth) {
          probaDepth = 1.0 - erf((depthError) / (m_depthSigma * sqrt(2.0)));
        }
        // double weight = 1.0 - ((p->stats.trace() / maxTrace));
        // if (weight < 0.0) {
        //   throw vpException(vpException::badValue, "Got invalid weight");
        // }
        double weight = 1.0;

        vpRGBf averageColor(0.f, 0.f, 0.f);
        for (int i = -1; i < 2; ++i) {
          for (int j = -1; j < 2; ++j) {
            const vpRGBa currentColor = frame.IRGB[p->projCurrPx[1] + i][p->projCurrPx[0] + j];
            averageColor.R += currentColor.R;
            averageColor.G += currentColor.G;
            averageColor.B += currentColor.B;
          }
        }
        averageColor = averageColor * (1.0 / 9.0);
        // const vpRGBf c(currentColor.R, currentColor.G, currentColor.B);
        const vpRGBf c(averageColor);

        const double probaColor = p->stats.probability(c);
        const double proba = probaColor * probaDepth;
        if (probaDepth > 1.f || probaDepth < 0.0) {
          throw vpException(vpException::badValue, "Wrong depth probability");
        }
        if (proba > maxProba) {
          maxProba = proba;
          minColor = c;
        }

        m_score += maxProba * weight;
        weightSum += weight;
        p->updateColor(minColor, m_colorUpdateRate * probaDepth);
      }
      m_score /= (weightSum);
    }
    else {
      m_score = 1.0;
    }
  }
  else {
    m_score = 1.0;
  }

  // Step 4: Sample bb to add new visible points
  const vpHomogeneousMatrix oMcprev = cprevTo.inverse();
  vpColVector cX(4, 1.0);
  vpColVector oX(4, 1.0);

  for (unsigned int i = frame.renders.boundingBox.getTop(); i < frame.renders.boundingBox.getBottom(); i += 2) {
    for (unsigned int j = frame.renders.boundingBox.getLeft(); j < frame.renders.boundingBox.getRight(); j += 2) {
      double u = static_cast<double>(j), v = static_cast<double>(i);
      double x = 0.0, y = 0.0;
      double Z = frame.renders.depth[i][j];
      if (Z > 0.f) {
        vpPixelMeterConversion::convertPoint(frame.cam, u, v, x, y);
        cX[0] = x * Z;
        cX[1] = y * Z;
        cX[2] = Z;
        oX = oMcprev * cX;
        vpStored3DSurfaceColorPoint newPoint;
        newPoint.X[0] = oX[0] / oX[3];
        newPoint.X[1] = oX[1] / oX[3];
        newPoint.X[2] = oX[2] / oX[3];
        const vpRGBa &c = previousFrame.IRGB[i][j];
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

void vpRBProbabilistic3DDriftDetector::display(const vpImage<vpRGBa> &/*I*/)
{
  // for (const vpStored3DSurfaceColorPoint &p : m_points) {
  //   if (p.visible) {
  //     const vpRGBf color = p.stats.mean;
  //     // vpDisplay::displayPoint(I, p.projCurrPx[1], p.projCurrPx[0], vpColor::blue);
  //     vpDisplay::displayPoint(I, p.projCurrPx[1], p.projCurrPx[0], vpColor(color.R, color.G, color.B), 3);
  //     // vpDisplay::displayPoint(I, p.projCurrPx[1], p.projCurrPx[0], vpColor::white, 3);

  //     // vpDisplay::displayLine(I, p.projCurrPx[1], p.projCurrPx[0], p.projPrevPx[1], p.projPrevPx[0], vpColor::red);
  //   }
  // }
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
}
#endif

END_VISP_NAMESPACE
