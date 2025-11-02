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
  \file vpRBSilhouetteControlPoint.h
  \brief Trackable silhouette point representation
*/

#ifndef VP_RB_SILHOUETTE_CONTROL_POINT_H
#define VP_RB_SILHOUETTE_CONTROL_POINT_H

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpPoint.h>
#include <visp3/core/vpPlane.h>
#include <visp3/core/vpLine.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/me/vpMe.h>
#include <visp3/visual_features/vpFeatureLine.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/core/vpDisplay.h>

BEGIN_VISP_NAMESPACE

/*!
  \brief Trackable silhouette point representation

  \ingroup group_rbt_core

  <h2 id="header-details" class="groupheader">Tutorials & Examples</h2>

  <b>Tutorials</b><br>
  <span style="margin-left:2em"> If you want to have an in-depth presentation of the Render-Based Tracker (RBT), you may have a look at:</span><br>

  - \ref tutorial-tracking-rbt
*/
class VISP_EXPORT vpRBSilhouetteControlPoint
{
private:

  double rho, theta;
  double thetaInit;
  int m_meMaskSign;
  //double a,b,c;
  vpFeatureLine m_lineFeature;
  vpLine m_line;

  std::vector<vpMeSite> m_candidates;
  unsigned int m_numCandidates;
  const vpMe *m_me;
  vpMeSite m_site;

  //! Normal to surface where the control point lies
  vpColVector m_normal;
  vpColVector m_normalO;

  bool m_valid;
  bool m_isSilhouette;

  const vpCameraParameters *m_cam;

public:

  vpImagePoint icpoint;

  // the 3D point
  vpPoint cpoint;
  vpPoint cpointo;

  double xs, ys, nxs, nys, Zs;

public:

  void init();
  vpRBSilhouetteControlPoint();
  vpRBSilhouetteControlPoint(const vpRBSilhouetteControlPoint &meTracker);
  vpRBSilhouetteControlPoint(const vpRBSilhouetteControlPoint &&meTracker);
  vpRBSilhouetteControlPoint &operator=(const vpRBSilhouetteControlPoint &meTracker);
  vpRBSilhouetteControlPoint &operator=(const vpRBSilhouetteControlPoint &&meTracker);
  ~vpRBSilhouetteControlPoint() = default;

  /**
   * @brief Set the number of candidates to use for multiple hypotheses testing
   *
   * @param numCandidates
   */
  void setNumCandidates(unsigned numCandidates) { m_numCandidates = numCandidates; }
  unsigned getNumCandidates() const { return m_numCandidates; }
  void setValid(bool valid) { m_valid = valid; }
  bool isValid() const { return m_valid; }

  const vpCameraParameters &getCameraParameters() const { return *m_cam; }
  bool siteIsValid() const { return m_site.getState() == vpMeSite::NO_SUPPRESSION; }
  const vpMeSite &getSite() const { return m_site; }
  vpMeSite &getSite() { return m_site; }
  const vpFeatureLine &getFeatureLine() const { return m_lineFeature; }
  const vpLine &getLine() const { return m_line; }
  double getTheta() const { return theta; }
  bool isSilhouette() const { return m_isSilhouette; }

  void initControlPoint(const vpImage<unsigned char> &I, double cvlt);
  void buildPoint(int n, int m, const double &Z, double orient, const vpColVector &normo, const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc, const vpCameraParameters &cam, const vpMe &me, bool isSilhouette);
  void buildSilhouettePoint(int n, int m, const double &Z, double orient, const vpColVector &normo, const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc, const vpCameraParameters &cam);

  void update(const vpHomogeneousMatrix &_cMo);
  void updateSilhouettePoint(const vpHomogeneousMatrix &_cMo);

  /**
   * @brief Track the moving edge at this point retaining only the hypothesis with the highest likelihood.
   *
   * @param I The image in which to track.
   */
  void track(const vpImage<unsigned char> &I);

  /**
   * @brief Track the moving edge and retain the best numCandidates hypotheses
   *
   * @param I The image in which to track
   *
   * \see setNumCandidates()
   */
  void trackMultipleHypotheses(const vpImage<unsigned char> &I);

  void computeMeInteractionMatrixError(const vpHomogeneousMatrix &cMo, unsigned int i, vpMatrix &L, vpColVector &e);
  void computeMeInteractionMatrixErrorMH(const vpHomogeneousMatrix &cMo, unsigned int i, vpMatrix &L, vpColVector &e);

  double getMaxMaskGradientAlongLine(const vpImage<float> &mask, int searchSize) const;

  bool tooCloseToBorder(unsigned int h, unsigned int w, int searchSize) const;

private:
  bool isLineDegenerate() const;

  void buildPlane(const vpPoint &pointn, const vpColVector &normal, vpPlane &plane);
  void buildPLine(const vpHomogeneousMatrix &oMc);

};

END_VISP_NAMESPACE

#endif
