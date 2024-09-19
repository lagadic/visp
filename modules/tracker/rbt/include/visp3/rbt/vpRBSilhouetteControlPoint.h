/****************************************************************************
 *
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
*****************************************************************************/

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
*/
class VISP_EXPORT vpRBSilhouetteControlPoint
{
private:

  double rho, theta;
  double thetaInit;
  double delta;
  int sign;
  //double a,b,c;
  vpFeatureLine featureline;
  vpLine line;

  std::vector<vpMeSite> m_candidates;
  unsigned int m_numCandidates;
  vpMe *me;
  vpMeSite s;

  bool m_valid;


public:
  const vpCameraParameters *cam;
  //int imin, imax;
  //int jmin, jmax;
  //double expecteddensity;
  // the image point

  vpImagePoint icpoint;


  // the 3D point
  vpPoint cpoint;
  vpPoint cpointo;

  //! The moving edge container associated to the control point
  //vpMbtMeLine *meline;
  //! The 3D line associated to the control point
  //vpLine *line;

  //! Normale to surface where the control point lies
  vpColVector norm;
  vpColVector normw;


  //! Gradient profile associated to the control Points

  double error;

  vpColVector L;

  double xs, ys, nxs, nys, Zs;

  bool isSilhouette;
  bool invnormal;


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

  int outOfImage(int i, int j, int half, int rows, int cols) const;
  int outOfImage(const vpImagePoint &iP, int half, int rows, int cols) const;

  bool siteIsValid() const { return s.getState() == vpMeSite::NO_SUPPRESSION; }
  const vpMeSite &getSite() const { return s; }
  vpMeSite &getSite() { return s; }
  const vpFeatureLine &getFeatureLine() const { return featureline; }
  const vpLine &getLine() const { return line; }
  double getTheta() const { return theta; }


  void setMovingEdge(vpMe *_me) { me = _me; }
  void setCameraParameters(const vpCameraParameters *_cam) { cam = _cam; }

  void initControlPoint(const vpImage<unsigned char> &I, double cvlt);
  void detectSilhouette(const vpImage<float> &I);
  void buildPoint(int n, int m, const double &Z, double orient, const vpColVector &normo, const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc);
  void buildSilhouettePoint(int n, int m, const double &Z, double orient, const vpColVector &normo, const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc);
  void buildPlane(const vpPoint &pointn, const vpColVector &normal, vpPlane &plane);
  void buildPLine(const vpHomogeneousMatrix &oMc);
  void update(const vpHomogeneousMatrix &_cMo);
  void updateSilhouettePoint(const vpHomogeneousMatrix &_cMo);

  /**
   * @brief Track the moving edge at this point retaining only the hypothesis with the highest likelihood
   *
   * @param I The image in which to track
   */
  void track(const vpImage<unsigned char> &I);

  /**
   * @brief Track the moving edge and retain the best numCandidates hypotheses
   *
   * @param I The image in which to track
   *
   * \see setNumCandidates
   */
  void trackMultipleHypotheses(const vpImage<unsigned char> &I);

  void initInteractionMatrixError();
  void computeInteractionMatrixError(const vpHomogeneousMatrix &cMo);
  void computeInteractionMatrixErrorMH(const vpHomogeneousMatrix &cMo);

private:
  void sample(const vpImage<unsigned char> &) { }
  bool isLineDegenerate() const;



};
END_VISP_NAMESPACE

#endif
