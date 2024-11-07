/*
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
 * Pose computation from any features.
 */

#include <visp3/core/vpDebug.h>
#include <visp3/vision/vpPoseFeatures.h>

#if defined(VISP_HAVE_MODULE_VISUAL_FEATURES) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)

BEGIN_VISP_NAMESPACE

vpPoseFeatures::vpPoseFeatures()
  : m_maxSize(0), m_totalSize(0), m_vvsIterMax(200), m_lambda(1.0), m_verbose(false), m_computeCovariance(false),
  m_covarianceMatrix(), m_featurePoint_Point_list(), m_featurePoint3D_Point_list(), m_featureVanishingPoint_Point_list(),
  m_featureVanishingPoint_DuoLine_list(), m_featureEllipse_Sphere_list(), m_featureEllipse_Circle_list(),
  m_featureLine_Line_list(), m_featureLine_DuoLineInt_List(), m_featureSegment_DuoPoints_list()
{ }

vpPoseFeatures::~vpPoseFeatures() { clear(); }

void vpPoseFeatures::clear()
{
  for (int i = (int)m_featurePoint_Point_list.size() - 1; i >= 0; i--)
    delete m_featurePoint_Point_list[(unsigned int)i].desiredFeature;
  m_featurePoint_Point_list.clear();

  for (int i = (int)m_featurePoint3D_Point_list.size() - 1; i >= 0; i--)
    delete m_featurePoint3D_Point_list[(unsigned int)i].desiredFeature;
  m_featurePoint3D_Point_list.clear();

  for (int i = (int)m_featureVanishingPoint_Point_list.size() - 1; i >= 0; i--)
    delete m_featureVanishingPoint_Point_list[(unsigned int)i].desiredFeature;
  m_featureVanishingPoint_Point_list.clear();

  for (int i = (int)m_featureVanishingPoint_DuoLine_list.size() - 1; i >= 0; i--)
    delete m_featureVanishingPoint_DuoLine_list[(unsigned int)i].desiredFeature;
  m_featureVanishingPoint_DuoLine_list.clear();

  for (int i = (int)m_featureEllipse_Sphere_list.size() - 1; i >= 0; i--)
    delete m_featureEllipse_Sphere_list[(unsigned int)i].desiredFeature;
  m_featureEllipse_Sphere_list.clear();

  for (int i = (int)m_featureEllipse_Circle_list.size() - 1; i >= 0; i--)
    delete m_featureEllipse_Circle_list[(unsigned int)i].desiredFeature;
  m_featureEllipse_Circle_list.clear();

  for (int i = (int)m_featureLine_Line_list.size() - 1; i >= 0; i--)
    delete m_featureLine_Line_list[(unsigned int)i].desiredFeature;
  m_featureLine_Line_list.clear();

  for (int i = (int)m_featureLine_DuoLineInt_List.size() - 1; i >= 0; i--)
    delete m_featureLine_DuoLineInt_List[(unsigned int)i].desiredFeature;
  m_featureLine_DuoLineInt_List.clear();

  for (int i = (int)m_featureSegment_DuoPoints_list.size() - 1; i >= 0; i--)
    delete m_featureSegment_DuoPoints_list[(unsigned int)i].desiredFeature;
  m_featureSegment_DuoPoints_list.clear();

  for (int i = (int)m_featureSpecific_list.size() - 1; i >= 0; i--)
    delete m_featureSpecific_list[(unsigned int)i];
  m_featureSpecific_list.clear();

  m_maxSize = 0;
  m_totalSize = 0;
}

void vpPoseFeatures::addFeaturePoint(const vpPoint &p)
{
  m_featurePoint_Point_list.push_back(vpDuo<vpFeaturePoint, vpPoint>());
  m_featurePoint_Point_list.back().firstParam = p;
  m_featurePoint_Point_list.back().desiredFeature = new vpFeaturePoint();
  vpFeatureBuilder::create(*m_featurePoint_Point_list.back().desiredFeature, p);

  m_totalSize++;
  if (m_featurePoint_Point_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featurePoint_Point_list.size();
}

void vpPoseFeatures::addFeaturePoint3D(const vpPoint &p)
{
  m_featurePoint3D_Point_list.push_back(vpDuo<vpFeaturePoint3D, vpPoint>());
  m_featurePoint3D_Point_list.back().firstParam = p;
  m_featurePoint3D_Point_list.back().desiredFeature = new vpFeaturePoint3D();
  vpFeatureBuilder::create(*m_featurePoint3D_Point_list.back().desiredFeature, p);

  m_totalSize++;
  if (m_featurePoint3D_Point_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featurePoint3D_Point_list.size();
}

void vpPoseFeatures::addFeatureVanishingPoint(const vpPoint &p)
{
  m_featureVanishingPoint_Point_list.push_back(vpDuo<vpFeatureVanishingPoint, vpPoint>());
  m_featureVanishingPoint_Point_list.back().firstParam = p;
  m_featureVanishingPoint_Point_list.back().desiredFeature = new vpFeatureVanishingPoint();
  vpFeatureBuilder::create(*m_featureVanishingPoint_Point_list.back().desiredFeature, p);

  m_totalSize++;
  if (m_featureVanishingPoint_Point_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featureVanishingPoint_Point_list.size();
}

void vpPoseFeatures::addFeatureVanishingPoint(const vpLine &l1, const vpLine &l2)
{
  m_featureVanishingPoint_DuoLine_list.push_back(vpTrio<vpFeatureVanishingPoint, vpLine, vpLine>());
  m_featureVanishingPoint_DuoLine_list.back().firstParam = l1;
  m_featureVanishingPoint_DuoLine_list.back().secondParam = l2;
  m_featureVanishingPoint_DuoLine_list.back().desiredFeature = new vpFeatureVanishingPoint();
  vpFeatureBuilder::create(*m_featureVanishingPoint_DuoLine_list.back().desiredFeature, l1, l2);

  m_totalSize++;
  if (m_featureVanishingPoint_DuoLine_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featureVanishingPoint_DuoLine_list.size();
}

void vpPoseFeatures::addFeatureEllipse(const vpSphere &s)
{
  m_featureEllipse_Sphere_list.push_back(vpDuo<vpFeatureEllipse, vpSphere>());
  m_featureEllipse_Sphere_list.back().firstParam = s;
  m_featureEllipse_Sphere_list.back().desiredFeature = new vpFeatureEllipse();
  vpFeatureBuilder::create(*m_featureEllipse_Sphere_list.back().desiredFeature, s);

  m_totalSize++;
  if (m_featureEllipse_Sphere_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featureEllipse_Sphere_list.size();
}

void vpPoseFeatures::addFeatureEllipse(const vpCircle &c)
{
  m_featureEllipse_Circle_list.push_back(vpDuo<vpFeatureEllipse, vpCircle>());
  m_featureEllipse_Circle_list.back().firstParam = c;
  m_featureEllipse_Circle_list.back().desiredFeature = new vpFeatureEllipse();
  vpFeatureBuilder::create(*m_featureEllipse_Circle_list.back().desiredFeature, c);

  m_totalSize++;
  if (m_featureEllipse_Circle_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featureEllipse_Circle_list.size();
}

void vpPoseFeatures::addFeatureLine(const vpLine &l)
{
  m_featureLine_Line_list.push_back(vpDuo<vpFeatureLine, vpLine>());
  m_featureLine_Line_list.back().firstParam = l;
  m_featureLine_Line_list.back().desiredFeature = new vpFeatureLine();
  vpFeatureBuilder::create(*m_featureLine_Line_list.back().desiredFeature, l);

  m_totalSize++;
  if (m_featureLine_Line_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featureLine_Line_list.size();
}

void vpPoseFeatures::addFeatureLine(const vpCylinder &c, const int &line)
{
  m_featureLine_DuoLineInt_List.push_back(vpTrio<vpFeatureLine, vpCylinder, int>());
  m_featureLine_DuoLineInt_List.back().firstParam = c;
  m_featureLine_DuoLineInt_List.back().secondParam = line;
  m_featureLine_DuoLineInt_List.back().desiredFeature = new vpFeatureLine();
  vpFeatureBuilder::create(*m_featureLine_DuoLineInt_List.back().desiredFeature, c, line);

  m_totalSize++;
  if (m_featureLine_DuoLineInt_List.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featureLine_DuoLineInt_List.size();
}

void vpPoseFeatures::addFeatureSegment(vpPoint &P1, vpPoint &P2)
{
  m_featureSegment_DuoPoints_list.push_back(vpTrio<vpFeatureSegment, vpPoint, vpPoint>());
  m_featureSegment_DuoPoints_list.back().firstParam = P1;
  m_featureSegment_DuoPoints_list.back().secondParam = P2;
  m_featureSegment_DuoPoints_list.back().desiredFeature = new vpFeatureSegment();
  vpFeatureBuilder::create(*m_featureSegment_DuoPoints_list.back().desiredFeature, P1, P2);

  m_totalSize++;
  if (m_featureSegment_DuoPoints_list.size() > m_maxSize)
    m_maxSize = (unsigned int)m_featureSegment_DuoPoints_list.size();
}

void vpPoseFeatures::error_and_interaction(vpHomogeneousMatrix &cMo, vpColVector &err, vpMatrix &L)
{
  err = vpColVector();
  L = vpMatrix();

  for (unsigned int i = 0; i < m_maxSize; i++) {
    //--------------vpFeaturePoint--------------
    // From vpPoint
    if (i < m_featurePoint_Point_list.size()) {
      vpFeaturePoint fp;
      vpPoint p(m_featurePoint_Point_list[i].firstParam);
      p.track(cMo);
      vpFeatureBuilder::create(fp, p);
      err.stack(fp.error(*(m_featurePoint_Point_list[i].desiredFeature)));
      L.stack(fp.interaction());
    }

    //--------------vpFeaturePoint3D--------------
    // From vpPoint
    if (i < m_featurePoint3D_Point_list.size()) {
      vpFeaturePoint3D fp3D;
      vpPoint p(m_featurePoint3D_Point_list[i].firstParam);
      p.track(cMo);
      vpFeatureBuilder::create(fp3D, p);
      err.stack(fp3D.error(*(m_featurePoint3D_Point_list[i].desiredFeature)));
      L.stack(fp3D.interaction());
    }

    //--------------vpFeatureVanishingPoint--------------
    // From vpPoint
    if (i < m_featureVanishingPoint_Point_list.size()) {
      vpFeatureVanishingPoint fvp;
      vpPoint p(m_featureVanishingPoint_Point_list[i].firstParam);
      p.track(cMo);
      vpFeatureBuilder::create(fvp, p);
      err.stack(fvp.error(*(m_featureVanishingPoint_Point_list[i].desiredFeature)));
      L.stack(fvp.interaction());
    }
    // From Duo of vpLines
    if (i < m_featureVanishingPoint_DuoLine_list.size()) {
      vpFeatureVanishingPoint fvp;
      vpLine l1(m_featureVanishingPoint_DuoLine_list[i].firstParam);
      vpLine l2(m_featureVanishingPoint_DuoLine_list[i].secondParam);
      l1.track(cMo);
      l2.track(cMo);
      vpFeatureBuilder::create(fvp, l1, l2);
      err.stack(fvp.error(*(m_featureVanishingPoint_DuoLine_list[i].desiredFeature)));
      L.stack(fvp.interaction());
    }

    //--------------vpFeatureEllipse--------------
    // From vpSphere
    if (i < m_featureEllipse_Sphere_list.size()) {
      vpFeatureEllipse fe;
      vpSphere s(m_featureEllipse_Sphere_list[i].firstParam);
      s.track(cMo);
      vpFeatureBuilder::create(fe, s);
      err.stack(fe.error(*(m_featureEllipse_Sphere_list[i].desiredFeature)));
      L.stack(fe.interaction());
    }
    // From vpCircle
    if (i < m_featureEllipse_Circle_list.size()) {
      vpFeatureEllipse fe;
      vpCircle c(m_featureEllipse_Circle_list[i].firstParam);
      c.track(cMo);
      vpFeatureBuilder::create(fe, c);
      err.stack(fe.error(*(m_featureEllipse_Circle_list[i].desiredFeature)));
      L.stack(fe.interaction());
    }

    //--------------vpFeatureLine--------------
    // From vpLine
    if (i < m_featureLine_Line_list.size()) {
      vpFeatureLine fl;
      vpLine l(m_featureLine_Line_list[i].firstParam);
      l.track(cMo);
      vpFeatureBuilder::create(fl, l);
      err.stack(fl.error(*(m_featureLine_Line_list[i].desiredFeature)));
      L.stack(fl.interaction());
    }
    // From Duo of vpCylinder / Integer
    if (i < m_featureLine_DuoLineInt_List.size()) {
      vpFeatureLine fl;
      vpCylinder c(m_featureLine_DuoLineInt_List[i].firstParam);
      c.track(cMo);
      vpFeatureBuilder::create(fl, c, m_featureLine_DuoLineInt_List[i].secondParam);
      err.stack(fl.error(*(m_featureLine_DuoLineInt_List[i].desiredFeature)));
      L.stack(fl.interaction());
    }

    //--------------vpFeatureSegment--------------
    // From Duo of vpPoints
    if (i < m_featureSegment_DuoPoints_list.size()) {
      vpFeatureSegment fs;
      vpPoint p1(m_featureSegment_DuoPoints_list[i].firstParam);
      vpPoint p2(m_featureSegment_DuoPoints_list[i].secondParam);
      p1.track(cMo);
      p2.track(cMo);
      vpFeatureBuilder::create(fs, p1, p2);
      err.stack(fs.error(*(m_featureSegment_DuoPoints_list[i].desiredFeature)));
      L.stack(fs.interaction());
    }

    //--------------Specific Feature--------------
    if (i < m_featureSpecific_list.size()) {
      m_featureSpecific_list[i]->createCurrent(cMo);
      err.stack(m_featureSpecific_list[i]->error());
      L.stack(m_featureSpecific_list[i]->currentInteraction());
    }
  }
}

void vpPoseFeatures::computePose(vpHomogeneousMatrix &cMo, const vpPoseFeaturesMethodType &type)
{
  switch (type) {
  case VIRTUAL_VS:
    computePoseVVS(cMo);
    break;
  case ROBUST_VIRTUAL_VS:
    computePoseRobustVVS(cMo);
    break;
  default:
    break;
  }
}

void vpPoseFeatures::computePoseVVS(vpHomogeneousMatrix &cMo)
{
  try {
    double residu_1 = 1e8;
    double r = 1e8 - 1;
    // we stop the minimization when the error is bellow 1e-8

    vpMatrix L;
    vpColVector err;
    vpColVector v;
    unsigned int rank_max = 0;
    unsigned int iter = 0;

    // while((int)((residu_1 - r)*1e12) != 0 )
    while (std::fabs((residu_1 - r) * 1e12) > std::numeric_limits<double>::epsilon()) {
      residu_1 = r;

      // Compute the interaction matrix and the error
      error_and_interaction(cMo, err, L);

      // compute the residual
      r = err.sumSquare();

      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp;
      unsigned int rank = L.pseudoInverse(Lp, 1e-6); // modif FC 1e-16

      if (rank_max < rank)
        rank_max = rank;

      // compute the VVS control law
      v = -m_lambda * Lp * err;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;
      if (iter++ > m_vvsIterMax) {
        vpTRACE("Max iteration reached");
        break;
      }
    }
    if (rank_max < 6) {
      if (m_verbose) {
        vpTRACE("Only %d pose parameters can be estimated.", rank_max);
      }
    }

    if (m_computeCovariance)
      m_covarianceMatrix = vpMatrix::computeCovarianceMatrix(L, v, -m_lambda * err);

  }
  catch (...) {
    vpERROR_TRACE("vpPoseFeatures::computePoseVVS");
    throw;
  }
}

void vpPoseFeatures::computePoseRobustVVS(vpHomogeneousMatrix &cMo)
{
  try {
    double residu_1 = 1e8;
    double r = 1e8 - 1;

    // we stop the minimization when the error is bellow 1e-8
    vpMatrix L, W;
    vpColVector w, res;
    vpColVector v;
    vpColVector error; // error vector

    vpRobust robust;
    robust.setMinMedianAbsoluteDeviation(0.00001);

    unsigned int rank_max = 0;
    unsigned int iter = 0;

    // while((int)((residu_1 - r)*1e12) !=0)
    while (std::fabs((residu_1 - r) * 1e12) > std::numeric_limits<double>::epsilon()) {
      residu_1 = r;

      // Compute the interaction matrix and the error
      error_and_interaction(cMo, error, L);

      // compute the residual
      r = error.sumSquare();

      if (iter == 0) {
        res.resize(error.getRows() / 2);
        w.resize(error.getRows() / 2);
        W.resize(error.getRows(), error.getRows());
        w = 1;
      }

      for (unsigned int k = 0; k < error.getRows() / 2; k++) {
        res[k] = vpMath::sqr(error[2 * k]) + vpMath::sqr(error[2 * k + 1]);
      }
      robust.MEstimator(vpRobust::TUKEY, res, w);

      // compute the pseudo inverse of the interaction matrix
      for (unsigned int k = 0; k < error.getRows() / 2; k++) {
        W[2 * k][2 * k] = w[k];
        W[2 * k + 1][2 * k + 1] = w[k];
      }
      // compute the pseudo inverse of the interaction matrix
      vpMatrix Lp;
      vpMatrix LRank;
      (W * L).pseudoInverse(Lp, 1e-6);
      unsigned int rank = L.pseudoInverse(LRank, 1e-6);

      if (rank_max < rank)
        rank_max = rank;

      // compute the VVS control law
      v = -m_lambda * Lp * W * error;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;
      if (iter++ > m_vvsIterMax) {
        vpTRACE("Max iteration reached");
        break;
      }
    }

    if (rank_max < 6) {
      if (m_verbose) {
        vpTRACE("Only %d pose parameters can be estimated.", rank_max);
      }
    }

    if (m_computeCovariance)
      m_covarianceMatrix =
      vpMatrix::computeCovarianceMatrix(L, v, -m_lambda * error, W * W); // Remark: W*W = W*W.t() since the
                                                                       // matrix is diagonal, but using W*W
                                                                       // is more efficient.
  }
  catch (...) {
    vpERROR_TRACE("vpPoseFeatures::computePoseRobustVVS");
    throw;
  }
}

END_VISP_NAMESPACE

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_vision.a(vpPoseFeatures.cpp.o) has no symbols
void dummy_vpPoseFeatures() { };
#endif
