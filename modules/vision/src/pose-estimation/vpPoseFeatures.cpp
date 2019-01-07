/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Pose computation from any features.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/
#include <visp3/vision/vpPoseFeatures.h>

#ifdef VISP_HAVE_MODULE_VISUAL_FEATURES

/*!
  Default constructor.
*/
vpPoseFeatures::vpPoseFeatures()
  : maxSize(0), totalSize(0), vvsIterMax(200), lambda(1.0), verbose(false), computeCovariance(false),
    covarianceMatrix(), featurePoint_Point_list(), featurePoint3D_Point_list(), featureVanishingPoint_Point_list(),
    featureVanishingPoint_DuoLine_list(), featureEllipse_Sphere_list(), featureEllipse_Circle_list(),
    featureLine_Line_list(), featureLine_DuoLineInt_List(), featureSegment_DuoPoints_list()
{
}

/*!
  Destructor that deletes the array of features and projections.
*/
vpPoseFeatures::~vpPoseFeatures() { clear(); }

/*!
 Clear all the features
*/
void vpPoseFeatures::clear()
{
  for (int i = (int)featurePoint_Point_list.size() - 1; i >= 0; i--)
    delete featurePoint_Point_list[(unsigned int)i].desiredFeature;
  featurePoint_Point_list.clear();

  for (int i = (int)featurePoint3D_Point_list.size() - 1; i >= 0; i--)
    delete featurePoint3D_Point_list[(unsigned int)i].desiredFeature;
  featurePoint3D_Point_list.clear();

  for (int i = (int)featureVanishingPoint_Point_list.size() - 1; i >= 0; i--)
    delete featureVanishingPoint_Point_list[(unsigned int)i].desiredFeature;
  featureVanishingPoint_Point_list.clear();

  for (int i = (int)featureVanishingPoint_DuoLine_list.size() - 1; i >= 0; i--)
    delete featureVanishingPoint_DuoLine_list[(unsigned int)i].desiredFeature;
  featureVanishingPoint_DuoLine_list.clear();

  for (int i = (int)featureEllipse_Sphere_list.size() - 1; i >= 0; i--)
    delete featureEllipse_Sphere_list[(unsigned int)i].desiredFeature;
  featureEllipse_Sphere_list.clear();

  for (int i = (int)featureEllipse_Circle_list.size() - 1; i >= 0; i--)
    delete featureEllipse_Circle_list[(unsigned int)i].desiredFeature;
  featureEllipse_Circle_list.clear();

  for (int i = (int)featureLine_Line_list.size() - 1; i >= 0; i--)
    delete featureLine_Line_list[(unsigned int)i].desiredFeature;
  featureLine_Line_list.clear();

  for (int i = (int)featureLine_DuoLineInt_List.size() - 1; i >= 0; i--)
    delete featureLine_DuoLineInt_List[(unsigned int)i].desiredFeature;
  featureLine_DuoLineInt_List.clear();

  for (int i = (int)featureSegment_DuoPoints_list.size() - 1; i >= 0; i--)
    delete featureSegment_DuoPoints_list[(unsigned int)i].desiredFeature;
  featureSegment_DuoPoints_list.clear();

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  for (int i = (int)featureSpecific_list.size() - 1; i >= 0; i--)
    delete featureSpecific_list[(unsigned int)i];
  featureSpecific_list.clear();
#endif

  maxSize = 0;
  totalSize = 0;
}

/*!
  Add a point feature to the list of features to be considered in the pose
  computation.

  \param p : Point projection expressed as a vpPoint.
*/
void vpPoseFeatures::addFeaturePoint(const vpPoint &p)
{
  featurePoint_Point_list.push_back(vpDuo<vpFeaturePoint, vpPoint>());
  featurePoint_Point_list.back().firstParam = p;
  featurePoint_Point_list.back().desiredFeature = new vpFeaturePoint();
  vpFeatureBuilder::create(*featurePoint_Point_list.back().desiredFeature, p);

  totalSize++;
  if (featurePoint_Point_list.size() > maxSize)
    maxSize = (unsigned int)featurePoint_Point_list.size();
}

/*!
  Add a point 3D feature to the list of features to be considered in the pose
  computation.

  \param p : Projection expressed as a vpPoint.
*/
void vpPoseFeatures::addFeaturePoint3D(const vpPoint &p)
{
  featurePoint3D_Point_list.push_back(vpDuo<vpFeaturePoint3D, vpPoint>());
  featurePoint3D_Point_list.back().firstParam = p;
  featurePoint3D_Point_list.back().desiredFeature = new vpFeaturePoint3D();
  vpFeatureBuilder::create(*featurePoint3D_Point_list.back().desiredFeature, p);

  totalSize++;
  if (featurePoint3D_Point_list.size() > maxSize)
    maxSize = (unsigned int)featurePoint3D_Point_list.size();
}

/*!
  Add a vanishing point feature to the list of features to be considered in
  the pose computation.

  \param p : Projection expressed as a vpPoint.
*/
void vpPoseFeatures::addFeatureVanishingPoint(const vpPoint &p)
{
  featureVanishingPoint_Point_list.push_back(vpDuo<vpFeatureVanishingPoint, vpPoint>());
  featureVanishingPoint_Point_list.back().firstParam = p;
  featureVanishingPoint_Point_list.back().desiredFeature = new vpFeatureVanishingPoint();
  vpFeatureBuilder::create(*featureVanishingPoint_Point_list.back().desiredFeature, p);

  totalSize++;
  if (featureVanishingPoint_Point_list.size() > maxSize)
    maxSize = (unsigned int)featureVanishingPoint_Point_list.size();
}

/*!
  Add a vanishing point feature to the list of features to be considered in
  the pose computation.

  \param l1 : First line used to create the feature.
  \param l2 : Second line used to create the feature.
*/
void vpPoseFeatures::addFeatureVanishingPoint(const vpLine &l1, const vpLine &l2)
{
  featureVanishingPoint_DuoLine_list.push_back(vpTrio<vpFeatureVanishingPoint, vpLine, vpLine>());
  featureVanishingPoint_DuoLine_list.back().firstParam = l1;
  featureVanishingPoint_DuoLine_list.back().secondParam = l2;
  featureVanishingPoint_DuoLine_list.back().desiredFeature = new vpFeatureVanishingPoint();
  vpFeatureBuilder::create(*featureVanishingPoint_DuoLine_list.back().desiredFeature, l1, l2);

  totalSize++;
  if (featureVanishingPoint_DuoLine_list.size() > maxSize)
    maxSize = (unsigned int)featureVanishingPoint_DuoLine_list.size();
}

/*!
  Add an ellipse feature to the list of features to be considered in the pose
  computation.

  \param s : Ellipse projection expressed as a vpSphere.
*/
void vpPoseFeatures::addFeatureEllipse(const vpSphere &s)
{
  featureEllipse_Sphere_list.push_back(vpDuo<vpFeatureEllipse, vpSphere>());
  featureEllipse_Sphere_list.back().firstParam = s;
  featureEllipse_Sphere_list.back().desiredFeature = new vpFeatureEllipse();
  vpFeatureBuilder::create(*featureEllipse_Sphere_list.back().desiredFeature, s);

  totalSize++;
  if (featureEllipse_Sphere_list.size() > maxSize)
    maxSize = (unsigned int)featureEllipse_Sphere_list.size();
}

/*!
  Add an ellipse feature to the list of features to be considered in the pose
  computation.

  \param c : Ellipse projection expressed as a vpCircle.
*/
void vpPoseFeatures::addFeatureEllipse(const vpCircle &c)
{
  featureEllipse_Circle_list.push_back(vpDuo<vpFeatureEllipse, vpCircle>());
  featureEllipse_Circle_list.back().firstParam = c;
  featureEllipse_Circle_list.back().desiredFeature = new vpFeatureEllipse();
  vpFeatureBuilder::create(*featureEllipse_Circle_list.back().desiredFeature, c);

  totalSize++;
  if (featureEllipse_Circle_list.size() > maxSize)
    maxSize = (unsigned int)featureEllipse_Circle_list.size();
}

/*!
  Add a line feature to the list of features to be considered in the pose
  computation.

  \param l : Line projection expressed as a vpLine.
*/
void vpPoseFeatures::addFeatureLine(const vpLine &l)
{
  featureLine_Line_list.push_back(vpDuo<vpFeatureLine, vpLine>());
  featureLine_Line_list.back().firstParam = l;
  featureLine_Line_list.back().desiredFeature = new vpFeatureLine();
  vpFeatureBuilder::create(*featureLine_Line_list.back().desiredFeature, l);

  totalSize++;
  if (featureLine_Line_list.size() > maxSize)
    maxSize = (unsigned int)featureLine_Line_list.size();
}

/*!
  Add a line feature to the list of features to be considered in the pose
  computation.

  \param c : Line projection expressed as a vpCylinder.
  \param line : Integer id that indicates which limb of the cylinder is to
  consider. It can be vpCylinder::line1 or vpCylinder::line2.
*/
void vpPoseFeatures::addFeatureLine(const vpCylinder &c, const int &line)
{
  featureLine_DuoLineInt_List.push_back(vpTrio<vpFeatureLine, vpCylinder, int>());
  featureLine_DuoLineInt_List.back().firstParam = c;
  featureLine_DuoLineInt_List.back().secondParam = line;
  featureLine_DuoLineInt_List.back().desiredFeature = new vpFeatureLine();
  vpFeatureBuilder::create(*featureLine_DuoLineInt_List.back().desiredFeature, c, line);

  totalSize++;
  if (featureLine_DuoLineInt_List.size() > maxSize)
    maxSize = (unsigned int)featureLine_DuoLineInt_List.size();
}

/*!
  Add a segment feature to the list of features to be considered in the pose
  computation.

  \param P1 : First extremity projection.
  \param P2 : Second extremity projection.
*/
void vpPoseFeatures::addFeatureSegment(vpPoint &P1, vpPoint &P2)
{
  featureSegment_DuoPoints_list.push_back(vpTrio<vpFeatureSegment, vpPoint, vpPoint>());
  featureSegment_DuoPoints_list.back().firstParam = P1;
  featureSegment_DuoPoints_list.back().secondParam = P2;
  featureSegment_DuoPoints_list.back().desiredFeature = new vpFeatureSegment();
  vpFeatureBuilder::create(*featureSegment_DuoPoints_list.back().desiredFeature, P1, P2);

  totalSize++;
  if (featureSegment_DuoPoints_list.size() > maxSize)
    maxSize = (unsigned int)featureSegment_DuoPoints_list.size();
}

/*!
  Get the error vector and L matrix from all the features.

  \param cMo : Current Pose.
  \param err : Resulting error vector.
  \param L : Resulting interaction matrix.
*/
void vpPoseFeatures::error_and_interaction(vpHomogeneousMatrix &cMo, vpColVector &err, vpMatrix &L)
{
  err = vpColVector();
  L = vpMatrix();

  for (unsigned int i = 0; i < maxSize; i++) {
    //--------------vpFeaturePoint--------------
    // From vpPoint
    if (i < featurePoint_Point_list.size()) {
      vpFeaturePoint fp;
      vpPoint p(featurePoint_Point_list[i].firstParam);
      p.track(cMo);
      vpFeatureBuilder::create(fp, p);
      err.stack(fp.error(*(featurePoint_Point_list[i].desiredFeature)));
      L.stack(fp.interaction());
    }

    //--------------vpFeaturePoint3D--------------
    // From vpPoint
    if (i < featurePoint3D_Point_list.size()) {
      vpFeaturePoint3D fp3D;
      vpPoint p(featurePoint3D_Point_list[i].firstParam);
      p.track(cMo);
      vpFeatureBuilder::create(fp3D, p);
      err.stack(fp3D.error(*(featurePoint3D_Point_list[i].desiredFeature)));
      L.stack(fp3D.interaction());
    }

    //--------------vpFeatureVanishingPoint--------------
    // From vpPoint
    if (i < featureVanishingPoint_Point_list.size()) {
      vpFeatureVanishingPoint fvp;
      vpPoint p(featureVanishingPoint_Point_list[i].firstParam);
      p.track(cMo);
      vpFeatureBuilder::create(fvp, p);
      err.stack(fvp.error(*(featureVanishingPoint_Point_list[i].desiredFeature)));
      L.stack(fvp.interaction());
    }
    // From Duo of vpLines
    if (i < featureVanishingPoint_DuoLine_list.size()) {
      vpFeatureVanishingPoint fvp;
      vpLine l1(featureVanishingPoint_DuoLine_list[i].firstParam);
      vpLine l2(featureVanishingPoint_DuoLine_list[i].secondParam);
      l1.track(cMo);
      l2.track(cMo);
      vpFeatureBuilder::create(fvp, l1, l2);
      err.stack(fvp.error(*(featureVanishingPoint_DuoLine_list[i].desiredFeature)));
      L.stack(fvp.interaction());
    }

    //--------------vpFeatureEllipse--------------
    // From vpSphere
    if (i < featureEllipse_Sphere_list.size()) {
      vpFeatureEllipse fe;
      vpSphere s(featureEllipse_Sphere_list[i].firstParam);
      s.track(cMo);
      vpFeatureBuilder::create(fe, s);
      err.stack(fe.error(*(featureEllipse_Sphere_list[i].desiredFeature)));
      L.stack(fe.interaction());
    }
    // From vpCircle
    if (i < featureEllipse_Circle_list.size()) {
      vpFeatureEllipse fe;
      vpCircle c(featureEllipse_Circle_list[i].firstParam);
      c.track(cMo);
      vpFeatureBuilder::create(fe, c);
      err.stack(fe.error(*(featureEllipse_Circle_list[i].desiredFeature)));
      L.stack(fe.interaction());
    }

    //--------------vpFeatureLine--------------
    // From vpLine
    if (i < featureLine_Line_list.size()) {
      vpFeatureLine fl;
      vpLine l(featureLine_Line_list[i].firstParam);
      l.track(cMo);
      vpFeatureBuilder::create(fl, l);
      err.stack(fl.error(*(featureLine_Line_list[i].desiredFeature)));
      L.stack(fl.interaction());
    }
    // From Duo of vpCylinder / Integer
    if (i < featureLine_DuoLineInt_List.size()) {
      vpFeatureLine fl;
      vpCylinder c(featureLine_DuoLineInt_List[i].firstParam);
      c.track(cMo);
      vpFeatureBuilder::create(fl, c, featureLine_DuoLineInt_List[i].secondParam);
      err.stack(fl.error(*(featureLine_DuoLineInt_List[i].desiredFeature)));
      L.stack(fl.interaction());
    }

    //--------------vpFeatureSegment--------------
    // From Duo of vpPoints
    if (i < featureSegment_DuoPoints_list.size()) {
      vpFeatureSegment fs;
      vpPoint p1(featureSegment_DuoPoints_list[i].firstParam);
      vpPoint p2(featureSegment_DuoPoints_list[i].secondParam);
      p1.track(cMo);
      p2.track(cMo);
      vpFeatureBuilder::create(fs, p1, p2);
      err.stack(fs.error(*(featureSegment_DuoPoints_list[i].desiredFeature)));
      L.stack(fs.interaction());
    }

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
    //--------------Specific Feature--------------
    if (i < featureSpecific_list.size()) {
      featureSpecific_list[i]->createCurrent(cMo);
      err.stack(featureSpecific_list[i]->error());
      L.stack(featureSpecific_list[i]->currentInteraction());
    }
#endif
  }
}

/*!
  Compute the pose according to the desired method (virtual visual servoing,
  or robust virtual visual servoing approach).

  \param cMo : Computed pose.

  \param type : Method to use for the pose computation.

  - The virtual visual servoing approach is described in \cite Marchand02c.

  - The robust virtual visual servoing approach is described in
  \cite Comport06b.

*/
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

/*!
  Compute the pose thanks to the virtual visual servoing approach.

  This approach is described in:

  E. Marchand, F. Chaumette. Virtual Visual Servoing: a framework for
  real-time augmented reality. In EUROGRAPHICS 2002 Conference Proceeding, G.
  Drettakis, H.-P. Seidel (eds.), Computer Graphics Forum, Volume 21(3), Pages
  289-298, Sarrebruck, Allemagne, 2002.

  \param cMo : Computed pose.
*/
void vpPoseFeatures::computePoseVVS(vpHomogeneousMatrix &cMo)
{
  try {
    double residu_1 = 1e8;
    double r = 1e8 - 1;
    // we stop the minimization when the error is bellow 1e-8

    vpMatrix L;
    vpColVector err;
    vpColVector v;

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
      unsigned int rank = L.pseudoInverse(Lp, 1e-16);

      if (rank < 6) {
        if (verbose)
          vpTRACE("Rank must be at least 6 ! cMo not computed.");

        break;
      }

      // compute the VVS control law
      v = -lambda * Lp * err;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;
      if (iter++ > vvsIterMax) {
        vpTRACE("Max iteration reached");
        break;
      }
    }

    if (computeCovariance)
      covarianceMatrix = vpMatrix::computeCovarianceMatrix(L, v, -lambda * err);

  } catch (...) {
    vpERROR_TRACE("vpPoseFeatures::computePoseVVS");
    throw;
  }
}

/*!
  Compute the pose thanks to the robust virtual visual servoing approach
  described in \cite Comport06b.

  \param cMo : Computed pose.
*/
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

    vpRobust robust(2 * totalSize);
    robust.setThreshold(0.0000);

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
      robust.setIteration(0);
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

      if (rank < 6) {
        if (verbose)
          vpTRACE("Rank must be at least 6 ! cMo not computed.");

        break;
      }

      // compute the VVS control law
      v = -lambda * Lp * W * error;

      cMo = vpExponentialMap::direct(v).inverse() * cMo;
      ;
      if (iter++ > vvsIterMax) {
        vpTRACE("Max iteration reached");
        break;
      }
    }

    if (computeCovariance)
      covarianceMatrix =
          vpMatrix::computeCovarianceMatrix(L, v, -lambda * error, W * W); // Remark: W*W = W*W.t() since the
                                                                           // matrix is diagonale, but using W*W
                                                                           // is more efficient.
  } catch (...) {
    vpERROR_TRACE("vpPoseFeatures::computePoseRobustVVS");
    throw;
  }
}

#endif //#ifdef VISP_HAVE_MODULE_VISUAL_FEATURES
