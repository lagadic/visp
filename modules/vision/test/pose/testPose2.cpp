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
 * Test pose computation methods.
 *
 *****************************************************************************/

#include <algorithm>    // std::transform
#include <map>
#include <visp3/core/vpGaussRand.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/vision/vpPose.h>

#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
#include <opencv2/calib3d.hpp>
#endif

namespace
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
using namespace cv;
#endif
using namespace std;

namespace posit
{
static vpHomogeneousMatrix pose_dementhon(const vector<vpColVector> &wX,
                                          const vector<vpColVector> &x)
{
  unsigned int npoints = static_cast<unsigned int>(wX.size());
  vpColVector r1, r2, r3;
  vpMatrix A(npoints, 4);
  for(unsigned int i = 0; i < npoints; i++) {
    for (unsigned int j = 0; j < 4; j++) {
      A[i][j] = wX[i][j];
    }
  }

  vpMatrix Ap = A.pseudoInverse();
  vpColVector eps(npoints);
  eps = 0; // Initialize epsilon_i = 0
  vpColVector Bx(npoints);
  vpColVector By(npoints);
  double tx = 0.0, ty = 0.0, tz = 0.0;
  vpMatrix I, J;
  vpColVector Istar(3), Jstar(3);

  // POSIT loop
  for (unsigned int iter = 0; iter < 20; iter++) {
    for (unsigned int i = 0; i < npoints; i++) {
      Bx[i] = x[i][0] * (eps[i] + 1.);
      By[i] = x[i][1] * (eps[i] + 1.);
    }

    I = Ap * Bx; // Notice that the pseudo inverse
    J = Ap * By; // of matrix A is a constant that has been precompiled.
    for (unsigned int i = 0; i < 3; i++) {
      Istar[i] = I[i][0];
      Jstar[i] = J[i][0];
    }

    // Estimation of the rotation matrix
    double normI = sqrt( Istar.sumSquare() );
    double normJ = sqrt( Jstar.sumSquare() );
    r1 = Istar / normI;
    r2 = Jstar / normJ;
    r3 = vpColVector::crossProd(r1, r2);

    // Estimation of the translation
    tz = 1/normI;
    tx = tz * I[3][0];
    ty = tz * J[3][0];

    // Update epsilon_i
    for(unsigned int i = 0; i < npoints; i++) {
      eps[i] = (r3[0] * wX[i][0] + r3[1] * wX[i][1] + r3[2] * wX[i][2]) / tz;
    }
  }

  vpHomogeneousMatrix cTw;
  // Update translation vector
  cTw[0][3] = tx;
  cTw[1][3] = ty;
  cTw[2][3] = tz;

  // update rotation matrix
  for (unsigned int i = 0; i < 3; i++) {
    cTw[0][i] = r1[i];
    cTw[1][i] = r2[i];
    cTw[2][i] = r3[i];
  }

  return cTw;
}

static vpHomogeneousMatrix pose_dementhon(const vector<vpPoint>& points, bool centered)
{
  vector<vpColVector> wX(points.size()), x(points.size());

  double cog_x = 0.0, cog_y = 0.0, cog_z = 0.0;
  if (centered) {
    /* compute the cog of the 3D points */
    for (size_t i = 0; i < points.size(); i++) {
      const vpPoint& pt = points[i];
      cog_x += pt.get_oX();
      cog_y += pt.get_oY();
      cog_z += pt.get_oZ();
    }

    cog_x /= points.size();
    cog_y /= points.size();
    cog_z /= points.size();
  }

  for (size_t i = 0; i < points.size(); i++) {
    const vpPoint& pt = points[i];

    if (centered) {
      wX[i] = {pt.get_oX() - cog_x, pt.get_oY() - cog_y, pt.get_oZ() - cog_z, 1.0};
      x[i] = {pt.get_x(), pt.get_y(), 1.0};
    } else {
      wX[i] = {pt.get_oX(), pt.get_oY(), pt.get_oZ(), 1.0};
      x[i] = {pt.get_x(), pt.get_y(), 1.0};
    }
  }

  vpHomogeneousMatrix cMo = pose_dementhon(wX, x);
  if (centered) {
    // go back to the initial frame
    cMo[0][3] -= (cog_x * cMo[0][0] + cog_y * cMo[0][1] + cog_z * cMo[0][2]);
    cMo[1][3] -= (cog_x * cMo[1][0] + cog_y * cMo[1][1] + cog_z * cMo[1][2]);
    cMo[2][3] -= (cog_x * cMo[2][0] + cog_y * cMo[2][1] + cog_z * cMo[2][2]);
  }


  return cMo;
}
} //namespace posit

static double randInterval(double a, double b, vpUniRand& randGen)
{
  return a + (b-a) * randGen();
}

static vpHomogeneousMatrix generatePose(const vector<vpPoint>& points, vpUniRand& randGen, int nbTrials=10)
{
  vpHomogeneousMatrix cMo;

  bool validPose = false;
  for (int trial = 0; trial < nbTrials && !validPose; trial++) {
    vpThetaUVector tu(randInterval(-1,1,randGen), randInterval(-1,1,randGen), randInterval(-1,1,randGen));
    vpTranslationVector t(randInterval(-1,1,randGen), randInterval(-1,1,randGen), randInterval(0.5,2,randGen));
    cMo.buildFrom(t, tu);

    bool positiveDepth = true;
    for (size_t i = 0; i < points.size() && positiveDepth; i++) {
      vpPoint pt = points[i];
      pt.changeFrame(cMo);
      if (pt.get_Z() <= 0) {
        positiveDepth = false;
      }
    }
    validPose = positiveDepth;
  }

  return  cMo;
}

static double max3(double a, double b, double c)
{
  double max1 = (a < b) ? b : a;
  return (max1 < c) ? c : max1;
}

static double safeAcos(double x)
{
  if (x < -1.0) x = -1.0;
  else if (x > 1.0) x = 1.0;
  return acos (x) ;
}

static double getRotationError(const vpHomogeneousMatrix& cMo_ref, const vpHomogeneousMatrix& cMo_est)
{
  vpRotationMatrix R_ref(cMo_ref), R_est(cMo_est);
  double err0 = safeAcos(R_ref.getCol(0).t() * R_est.getCol(0)) * 180 / M_PI;
  double err1 = safeAcos(R_ref.getCol(1).t() * R_est.getCol(1)) * 180 / M_PI;
  double err2 = safeAcos(R_ref.getCol(2).t() * R_est.getCol(2)) * 180 / M_PI;

  return max3(err0, err1, err2);
}

static double getTranslationError(const vpHomogeneousMatrix& cMo_ref, const vpHomogeneousMatrix& cMo_est)
{
  vpTranslationVector t_ref(cMo_ref), t_est(cMo_est);
  return sqrt((t_ref - t_est).sumSquare()) / sqrt(t_est.sumSquare()) * 100;
}

static void getTimeInfo(const vector<double>& times, double& meanTime, double& medianTime, double& stdTime)
{
  if (!times.empty()) {
    meanTime = vpMath::getMean(times);
    medianTime = vpMath::getMedian(times);
    stdTime = vpMath::getStdev(times);
  }
}

struct ErrorInfo
{
  double rotationError;
  double translationError;

  ErrorInfo(double rotErr, double tErr) : rotationError(rotErr), translationError(tErr) {}
};

static double opExtractRotation(const ErrorInfo& err)
{
  return err.rotationError;
}

static double opExtractTranslation(const ErrorInfo& err)
{
  return err.translationError;
}

static void getErrorMean(const vector<ErrorInfo>& errors, double& rotationMeanError, double& translationMeanError)
{
  vector<double> rotationErrors(errors.size()), translationErrors(errors.size());
  transform(errors.begin(), errors.end(), rotationErrors.begin(), opExtractRotation);
  transform(errors.begin(), errors.end(), translationErrors.begin(), opExtractTranslation);

  rotationMeanError = vpMath::getMean(rotationErrors);
  translationMeanError = vpMath::getMean(translationErrors);
}

static void getErrorMedian(const vector<ErrorInfo>& errors, double& rotationMedianError, double& translationMedianError)
{
  vector<double> rotationErrors(errors.size()), translationErrors(errors.size());
  transform(errors.begin(), errors.end(), rotationErrors.begin(), opExtractRotation);
  transform(errors.begin(), errors.end(), translationErrors.begin(), opExtractTranslation);

  rotationMedianError = vpMath::getMedian(rotationErrors);
  translationMedianError = vpMath::getMedian(translationErrors);
}

static void getErrorStd(const vector<ErrorInfo>& errors, double& rotationStdError, double& translationStdError)
{
  vector<double> rotationErrors(errors.size()), translationErrors(errors.size());
  transform(errors.begin(), errors.end(), rotationErrors.begin(), opExtractRotation);
  transform(errors.begin(), errors.end(), translationErrors.begin(), opExtractTranslation);

  rotationStdError = vpMath::getStdev(rotationErrors);
  translationStdError = vpMath::getStdev(translationErrors);
}

static ErrorInfo getPoseError(const vpHomogeneousMatrix& cMo_ref, const vpHomogeneousMatrix& cMo_est)
{
  return ErrorInfo(getRotationError(cMo_ref, cMo_est), getTranslationError(cMo_ref, cMo_est));
}

static void runTest(vector<vpPoint> points,
                    pair<vector<ErrorInfo>, vector<double> >& errorsDementhon,
                    pair<vector<ErrorInfo>, vector<double> >& errorsPOSIT,
                    pair<vector<ErrorInfo>, vector<double> >& errorsPOSIT_centered,
                    pair<vector<ErrorInfo>, vector<double> >& errorsLagrange,
                    pair<vector<ErrorInfo>, vector<double> >& errorsEPPnP,
                    pair<vector<ErrorInfo>, vector<double> >& errorsEPnP,
                    pair<vector<ErrorInfo>, vector<double> >& errorsDementhonVVS,
                    pair<vector<ErrorInfo>, vector<double> >& errorsLagrangeVVS,
                    int gaussianSigma=0, int nbTrials=100)
{
  vpUniRand randGen;
  vpGaussRand gaussRandGen(gaussianSigma, 0.0);
  vpCameraParameters cam(600.0, 600.0, 320.0, 240.0);

  vector<Point3d> cv_objectPoints(points.size());
  for (size_t i = 0; i < points.size(); i++) {
    const vpPoint& pt = points[i];
    cv_objectPoints[i] = Point3d(pt.get_oX(), pt.get_oY(), pt.get_oZ());
  }
  vector<Point2d> cv_imagePoints(points.size());

  for (int trial = 0; trial < nbTrials; trial++) {
    vpHomogeneousMatrix cMo_ref = generatePose(points, randGen);
    for (size_t i = 0; i < points.size(); i++) {
      points[i].project(cMo_ref);

      if (gaussianSigma > 0) {
        vpImagePoint imPt;
        vpMeterPixelConversion::convertPoint(cam, points[i].get_x(), points[i].get_y(), imPt);
        imPt.set_uv(imPt.get_u() + gaussRandGen(), imPt.get_v() + gaussRandGen());

        double x = 0.0, y = 0.0;
        vpPixelMeterConversion::convertPoint(cam, imPt.get_u(), imPt.get_v(), x, y);
        points[i].set_x(x);
        points[i].set_y(y);
      }

      cv_imagePoints[i].x = points[i].get_x();
      cv_imagePoints[i].y = points[i].get_y();
    }
    {
      vpHomogeneousMatrix cMo_dementhon;
      vpPose pose;
      pose.addPoints(points);
      try {
        double t = vpTime::measureTimeMs();
        pose.computePose(vpPose::DEMENTHON, cMo_dementhon);
        t = vpTime::measureTimeMs() - t;
        errorsDementhon.first.push_back(getPoseError(cMo_ref, cMo_dementhon));
        errorsDementhon.second.push_back(t);
      } catch (...) {}
    }
    {
      const bool centered = false;
      double t = vpTime::measureTimeMs();
      vpHomogeneousMatrix cMo_POSIT = posit::pose_dementhon(points, centered);
      t = vpTime::measureTimeMs() - t;
      errorsPOSIT.first.push_back(getPoseError(cMo_ref, cMo_POSIT));
      errorsPOSIT.second.push_back(t);
    }
    {
      const bool centered = true;
      double t = vpTime::measureTimeMs();
      vpHomogeneousMatrix cMo_POSIT_centered = posit::pose_dementhon(points, centered);
      t = vpTime::measureTimeMs() - t;
      errorsPOSIT_centered.first.push_back(getPoseError(cMo_ref, cMo_POSIT_centered));
      errorsPOSIT_centered.second.push_back(t);
    }
    {
      vpHomogeneousMatrix cMo_lagrange;
      vpPose pose;
      pose.addPoints(points);
      try {
        double t = vpTime::measureTimeMs();
        pose.computePose(vpPose::LAGRANGE, cMo_lagrange);
        t = vpTime::measureTimeMs() - t;
        errorsLagrange.first.push_back(getPoseError(cMo_ref, cMo_lagrange));
        errorsLagrange.second.push_back(t);
      } catch (...) {}
    }
    {
      vpHomogeneousMatrix cMo_EPPnP;
      vpPose pose;
      pose.addPoints(points);
      try {
        double t = vpTime::measureTimeMs();
        pose.computePose(vpPose::EPPnP, cMo_EPPnP);
        t = vpTime::measureTimeMs() - t;
        errorsEPPnP.first.push_back(getPoseError(cMo_ref, cMo_EPPnP));
        errorsEPPnP.second.push_back(t);
      } catch (...) {}
    }
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    {
      Matx31d rvec, tvec;
      double t = vpTime::measureTimeMs();
      bool success = solvePnP(cv_objectPoints, cv_imagePoints, Matx33d::eye(), noArray(), rvec, tvec, false, SOLVEPNP_EPNP);
      t = vpTime::measureTimeMs() - t;
      if (success) {
        vpHomogeneousMatrix cMo_EPnP(vpTranslationVector(tvec(0), tvec(1), tvec(2)), vpThetaUVector(rvec(0), rvec(1), rvec(2)));
        errorsEPnP.first.push_back(getPoseError(cMo_ref, cMo_EPnP));
        errorsEPnP.second.push_back(t);
      }
    }
#endif
    {
      vpHomogeneousMatrix cMo_dementhonVVS;
      vpPose pose;
      pose.addPoints(points);
      try {
        double t = vpTime::measureTimeMs();
        pose.computePose(vpPose::DEMENTHON_VIRTUAL_VS, cMo_dementhonVVS);
        t = vpTime::measureTimeMs() - t;
        errorsDementhonVVS.first.push_back(getPoseError(cMo_ref, cMo_dementhonVVS));
        errorsDementhonVVS.second.push_back(t);
      } catch (...) {}
    }
    {
      vpHomogeneousMatrix cMo_lagrangeVVS;
      vpPose pose;
      pose.addPoints(points);
      try {
        double t = vpTime::measureTimeMs();
        pose.computePose(vpPose::LAGRANGE_VIRTUAL_VS, cMo_lagrangeVVS);
        t = vpTime::measureTimeMs() - t;
        errorsLagrangeVVS.first.push_back(getPoseError(cMo_ref, cMo_lagrangeVVS));
        errorsLagrangeVVS.second.push_back(t);
      } catch (...) {}
    }
  }
}

static vector<vpPoint> generateRandomObjectPointsSkew(int nbPoints, vpUniRand& randGen, double size=0.2)
{
  vector<vpPoint> points(static_cast<size_t>(nbPoints));

  for (int i = 0; i < nbPoints; i++) {
    points.push_back(vpPoint(randInterval(0, 2*size, randGen),
                             randInterval(0, 2*size, randGen),
                             randInterval(0, 2*size, randGen)));
  }

  return  points;
}

static vector<vpPoint> generateRandomObjectPoints(int nbPoints, vpUniRand& randGen, double size=0.2)
{
  vector<vpPoint> points(static_cast<size_t>(nbPoints));

  for (int i = 0; i < nbPoints; i++) {
    points.push_back(vpPoint(randInterval(-size, size, randGen),
                             randInterval(-size, size, randGen),
                             randInterval(-size, size, randGen)));
  }

  return  points;
}

struct TestResults
{
  double meanRotationError;
  double meanTranslationError;
  double medianRotationError;
  double medianTranslationError;
  double stdRotationError;
  double stdTranslationError;
  double meanComputationTime;
  double medianComputationTime;
  double stdComputationTime;

  TestResults(double meanRot, double meanTrans,
              double medianRot, double medianTrans,
              double stdRot, double stdTrans,
              double meanTime, double medianTime, double stdTime) :
    meanRotationError(meanRot), meanTranslationError(meanTrans), medianRotationError(medianRot),
    medianTranslationError(medianTrans), stdRotationError(stdRot), stdTranslationError(stdTrans),
    meanComputationTime(meanTime), medianComputationTime(medianTime), stdComputationTime(stdTime) {}

  TestResults() :
    meanRotationError(0), meanTranslationError(0), medianRotationError(0),
    medianTranslationError(0), stdRotationError(0), stdTranslationError(0),
    meanComputationTime(0), medianComputationTime(0), stdComputationTime(0) {}
};

static void runTestNonPlanar(bool skew, int minPoints=4, int maxPoints=100)
{
  vpUniRand randGen;

  map<int, map<string, TestResults> > mapOfResults;
  for (int nbPoints = minPoints; nbPoints <= maxPoints; nbPoints++) {
    vector<vpPoint> points;
    if (skew) {
      points = generateRandomObjectPointsSkew(nbPoints, randGen);
    } else {
      points = generateRandomObjectPoints(nbPoints, randGen);
    }

    pair<vector<ErrorInfo>, vector<double> > errorsDementhon, errorsPOSIT, errorsPOSIT_centered, errorsLagrange, errorsEPPnP, errorsEPnP;
    pair<vector<ErrorInfo>, vector<double> > errorsDementhonVVS, errorsLagrangeVVS;
    const int gaussianSigma = 2; //constant Gaussian noise of 2
    runTest(points, errorsDementhon, errorsPOSIT, errorsPOSIT_centered, errorsLagrange, errorsEPPnP, errorsEPnP,
            errorsDementhonVVS, errorsLagrangeVVS, gaussianSigma);

    map<string, TestResults> mapOfTestResults;
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsDementhon.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsDementhon.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsDementhon.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsDementhon.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Dementhon"] = TestResults(rotErrMean, transErrMean,
                                                  rotErrMedian, transErrMedian,
                                                  rotErrStd, transErrStd,
                                                  meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsPOSIT.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsPOSIT.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsPOSIT.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsPOSIT.second, meanTime, medianTime, stdTime);

      mapOfTestResults["POSIT"] = TestResults(rotErrMean, transErrMean,
                                              rotErrMedian, transErrMedian,
                                              rotErrStd, transErrStd,
                                              meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsPOSIT_centered.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsPOSIT_centered.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsPOSIT_centered.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsPOSIT_centered.second, meanTime, medianTime, stdTime);

      mapOfTestResults["POSIT centered"] = TestResults(rotErrMean, transErrMean,
                                                       rotErrMedian, transErrMedian,
                                                       rotErrStd, transErrStd,
                                                       meanTime, medianTime, stdTime);
    }
    if (nbPoints >= 6)
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsLagrange.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsLagrange.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsLagrange.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsLagrange.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Lagrange"] = TestResults(rotErrMean, transErrMean,
                                                 rotErrMedian, transErrMedian,
                                                 rotErrStd, transErrStd,
                                                 meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsEPPnP.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsEPPnP.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsEPPnP.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsEPPnP.second, meanTime, medianTime, stdTime);

      mapOfTestResults["EPPnP"] = TestResults(rotErrMean, transErrMean,
                                              rotErrMedian, transErrMedian,
                                              rotErrStd, transErrStd,
                                              meanTime, medianTime, stdTime);
    }
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsEPnP.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsEPnP.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsEPnP.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsEPnP.second, meanTime, medianTime, stdTime);

      mapOfTestResults["EPnP"] = TestResults(rotErrMean, transErrMean,
                                             rotErrMedian, transErrMedian,
                                             rotErrStd, transErrStd,
                                             meanTime, medianTime, stdTime);
    }
#endif
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsDementhonVVS.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsDementhonVVS.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsDementhonVVS.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsDementhonVVS.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Dementhon VVS"] = TestResults(rotErrMean, transErrMean,
                                                      rotErrMedian, transErrMedian,
                                                      rotErrStd, transErrStd,
                                                      meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsLagrangeVVS.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsLagrangeVVS.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsLagrangeVVS.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsLagrangeVVS.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Lagrange VVS"] = TestResults(rotErrMean, transErrMean,
                                                     rotErrMedian, transErrMedian,
                                                     rotErrStd, transErrStd,
                                                     meanTime, medianTime, stdTime);
    }

    mapOfResults[nbPoints] = mapOfTestResults;
  }

  vpPlot plot(3);
  plot.initGraph(0, 8);
  plot.setLegend(0, 0, "Dementhon");
  plot.setLegend(0, 1, "POSIT");
  plot.setLegend(0, 2, "POSIT centered");
  plot.setLegend(0, 3, "Lagrange");
  plot.setLegend(0, 4, "EPPnP");
  plot.setLegend(0, 5, "EPnP");
  plot.setLegend(0, 6, "Dementhon VVS");
  plot.setLegend(0, 7, "Lagrange VVS");

  plot.initGraph(1, 8);
  plot.setLegend(1, 0, "Dementhon");
  plot.setLegend(1, 1, "POSIT");
  plot.setLegend(1, 2, "POSIT centered");
  plot.setLegend(1, 3, "Lagrange");
  plot.setLegend(1, 4, "EPPnP");
  plot.setLegend(1, 5, "EPnP");
  plot.setLegend(1, 6, "Dementhon VVS");
  plot.setLegend(1, 7, "Lagrange VVS");

  plot.initGraph(2, 8);
  plot.setLegend(2, 0, "Dementhon");
  plot.setLegend(2, 1, "POSIT");
  plot.setLegend(2, 2, "POSIT centered");
  plot.setLegend(2, 3, "Lagrange");
  plot.setLegend(2, 4, "EPPnP");
  plot.setLegend(2, 5, "EPnP");
  plot.setLegend(2, 6, "Dementhon VVS");
  plot.setLegend(2, 7, "Lagrange VVS");

  for (map<int, map<string, TestResults> >::const_iterator it1 = mapOfResults.begin(); it1 != mapOfResults.end(); ++it1) {
    map<string, TestResults>::const_iterator it2 = it1->second.find("Dementhon");
    if (it2 != it1->second.end()) {
      plot.plot(0, 0, it1->first, it2->second.medianRotationError);
      plot.plot(1, 0, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 0, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("POSIT");
    if (it2 != it1->second.end()) {
      plot.plot(0, 1, it1->first, it2->second.medianRotationError);
      plot.plot(1, 1, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 1, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("POSIT centered");
    if (it2 != it1->second.end()) {
      plot.plot(0, 2, it1->first, it2->second.medianRotationError);
      plot.plot(1, 2, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 2, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("Lagrange");
    if (it2 != it1->second.end()) {
      plot.plot(0, 3, it1->first, it2->second.medianRotationError);
      plot.plot(1, 3, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 3, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("EPPnP");
    if (it2 != it1->second.end()) {
      plot.plot(0, 4, it1->first, it2->second.medianRotationError);
      plot.plot(1, 4, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 4, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("EPnP");
    if (it2 != it1->second.end()) {
      plot.plot(0, 5, it1->first, it2->second.medianRotationError);
      plot.plot(1, 5, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 5, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("Dementhon VVS");
    if (it2 != it1->second.end()) {
      plot.plot(0, 6, it1->first, it2->second.medianRotationError);
      plot.plot(1, 6, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 6, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("Lagrange VVS");
    if (it2 != it1->second.end()) {
      plot.plot(0, 7, it1->first, it2->second.medianRotationError);
      plot.plot(1, 7, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 7, it1->first, it2->second.medianComputationTime);
    }
  }

  if (skew) {
    plot.saveData(0, "TestNonPlanar_rotationErrors_skew.txt");
    plot.saveData(1, "TestNonPlanar_translationErrors_skew.txt");
    plot.saveData(2, "TestNonPlanar_computationTime_skew.txt");
  } else {
    plot.saveData(0, "TestNonPlanar_rotationErrors.txt");
    plot.saveData(1, "TestNonPlanar_translationErrors.txt");
    plot.saveData(2, "TestNonPlanar_computationTime.txt");
  }
}

static void runTestNonPlanarGaussianNoise(int nbPoints, bool skew, int minGaussianSigma=0, int maxGaussianSigma=20)
{
  vpUniRand randGen;

  map<int, map<string, TestResults> > mapOfResults;
  for (int gaussianSigma = minGaussianSigma; gaussianSigma <= maxGaussianSigma; gaussianSigma++) {
    vector<vpPoint> points;
    if (skew) {
      points = generateRandomObjectPointsSkew(nbPoints, randGen);
    } else {
      points = generateRandomObjectPoints(nbPoints, randGen);
    }

    pair<vector<ErrorInfo>, vector<double> > errorsDementhon, errorsPOSIT, errorsPOSIT_centered, errorsLagrange, errorsEPPnP, errorsEPnP;
    pair<vector<ErrorInfo>, vector<double> > errorsDementhonVVS, errorsLagrangeVVS;
    runTest(points, errorsDementhon, errorsPOSIT, errorsPOSIT_centered, errorsLagrange, errorsEPPnP, errorsEPnP,
            errorsDementhonVVS, errorsLagrangeVVS, gaussianSigma);

    map<string, TestResults> mapOfTestResults;
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsDementhon.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsDementhon.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsDementhon.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsDementhon.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Dementhon"] = TestResults(rotErrMean, transErrMean,
                                                  rotErrMedian, transErrMedian,
                                                  rotErrStd, transErrStd,
                                                  meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsPOSIT.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsPOSIT.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsPOSIT.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsPOSIT.second, meanTime, medianTime, stdTime);

      mapOfTestResults["POSIT"] = TestResults(rotErrMean, transErrMean,
                                              rotErrMedian, transErrMedian,
                                              rotErrStd, transErrStd,
                                              meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsPOSIT_centered.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsPOSIT_centered.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsPOSIT_centered.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsPOSIT_centered.second, meanTime, medianTime, stdTime);

      mapOfTestResults["POSIT centered"] = TestResults(rotErrMean, transErrMean,
                                                       rotErrMedian, transErrMedian,
                                                       rotErrStd, transErrStd,
                                                       meanTime, medianTime, stdTime);
    }
    if (nbPoints >= 6)
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsLagrange.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsLagrange.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsLagrange.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsLagrange.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Lagrange"] = TestResults(rotErrMean, transErrMean,
                                                 rotErrMedian, transErrMedian,
                                                 rotErrStd, transErrStd,
                                                 meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsEPPnP.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsEPPnP.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsEPPnP.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsEPPnP.second, meanTime, medianTime, stdTime);

      mapOfTestResults["EPPnP"] = TestResults(rotErrMean, transErrMean,
                                              rotErrMedian, transErrMedian,
                                              rotErrStd, transErrStd,
                                              meanTime, medianTime, stdTime);
    }
#if (VISP_HAVE_OPENCV_VERSION >= 0x030000)
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsEPnP.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsEPnP.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsEPnP.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsEPnP.second, meanTime, medianTime, stdTime);

      mapOfTestResults["EPnP"] = TestResults(rotErrMean, transErrMean,
                                             rotErrMedian, transErrMedian,
                                             rotErrStd, transErrStd,
                                             meanTime, medianTime, stdTime);
    }
#endif
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsDementhonVVS.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsDementhonVVS.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsDementhonVVS.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsDementhonVVS.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Dementhon VVS"] = TestResults(rotErrMean, transErrMean,
                                                      rotErrMedian, transErrMedian,
                                                      rotErrStd, transErrStd,
                                                      meanTime, medianTime, stdTime);
    }
    {
      double rotErrMean = 0.0, transErrMean = 0.0;
      getErrorMean(errorsLagrangeVVS.first, rotErrMean, transErrMean);
      double rotErrMedian = 0.0, transErrMedian = 0.0;
      getErrorMedian(errorsLagrangeVVS.first, rotErrMedian, transErrMedian);
      double rotErrStd = 0.0, transErrStd = 0.0;
      getErrorStd(errorsLagrangeVVS.first, rotErrStd, transErrStd);
      double meanTime = 0.0, medianTime = 0.0, stdTime = 0.0;
      getTimeInfo(errorsLagrangeVVS.second, meanTime, medianTime, stdTime);

      mapOfTestResults["Lagrange VVS"] = TestResults(rotErrMean, transErrMean,
                                                     rotErrMedian, transErrMedian,
                                                     rotErrStd, transErrStd,
                                                     meanTime, medianTime, stdTime);
    }

    mapOfResults[gaussianSigma] = mapOfTestResults;
  }

  vpPlot plot(3);
  plot.initGraph(0, 8);
  plot.setLegend(0, 0, "Dementhon");
  plot.setLegend(0, 1, "POSIT");
  plot.setLegend(0, 2, "POSIT centered");
  plot.setLegend(0, 3, "Lagrange");
  plot.setLegend(0, 4, "EPPnP");
  plot.setLegend(0, 5, "EPnP");
  plot.setLegend(0, 6, "Dementhon VVS");
  plot.setLegend(0, 7, "Lagrange VVS");

  plot.initGraph(1, 8);
  plot.setLegend(1, 0, "Dementhon");
  plot.setLegend(1, 1, "POSIT");
  plot.setLegend(1, 2, "POSIT centered");
  plot.setLegend(1, 3, "Lagrange");
  plot.setLegend(1, 4, "EPPnP");
  plot.setLegend(1, 5, "EPnP");
  plot.setLegend(1, 6, "Dementhon VVS");
  plot.setLegend(1, 7, "Lagrange VVS");

  plot.initGraph(2, 8);
  plot.setLegend(2, 0, "Dementhon");
  plot.setLegend(2, 1, "POSIT");
  plot.setLegend(2, 2, "POSIT centered");
  plot.setLegend(2, 3, "Lagrange");
  plot.setLegend(2, 4, "EPPnP");
  plot.setLegend(2, 5, "EPnP");
  plot.setLegend(2, 6, "Dementhon VVS");
  plot.setLegend(2, 7, "Lagrange VVS");

  for (map<int, map<string, TestResults> >::const_iterator it1 = mapOfResults.begin(); it1 != mapOfResults.end(); ++it1) {
    map<string, TestResults>::const_iterator it2 = it1->second.find("Dementhon");
    if (it2 != it1->second.end()) {
      plot.plot(0, 0, it1->first, it2->second.medianRotationError);
      plot.plot(1, 0, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 0, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("POSIT");
    if (it2 != it1->second.end()) {
      plot.plot(0, 1, it1->first, it2->second.medianRotationError);
      plot.plot(1, 1, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 1, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("POSIT centered");
    if (it2 != it1->second.end()) {
      plot.plot(0, 2, it1->first, it2->second.medianRotationError);
      plot.plot(1, 2, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 2, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("Lagrange");
    if (it2 != it1->second.end()) {
      plot.plot(0, 3, it1->first, it2->second.medianRotationError);
      plot.plot(1, 3, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 3, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("EPPnP");
    if (it2 != it1->second.end()) {
      plot.plot(0, 4, it1->first, it2->second.medianRotationError);
      plot.plot(1, 4, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 4, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("EPnP");
    if (it2 != it1->second.end()) {
      plot.plot(0, 5, it1->first, it2->second.medianRotationError);
      plot.plot(1, 5, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 5, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("Dementhon VVS");
    if (it2 != it1->second.end()) {
      plot.plot(0, 6, it1->first, it2->second.medianRotationError);
      plot.plot(1, 6, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 6, it1->first, it2->second.medianComputationTime);
    }

    it2 = it1->second.find("Lagrange VVS");
    if (it2 != it1->second.end()) {
      plot.plot(0, 7, it1->first, it2->second.medianRotationError);
      plot.plot(1, 7, it1->first, it2->second.medianTranslationError);
      plot.plot(2, 7, it1->first, it2->second.medianComputationTime);
    }
  }

  if (skew) {
    plot.saveData(0, "TestNonPlanar_GaussianNoise_rotationErrors_skew.txt");
    plot.saveData(1, "TestNonPlanar_GaussianNoise_translationErrors_skew.txt");
    plot.saveData(2, "TestNonPlanar_GaussianNoise_computationTime_skew.txt");
  } else {
    plot.saveData(0, "TestNonPlanar_GaussianNoise_rotationErrors.txt");
    plot.saveData(1, "TestNonPlanar_GaussianNoise_translationErrors.txt");
    plot.saveData(2, "TestNonPlanar_GaussianNoise_computationTime.txt");
  }
}

} //namespace

int main()
{
  {
    const bool skew = false;
    const int minPoints = 6;
    runTestNonPlanar(skew, minPoints);
  }
  {
    const bool skew = false;
    const int nbPoints = 30;
    runTestNonPlanarGaussianNoise(nbPoints, skew);
  }
  {
    const bool skew = true;
    const int minPoints = 6;
    runTestNonPlanar(skew, minPoints);
  }
  {
    const bool skew = true;
    const int nbPoints = 30;
    runTestNonPlanarGaussianNoise(nbPoints, skew);
  }

  return EXIT_SUCCESS;
}
