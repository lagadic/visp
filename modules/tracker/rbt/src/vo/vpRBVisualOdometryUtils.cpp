#include <visp3/rbt/vpRBVisualOdometryUtils.h>

#include <visp3/core/vpMatrix.h>
#include <visp3/rbt/vpRBFeatureTrackerInput.h>
#include <vector>
#include <utility>

#include <visp3/core/vpRobust.h>
#include <visp3/core/vpExponentialMap.h>
#include <visp3/core/vpHomogeneousMatrix.h>



std::pair<std::vector<unsigned int>, std::vector<unsigned int>>
vpRBVisualOdometryUtils::computeIndicesObjectAndEnvironment(
    const vpMatrix &keypoints, const vpRBFeatureTrackerInput &frame,
    double minMaskConfidence, double minEdgeDistObject, double minEdgeDistEnv)
{
  std::vector<unsigned int> objIndices, envIndices;
  const vpImage<float> &renderDepth = frame.renders.depth;
  const unsigned int h = renderDepth.getHeight(), w = renderDepth.getWidth();
  const bool useMask = frame.hasMask() && minMaskConfidence > 0.0;
  const double thresholdObject = vpMath::sqr(minEdgeDistObject), thresholdEnv = vpMath::sqr(minEdgeDistEnv);


  const auto testDistanceEdge = [](double u, double v, const std::vector<vpRBSilhouettePoint> &silhouettePoints, double threshold) -> bool {
    for (const vpRBSilhouettePoint &p: silhouettePoints) {
      double dist2 = vpMath::sqr(static_cast<double>(p.i) - v) + vpMath::sqr(static_cast<double>(p.j) - u);
      if (dist2 < threshold) {
        return false;
      }
    }
    return true;
    };

  for (unsigned int i = 0; i < keypoints.getRows(); ++i) {
    double u = keypoints[i][0], v = keypoints[i][1];
    if (u < 0.0 || v < 0.0 || u >= w || v >= h) {
      continue;
    }
    const unsigned int ui = static_cast<unsigned int>(u), vi = static_cast<unsigned int>(v);
    const double Z = static_cast<double>(renderDepth[vi][ui]);

    if (Z > 0.0) { // Potential object candidate
      if (!useMask || frame.mask[vi][ui] > minMaskConfidence) {
        bool notTooCloseToEdge = testDistanceEdge(u, v, frame.silhouettePoints, thresholdObject);
        if (notTooCloseToEdge) {
          objIndices.push_back(i);
        }
      }
    }
    else { // Env candidate
      bool notTooCloseToEdge = testDistanceEdge(u, v, frame.silhouettePoints, thresholdEnv);
      if (notTooCloseToEdge) {
        envIndices.push_back(i);
      }
    }
  }
  return std::make_pair(objIndices, envIndices);
}

void vpRBVisualOdometryUtils::levenbergMarquardtKeypoints2D(const vpMatrix &points3d, const vpMatrix &observations, const vpLevenbergMarquardtParameters &parameters, vpHomogeneousMatrix &cTw)
{
  vpMatrix L(points3d.getRows() * 2, 6);
  vpMatrix Lt(6, points3d.getRows());
  vpColVector e(points3d.getRows() * 2);
  vpColVector weights(points3d.getRows() * 2);
  vpColVector weighted_error(points3d.getRows() * 2);


  vpMatrix Id(6, 6);
  Id.eye();
  vpRobust robust;
  vpMatrix H(6, 6);
  double mu = parameters.muInit;
  double errorNormPrev = std::numeric_limits<double>::max();
  for (unsigned int iter = 0; iter < parameters.maxNumIters; ++iter) {
    const vpTranslationVector t = cTw.getTranslationVector();
    const vpRotationMatrix cRw = cTw.getRotationMatrix();
    // Project 3D points and compute interaction matrix
    for (unsigned int i = 0; i < points3d.getRows(); ++i) {
      vpColVector cX(3), wX(3);
      wX[0] = points3d[i][0]; wX[1] = points3d[i][1]; wX[2] = points3d[i][2];

      cX = cRw * wX;
      cX += t;

      const double x = cX[0] / cX[2], y = cX[1] / cX[2];
      const double Z = cX[2];
      e[i * 2] = x - observations[i][0];
      e[i * 2 + 1] = y - observations[i][1];

      L[i * 2][0] = -1.0 / Z; L[i * 2][1] = 0.0; L[i * 2][2] = x / Z;
      L[i * 2][3] = x * y; L[i * 2][4] = -(1.0 + x * x); L[i * 2][5] = y;

      L[i * 2 + 1][0] = 0.0; L[i * 2 + 1][1] = -1.0 / Z; L[i * 2 + 1][2] = y / Z;
      L[i * 2 + 1][3] = 1.0 + y * y; L[i * 2 + 1][4] = -(x * y); L[i * 2 + 1][5] = -x;
    }

    robust.MEstimator(vpRobust::TUKEY, e, weights);
    for (unsigned int i = 0; i < e.getRows(); ++i) {
      weighted_error[i] = e[i] * weights[i];
      for (unsigned int j = 0; j < 6; ++j) {
        L[i][j] *= weights[i];
      }
    }

    L.transpose(Lt);
    L.AtA(H);
    vpColVector Lte = Lt * weighted_error;
    vpColVector v = -parameters.gain * ((H + Id * mu).pseudoInverse() * Lte);
    cTw = vpExponentialMap::direct(v).inverse() * cTw;
    double errorNormCurr = weighted_error.frobeniusNorm();
    double improvementFactor = errorNormCurr / errorNormPrev;
    if (iter > 0) {
      if (improvementFactor < 1.0 && improvementFactor >(1.0 - parameters.minImprovementFactor)) {
        break;
      }
    }
    // if (improvementFactor < 1.0) {
    mu *= parameters.muIterFactor;
  // }
  // else {
  //   mu /= parameters.muIterFactor;
  // }
    errorNormPrev = errorNormCurr;
  }
}
