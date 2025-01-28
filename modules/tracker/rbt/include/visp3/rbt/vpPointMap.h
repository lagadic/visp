#ifndef VP_POINT_MAP_H
#define VP_POINT_MAP_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>


#include <list>


class VISP_EXPORT vpPointMap
{
public:
  vpPointMap(unsigned maxPoints, double minDistNewPoints, double maxDepthErrorVisibility, double outlierThreshold)
  {
    m_maxPoints = maxPoints;
    m_minDistNewPoint = minDistNewPoints;
    m_maxDepthError = maxDepthErrorVisibility;
    m_outlierThreshold = outlierThreshold;
  }

  const vpMatrix &getPoints() { return m_X; }
  void setPoints(const vpMatrix &X) { m_X = X; }

  void getPoints(const vpArray2D<int> &indices, vpMatrix &X);

  void project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX);
  void project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs);
  void project(const vpCameraParameters &cam, const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs, vpMatrix &uvs);

  void getVisiblePoints(const unsigned int h, const unsigned int w, const vpMatrix &cX, const vpMatrix &uvs, const vpColVector &expectedZ, std::list<int> &indices);
  void getVisiblePoints(const unsigned int h, const unsigned int w, const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const vpImage<float> &depth, std::list<int> &indices);

  void getOutliers(const vpArray2D<int> &originalIndices, const vpMatrix &uvs,
  const vpMatrix &observations, std::list<int> &indices);

  void selectValidNewCandidates(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw,
   const vpArray2D<int> &originalIndices, const vpMatrix &uvs,
   const vpImage<float> &depth, vpMatrix &oXs, std::list<int> &validCandidateIndices);

  void updatePoints(const vpArray2D<int> &indicesToRemove, const vpMatrix &pointsToAdd, std::list<int> &removedIndices, unsigned int &numAddedPoints);
  void updatePoint(unsigned int index, double X, double Y, double Z)
  {
    m_X[index][0] = X;
    m_X[index][1] = Y;
    m_X[index][2] = Z;
  }

private:
  vpMatrix m_X; // N x 3, points expressed in world frame
  unsigned m_maxPoints;
  double m_minDistNewPoint;
  double m_maxDepthError;
  double m_outlierThreshold;


};

#endif
