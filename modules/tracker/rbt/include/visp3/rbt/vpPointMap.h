#ifndef VP_POINT_MAP_H
#define VP_POINT_MAP_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>

#include <list>


class VISP_EXPORT vpPointMap
{
public:
  vpPointMap(unsigned maxPoints)
  {
    m_maxPoints = maxPoints;
    m_minDistNewPoint = 1e-2;
    m_maxDepthError = 1e-3;
    m_outlierThreshold = 50.0;
  }

  void getPoints(const vpArray2D<int> &indices, vpMatrix &X);

  void project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX);
  void project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs);
  void project(const vpCameraParameters &cam, const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs, vpMatrix &uvs);

  void getVisiblePoints(const unsigned int h, const unsigned int w, const vpMatrix &cX, const vpMatrix &uvs, const vpColVector &expectedZ, std::list<int> &indices);

  void getOutliers(const vpArray2D<int> &originalIndices, const vpMatrix &uvs, const vpMatrix &observations, std::list<int> &indices)
  {
    if (uvs.getRows() != observations.getRows()) {
      throw vpException(vpException::dimensionError, "Uvs and observations should have same number of rows");
    }
    double thresholdSqr = vpMath::sqr(m_outlierThreshold);
    for (unsigned int i = 0; i < uvs.getRows(); ++i) {
      const double error = vpMath::sqr(uvs[i][0] - observations[i][0]) + vpMath::sqr(uvs[i][1] - observations[i][1]);
      if (error >= thresholdSqr) {
        indices.push_back(originalIndices[i][0]);
      }
    }
  }

  void updatePoints(const vpArray2D<int> &indicesToRemove, const vpMatrix &pointsToAdd, std::list<int> &removedIndices)
  {
    unsigned int newSize = m_X.getRows() - indicesToRemove.getRows() + pointsToAdd.size();
    unsigned int startIndex = 0;

    for (unsigned int i = 0; i < indicesToRemove.getRows(); ++i) {
      removedIndices.push_back(indicesToRemove[i][0]);
    }

    removedIndices.sort();
    if (newSize > m_maxPoints) {
      int shouldBeRemoved = newSize - m_maxPoints;
      newSize = m_maxPoints;
      startIndex = shouldBeRemoved;

      // If the first values are filtered by indicesToRemove, we need to further increment the start index
      std::list<int> startingIndices;
      auto removedIt = removedIndices.begin();
      for (int i = 0; i < shouldBeRemoved; ++i) {
        if (i < (*removedIt)) {
          startingIndices.push_back(i);
          ++startIndex;
        }
        else {
          ++removedIt;
        }
      }
      for (int v : startingIndices) {
        removedIndices.push_front(v);
      }
    }

    vpMatrix newX(newSize, 3);
    std::cout << newX.getRows() << std::endl;

    unsigned int newXIndex = 0;
    unsigned int oldXIndex = 0;
    // Copy between removed rows
    for (unsigned int removedRow : removedIndices) {
      unsigned int copiedRows = removedRow - oldXIndex;
      if (copiedRows > 0) {
        memcpy(newX[newXIndex], m_X[oldXIndex], copiedRows * m_X.getCols());
        newXIndex += copiedRows;
      }
      oldXIndex = removedRow + 1;
    }
    // Copy from last removed row to the end of the array
    unsigned int copiedRows = m_X.getRows() - oldXIndex;
    if (copiedRows > 0) {
      memcpy(newX[newXIndex], m_X[oldXIndex], copiedRows * m_X.getCols());
      newXIndex += copiedRows;
    }

    for (unsigned int i = 0; i < pointsToAdd.getRows(); ++i) {
      for (unsigned int j = 0; j < 3; ++j) {
        newX[newXIndex][j] = pointsToAdd[i][j];
      }
      ++newXIndex;
    }

    m_X = std::move(newX);

  }


private:
  vpMatrix m_X; // N x 3, points expressed in world frame
  unsigned m_maxPoints;
  double m_minDistNewPoint;
  double m_maxDepthError;
  double m_outlierThreshold;


};

#endif
