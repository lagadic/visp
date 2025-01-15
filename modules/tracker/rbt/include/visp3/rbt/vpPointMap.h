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

  void selectValidNewCandidates(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const vpArray2D<int> &originalIndices, const vpMatrix &uvs, vpImage<float> &depth, vpMatrix &oXs, std::list<int> &validCandidateIndices)
  {
    if (originalIndices.getRows() != uvs.getRows()) {
      throw vpException(vpException::dimensionError, "Indices and keypoint locations should have the same dimensions");
    }
    validCandidateIndices.clear();
    double x, y;
    vpColVector oX(3);
    vpColVector cX(3);
    const vpHomogeneousMatrix wTc = cTw.inverse();
    const vpRotationMatrix wRc = wTc.getRotationMatrix();
    const vpTranslationVector t = wTc.getTranslationVector();
    double farEnoughThresholdSq = m_minDistNewPoint * m_minDistNewPoint;

    std::list<vpColVector> validoXList;

    for (unsigned int i = 0; i < uvs.getRows(); ++i) {
      double u = uvs[i][0], v = uvs[i][1];
      unsigned int uint = static_cast<unsigned int>(u), vint = static_cast<unsigned int>(v);
      double Z = static_cast<double>(depth[vint][uint]);
      if (Z <= 0.0) {
        continue;
      }

      vpPixelMeterConversion::convertPointWithoutDistortion(cam, u, v, x, y);
      cX[0] = x * Z;
      cX[1] = y * Z;
      cX[2] = Z;
      oX = wRc * cX;
      oX += t;

      bool isFarEnoughFromOtherPoints = true;
      for (unsigned int j = 0; j < m_X.getRows(); ++j) {
        double errSq = vpMath::sqr(oX[0] - m_X[j][0]) + vpMath::sqr(oX[1] - m_X[j][1]) + vpMath::sqr(oX[2] - m_X[j][2]);
        if (errSq < farEnoughThresholdSq) {
          isFarEnoughFromOtherPoints = false;
          break;
        }
      }
      if (isFarEnoughFromOtherPoints) {
        validoXList.push_back(oX);
        validCandidateIndices.push_back(originalIndices[i][0]);

      }
    }

    oXs.resize(validoXList.size(), 3);
    unsigned int i = 0;
    for (const vpColVector &oX: validoXList) {
      oXs[i][0] = oX[0];
      oXs[i][1] = oX[1];
      oXs[i][2] = oX[2];

      ++i;
    }

  }

  void updatePoints(const vpArray2D<int> &indicesToRemove, const vpMatrix &pointsToAdd, std::list<int> &removedIndices)
  {

    int newSize = m_X.getRows() - indicesToRemove.getRows() + pointsToAdd.getRows();
    std::cout << "m_X.getRows() =" << m_X.getRows()<< ", indicesToRemove.getRows() = " << indicesToRemove.getRows() <<  "pointsToAdd.size() = "<< pointsToAdd.size() << std::endl;
    for (unsigned int i = 0; i < indicesToRemove.getRows(); ++i) {
      removedIndices.push_back(indicesToRemove[i][0]);
    }

    removedIndices.sort();
    if (newSize > m_maxPoints) {
      int shouldBeRemoved = newSize - m_maxPoints;
      newSize = m_maxPoints;

      // If the first values are filtered by indicesToRemove, we need to further increment the start index
      std::list<int> startingIndices;
      auto removedIt = removedIndices.begin();
      int i = 0;
      // std::cout << "Should be removed = " << shouldBeRemoved << std::endl;
      while (startingIndices.size() < shouldBeRemoved) {

        if (removedIt == removedIndices.end() || i < (*removedIt)) {
          startingIndices.push_back(i);
        }
        else {
          ++removedIt;
        }
        ++i;
      }
      removedIndices.insert(removedIndices.begin(), startingIndices.begin(), startingIndices.end());
    }

    vpMatrix newX(newSize, 3);

    unsigned int newXIndex = 0;
    unsigned int oldXIndex = 0;
    // Copy between removed rows
    for (int removedRow : removedIndices) {
      unsigned int copiedRows = removedRow - oldXIndex;
      // std::cout << "NewXIndex = " << newXIndex << std::endl;
      // std::cout << "oldXIndex = " << oldXIndex << std::endl;
      // std::cout << "copiedRows = " << copiedRows << std::endl;
      // std::cout << "RemovedRow = " << removedRow << std::endl;

      if (copiedRows > 0) {
        memcpy(newX[newXIndex], m_X[oldXIndex], copiedRows * 3);
        newXIndex += copiedRows;
      }
      oldXIndex = removedRow + 1;
    }
    // std::cout << "Finishing copy with last rows" << std::endl;
    // Copy from last removed row to the end of the array
    unsigned int copiedRows = m_X.getRows() - oldXIndex;
    if (copiedRows > 0) {
      memcpy(newX[newXIndex], m_X[oldXIndex], copiedRows * 3);
      newXIndex += copiedRows;
    }
    // std::cout << "Before add" << std::endl;
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
