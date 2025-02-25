#ifndef VP_RB_ADDS_H
#define VP_RB_ADDS_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpUniRand.h>

#include <visp3/rbt/vpPointMap.h>
#include <visp3/rbt/vpObjectCentricRenderer.h>


class VISP_EXPORT vpRBADDSMetric
{

public:
  vpRBADDSMetric(unsigned int numPoints, unsigned int seed) : m_seed(seed), m_map(numPoints, 0.0, 0.0, 0.0), m_random(seed)
  {

  }

  void sampleObject(vpObjectCentricRenderer &renderer)
  {
    m_random.setSeed(m_seed, 0x123465789ULL);
    vpTranslationVector minAxes, maxAxes;
    renderer.get3DExtents(minAxes, maxAxes);

    vpMatrix oX(m_map.getNumMaxPoints(), 3);
    for (unsigned int i = 0; i < oX.getRows(); ++i) {
      for (unsigned int j = 0; j < 3; ++j) {
        oX[i][j] = m_random() * (maxAxes[j] - minAxes[j]) + minAxes[j];
      }
    }
    vpArray2D<int> empty;
    std::list<int> removed;
    unsigned int added;
    m_map.updatePoints(empty, oX, removed, added);
    if (added != m_map.getNumMaxPoints()) {
      throw vpException(vpException::dimensionError, "Something went wrong when inserting bb points into the map");
    }
  }

  double ADDS(const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2)
  {
    vpMatrix X1, X2;

    m_map.project(cTo1, X1);
    m_map.project(cTo2, X2);
    if (X1.getRows() == 0) {
      throw vpException(vpException::badValue, "Points were not sampled from the object");
    }
    double error = 0.0;

    for (unsigned int i = 0; i < X1.getRows(); ++i) {
      double d = sqrtf(vpMath::sqr(X1[i][0] - X2[i][0]) + vpMath::sqr(X1[i][1] - X2[i][1])  + vpMath::sqr(X1[i][2] - X2[i][2]));
      error += d;
    }

    return error / static_cast<double>(X1.getRows());

  }

private:
  unsigned int m_seed;
  vpPointMap m_map;
  vpUniRand m_random;



};


#endif
