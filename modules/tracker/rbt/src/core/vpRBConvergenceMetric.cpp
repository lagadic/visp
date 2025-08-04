#include <visp3/rbt/vpRBConvergenceMetric.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include VISP_NLOHMANN_JSON(json_fwd.hpp)

std::shared_ptr<vpRBConvergenceMetric> vpRBConvergenceMetric::loadFromJSON(const nlohmann::json &j)
{
  const std::string key = j.at("type");
  double renderThreshold = j.value("renderThreshold", 0.0);
  double convergenceThreshold = j.value("convergenceThreshold", 0.0);
  unsigned int numPoints = j.value("samples", 0);
  unsigned int seed = j.value("seed", 42);

  if (key == "reprojection") {
    return std::make_shared<vpRBConvergenceReprojectionMetric>(renderThreshold, convergenceThreshold, numPoints, seed);
  }
  else if (key == "add") {
    return std::make_shared<vpRBConvergenceADDMetric>(renderThreshold, convergenceThreshold, numPoints, seed);
  }

  throw vpException(vpException::badValue, "Tried to parse an incorrect convergence metric type: %s", key.c_str());

}
#endif


vpRBConvergenceMetric::vpRBConvergenceMetric(double renderThreshold, double convergedThreshold, unsigned int numPoints, unsigned int seed)
  : m_seed(seed), m_map(numPoints, 0.0, 0.0, 0.0, 0.0), m_random(seed),
  m_rerenderThreshold(renderThreshold), m_convergedThreshold(convergedThreshold)
{
  m_indices.resize(numPoints, 1);
  for (unsigned int i = 0; i < numPoints; ++i) {
    m_indices[i][0] = i;
  }
}

void vpRBConvergenceMetric::sampleObject(vpObjectCentricRenderer &renderer)
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
  std::vector<int> removed;
  unsigned int added;
  m_map.updatePoints(empty, oX, removed, added);
  if (added != m_map.getNumMaxPoints()) {
    throw vpException(vpException::dimensionError, "Something went wrong when inserting bb points into the map");
  }
}

vpRBConvergenceADDMetric::vpRBConvergenceADDMetric(double renderThreshold, double convergedThreshold, unsigned int numPoints, unsigned int seed) : vpRBConvergenceMetric(renderThreshold, convergedThreshold, numPoints, seed)
{ }

vpRBConvergenceReprojectionMetric::vpRBConvergenceReprojectionMetric(double renderThreshold, double convergedThreshold, unsigned int numPoints, unsigned int seed) : vpRBConvergenceMetric(renderThreshold, convergedThreshold, numPoints, seed)
{ }




double vpRBConvergenceADDMetric::operator()(const vpCameraParameters & /*cam*/, const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2)
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

double vpRBConvergenceReprojectionMetric::operator()(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTo1, const vpHomogeneousMatrix &cTo2)
{
  vpMatrix X1, X2;
  vpMatrix xs1, xs2;
  vpMatrix uv1, uv2;
  m_map.project(cam, m_indices, cTo1, X1, xs1, uv1);
  m_map.project(cam, m_indices, cTo2, X2, xs2, uv2);
  if (X1.getRows() == 0) {
    throw vpException(vpException::badValue, "Points were not sampled from the object");
  }
  double error = 0.0;
  for (unsigned int i = 0; i < uv1.getRows(); ++i) {
    double d = sqrtf((uv1[i][0] - uv2[i][0]) + vpMath::sqr(uv1[i][1] - uv2[i][1]));
    if (!vpMath::isNaN(d)) {
      error += d;
    }
  }

  return error / static_cast<double>(uv1.getRows());
}
