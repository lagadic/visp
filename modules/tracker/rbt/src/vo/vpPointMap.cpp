#include <visp3/rbt/vpPointMap.h>


void vpPointMap::getPoints(const vpArray2D<int> &indices, vpMatrix &X)
{
  X.resize(indices.getRows(), 3);
  for (unsigned int i = 0; i < indices.getRows(); ++i) {
    unsigned idx = indices[i][0];
    X[i][0] = m_X[idx][0];
    X[i][1] = m_X[idx][1];
    X[i][2] = m_X[idx][2];
  }
}

void vpPointMap::project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX)
{
  cX.resize(indices.getRows(), 3);
  vpColVector X(3);
  vpColVector rX(3);

  const vpColVector t = cTw.getTranslationVector();
  const vpRotationMatrix R = cTw.getRotationMatrix();
  for (unsigned int i = 0; i < indices.getRows(); ++i) {
    unsigned idx = indices[i][0];
    X[0] = m_X[idx][0];
    X[1] = m_X[idx][1];
    X[2] = m_X[idx][2];

    rX = R * X;

    cX[i][0] = rX[0] + t[0];
    cX[i][1] = rX[1] + t[1];
    cX[i][2] = rX[2] + t[2];
  }
}

void vpPointMap::project(const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs)
{
  project(indices, cTw, cX);
  xs.resize(cX.getRows(), 2);
  for (unsigned int i = 0; i < cX.getRows(); ++i) {
    xs[i][0] = cX[i][0] / cX[i][2];
    xs[i][1] = cX[i][1] / cX[i][2];
  }
}

void vpPointMap::project(const vpCameraParameters &cam, const vpArray2D<int> &indices, const vpHomogeneousMatrix &cTw, vpMatrix &cX, vpMatrix &xs, vpMatrix &uvs)
{
  if (cam.get_projModel() != vpCameraParameters::vpCameraParametersProjType::perspectiveProjWithoutDistortion) {
    throw vpException(vpException::badValue, "Only cameras without distortion are supported");
  }
  project(indices, cTw, cX, xs);
  uvs.resize(xs.getRows(), xs.getCols(), false, false);
  for (unsigned int i = 0; i < xs.getRows(); ++i) {
    vpMeterPixelConversion::convertPointWithoutDistortion(cam, xs[i][0], xs[i][1], uvs[i][0], uvs[i][1]);
  }
}

void vpPointMap::getVisiblePoints(const unsigned int h, const unsigned int w, const vpMatrix &cX, const vpMatrix &uvs, const vpColVector &expectedZ, std::list<int> &indices)
{
  for (unsigned int i = 0; i < cX.getRows(); ++i) {
    const double u = uvs[i][0], v = uvs[i][1];
    const double Z = cX[i][2];
    if (u < 0 || v < 0 || u >= w || v >= h) {
      continue;
    }
    if (fabs(Z - expectedZ[i]) > m_maxDepthError) {
      continue;
    }
    indices.push_back(i);
  }
}
