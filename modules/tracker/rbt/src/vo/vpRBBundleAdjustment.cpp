#include <visp3/rbt/vpRBBundleAdjustment.h>

#define DEBUG_RB_BA 1


#include <set>

vpRBBundleAdjustment::vpRBBundleAdjustment(unsigned int numCams, const vpCameraParameters &cam, vpPointMap &map)
{
  m_numCams = numCams;
  m_cam = cam;
  m_map = &map;
}

void vpRBBundleAdjustment::addNewCamera(const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs)
{
  if (m_cameras.size() == m_numCams) {
    m_cameras.pop_front();
  }
#ifdef DEBUG_RB_BA
  for (unsigned int index : indices3d) {
    if (index >= m_map->getPoints().getRows()) {
      throw vpException(vpException::badValue, "Got a 3D index that was greater than number of points in the map");
    }
  }
#endif
  m_cameras.push_back(CameraData(m_cam, cTw, indices3d, uvs));
  m_mapView.update(m_cameras);
}

void vpRBBundleAdjustment::updateEnvironment(const std::vector<unsigned int> &filteredIndices)
{
  std::vector<unsigned int> sortedFilteredIndices(filteredIndices);
  std::sort(sortedFilteredIndices.begin(), sortedFilteredIndices.end());
  for (CameraData &camera: m_cameras) {
    camera.filter(sortedFilteredIndices);
  }
  m_mapView.update(m_cameras);
}

void vpRBBundleAdjustment::asParamVector(vpColVector &params) const
{

  unsigned int numUsedPoints = m_mapView.numPoints();
  params.resize(m_cameras.size() * 6 + numUsedPoints * 3, false);
  unsigned int i = 0;
  // First parameters are the camera poses
  for (const CameraData &camera: m_cameras) {
    vpPoseVector r(camera.pose());
    double *rp = params.data + i;
    for (unsigned int j = 0; j < 6; ++j) {
      rp[j] = r[j];
    }
    i += 6;
  }
  // Then, add 3D Points from the map
  const vpMatrix &Xs = m_map->getPoints();
  for (unsigned int vpi = 0; vpi < numUsedPoints; ++vpi) {
    double *pp = params.data + i;
    unsigned int pi = m_mapView.getPointIndex(vpi);
    pp[0] = Xs[pi][0];
    pp[1] = Xs[pi][1];
    pp[2] = Xs[pi][2];
    i += 3;
  }

#ifdef DEBUG_RB_BA
  if (i != params.size()) {
    throw vpException(vpException::dimensionError, "Mismatch between stored number of cameras and points and param vector size");
  }
#endif
}

void vpRBBundleAdjustment::updateFromParamVector(const vpColVector &params)
{

#ifdef DEBUG_RB_BA
  if (params.size() != m_cameras.size() * 6 + m_mapView.numPoints() * 3) {
    throw vpException(vpException::dimensionError, "Mismatch between param vector size and number of points/cameras");
  }
#endif
  unsigned int i = 0;

  // Update camera poses
  for (CameraData &camera: m_cameras) {
    const double *cp = params.data + i;
    camera.setPose(vpPoseVector(cp[0], cp[1], cp[2], cp[3], cp[4], cp[5]));
    i += 6;
  }

  //Update 3D points
  unsigned int numPoints = m_mapView.numPoints();

  for (unsigned int viewIndex = 0; viewIndex < numPoints; ++viewIndex) {
    const double *pp = params.data + i;
    const unsigned int pointIndex = m_mapView.getPointIndex(viewIndex);
    m_map->updatePoint(pointIndex, pp[0], pp[1], pp[2]);
    i += 3;
  }

#ifdef DEBUG_RB_BA
  if (i != params.size()) {
    throw vpException(vpException::dimensionError, "Mismatch between stored number of cameras and points and param vector size");
  }
#endif

}

std::vector<vpHomogeneousMatrix> vpRBBundleAdjustment::getCameraPoses() const
{
  std::vector<vpHomogeneousMatrix> poses;
  for (const CameraData &camera: m_cameras) {
    poses.push_back(vpHomogeneousMatrix(camera.pose()));
  }
  return poses;
}


void vpRBBundleAdjustment::computeError(const vpColVector &params, vpColVector &e)
{
  e.resize(numResiduals(), false);
  unsigned int i = 0;
  unsigned int cameraIndex = 0;
  for (CameraData &camera: m_cameras) {
    camera.error(m_mapView, params, e, cameraIndex, m_cameras.size(), i);
    i += camera.numResiduals();
    ++cameraIndex;
  }
}

void vpRBBundleAdjustment::jacobianSparsity(vpArray2D<int> &S)
{
  unsigned int numParams = numCameras() * 6 + numPoints3d() * 3;
  S.resize(numResiduals(), numParams);
  unsigned int i = 0;
  unsigned int cameraIndex = 0;
  for (const CameraData &camera: m_cameras) {
    camera.fillJacobianSparsity(m_mapView, S, cameraIndex, m_cameras.size(), i);
    i += camera.numResiduals();
    ++cameraIndex;
  }
}

////////// Camera data
vpRBBundleAdjustment::CameraData::CameraData(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs)
{
  m_cTw = cTw;
  m_indices3d = indices3d;

#ifdef DEBUG_RB_BA
  if (m_indices3d.size() != uvs.getRows()) {
    throw vpException(vpException::badValue, "Number of 3D points and 2D observations should be the same!");
  }

  for (unsigned int i = 1; i < m_indices3d.size(); ++i) {
    if (m_indices3d[i] < m_indices3d[i - 1]) {
      throw vpException(vpException::badValue, "3D index list should be sorted!");
    }
  }
#endif

  m_points2d.resize(uvs.getRows());

  for (unsigned int i = 0; i < uvs.getRows(); ++i) {
    vpPixelMeterConversion::convertPointWithoutDistortion(cam, uvs[i][0], uvs[i][1], m_points2d[i][0], m_points2d[i][1]);
  }
}

void vpRBBundleAdjustment::CameraData::error(MapIndexView &mapView, const vpColVector &params, vpColVector &e,
  unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const
{
  const double *cp = params.data + 6 * cameraIndex;
  const vpHomogeneousMatrix cTw(cp[0], cp[1], cp[2], cp[3], cp[4], cp[5]);
  const vpRotationMatrix cRw(cTw);
  const vpTranslationVector t(cTw);

  unsigned int pointIndex = 0;
  const double *pointsParams = params.data + 6 * numCameras;
  vpColVector wX(3);
  vpColVector cX(3);
  for (unsigned int i = startResidual; i < startResidual + numResiduals(); i += 2) {
    const unsigned int pi = mapView.getViewIndex(m_indices3d[pointIndex]);
    const double *pp = pointsParams + 3 * pi;
    wX[0] = pp[0];
    wX[1] = pp[1];
    wX[2] = pp[2];

    cX = cRw * wX;
    cX += t;

    double x = cX[0] / cX[2];
    double y = cX[1] / cX[2];

    e[i] = x - m_points2d[pointIndex][0];
    e[i + 1] = y - m_points2d[pointIndex][1];
    ++pointIndex;
  }
}

void vpRBBundleAdjustment::CameraData::jacobian(const MapIndexView &mapView, const vpColVector &params, vpMatrix &J, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const
{

  const double *cp = params.data + cameraIndex * 6;
  unsigned int ci = cameraIndex * 6;
  const vpHomogeneousMatrix cTw(cp[0], cp[1], cp[2], cp[3], cp[4], cp[5]);
  const vpRotationMatrix cRw(cTw);
  const vpTranslationVector t(cTw);
  const double *pointsParams = params.data + 6 * numCameras;
  vpColVector wX(3);
  vpColVector cX(3);


  vpMatrix dpDXp(2, 3, 0.0);
  vpMatrix dpDX(2, 3);
  unsigned int pointIndex = 0;

  for (unsigned int i = startResidual; i < startResidual + numResiduals(); i += 2) {
    const unsigned int vpi = mapView.getViewIndex(m_indices3d[pointIndex]);
    const double *pp = pointsParams + 3 * vpi;

    wX[0] = pp[0];
    wX[1] = pp[1];
    wX[2] = pp[2];

    cX = cRw * wX;
    cX += t;

    double x = cX[0] / cX[2];
    double y = cX[1] / cX[2];

    double Zinv = 1.0 / cX[2];

    double *Jx = J[i];
    double *Jy = J[i + 1];

    // Camera jacobian for x
    Jx[ci + 0] = -Zinv; Jx[ci + 1] = 0.0; Jx[ci + 2] = x * Zinv;
    Jx[ci + 3] = x * y; Jx[ci + 4] = -(1.0 + x * x); Jx[ci + 5] = y;
    // Camera Jacobian for y
    Jy[ci + 0] = 0.0; Jy[ci + 1] = -Zinv; Jy[ci + 2] = y * Zinv;
    Jy[ci + 3] = 1.0 + y * y; Jy[ci + 4] = -(x * y); Jy[ci + 5] = -x;

    unsigned int pi = numCameras * 6 + vpi * 3;

    dpDXp[0][0] = -Zinv; dpDXp[0][2] = x * Zinv;
    dpDXp[1][1] = -Zinv; dpDXp[1][2] = y * Zinv;
    dpDX = dpDXp * cRw;
    Jx[pi + 0] = dpDX[0][0]; Jx[pi + 1] = dpDX[0][1]; Jx[pi + 2] = dpDX[0][2];
    Jy[pi + 0] = dpDX[1][0]; Jy[pi + 1] = dpDX[1][1]; Jy[pi + 2] = dpDX[1][2];

    ++pointIndex;
  }
}

void vpRBBundleAdjustment::CameraData::fillJacobianSparsity(const MapIndexView &mapView, vpArray2D<int> &S, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const
{
  unsigned int pointIndex = 0;
  for (unsigned int i = startResidual; i < startResidual + numResiduals(); i += 2) {
    for (unsigned int j = 0; j < 6; ++j) {
      S[i][cameraIndex * 6 + j] = 1;
      S[i + 1][cameraIndex * 6 + j] = 1;
    }


    unsigned int pi = mapView.getViewIndex(m_indices3d[pointIndex]);
    for (unsigned int j = 0; j < 3; ++j) {
      S[i][numCameras * 6 + pi * 3 + j] = 1;
      S[i + 1][numCameras * 6 + pi * 3 + j] = 1;
    }
    ++pointIndex;

  }
}

void vpRBBundleAdjustment::CameraData::filter(const std::vector<unsigned int> &filteredIndices)
{
#ifdef DEBUG_RB_BA
  for (unsigned int i = 1; i < filteredIndices.size(); ++i) {
    if (filteredIndices[i] < filteredIndices[i - 1]) {
      throw vpException(vpException::badValue, "Removed indices should be sorted!");
    }
  }
  for (unsigned int i = 1; i < m_indices3d.size(); ++i) {
    if (m_indices3d[i] < m_indices3d[i - 1]) {
      throw vpException(vpException::badValue, "indices 3d are not sorted!");
    }
  }
#endif
  std::vector<unsigned int> indicesToKeep;
  std::vector<unsigned int> indices3dToKeep; // Since removing indices may shift values that are greater, we need to keep a separate list for 3D indices
  unsigned int currentFilterIndex = 0;
  unsigned int currentIndex = 0;
  while (currentIndex < m_indices3d.size() && currentFilterIndex < filteredIndices.size()) {
    unsigned int toRemove = filteredIndices[currentFilterIndex];
    unsigned int currentValue = m_indices3d[currentIndex];

    while (currentValue < toRemove) {

      indicesToKeep.push_back(currentIndex);
      // If there are currentFilterIndex values that were removed before, substract it from the current value
      indices3dToKeep.push_back(currentValue - currentFilterIndex);
      ++currentIndex;
      if (currentIndex >= m_indices3d.size()) {
        break;
      }
      currentValue = m_indices3d[currentIndex];
    }

    if (currentValue == toRemove) {
      ++currentIndex;
    }
    ++currentFilterIndex;
  }

  std::vector<std::array<double, 2>> newPoints2d(indicesToKeep.size());
  std::vector<unsigned int> newPointIndices3d(indicesToKeep.size());

  for (unsigned int i = 0; i < indicesToKeep.size(); ++i) {
    newPoints2d[i][0] = m_points2d[indicesToKeep[i]][0];
    newPoints2d[i][1] = m_points2d[indicesToKeep[i]][1];
    newPointIndices3d[i] = indices3dToKeep[i];
  }

  m_indices3d = std::move(newPointIndices3d);
  m_points2d = std::move(newPoints2d);

#ifdef DEBUG_RB_BA
  if (m_indices3d.size() != m_points2d.size()) {
    throw vpException(vpException::badValue, "Number of 3D points and 2D observations should be the same!");
  }
#endif
}

////// MapIndexView

void vpRBBundleAdjustment::MapIndexView::update(const std::list<CameraData> &cameras)
{
  std::set<unsigned int> usedPointIndices;

  for (const CameraData &camera: cameras) {
    const std::vector<unsigned int> &cameraIndices = camera.getPointsIndices();
    usedPointIndices.insert(cameraIndices.begin(), cameraIndices.end());
  }

  std::vector<unsigned int> sortedPointIndices(usedPointIndices.begin(), usedPointIndices.end());
  std::sort(sortedPointIndices.begin(), sortedPointIndices.end());

  m_pointToView.clear();
  m_viewToPoint.clear();

  for (unsigned int i = 0; i < sortedPointIndices.size(); ++i) {
    unsigned int pointIndex = sortedPointIndices[i];
    m_viewToPoint[i] = pointIndex;
    m_pointToView[pointIndex] = i;
  }

#ifdef DEBUG_RB_BA
  if (m_pointToView.size() != m_viewToPoint.size()) {
    throw vpException(vpException::dimensionError, "Mismatch between map sizes!");
  }
#endif
}
