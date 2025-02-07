#include <visp3/rbt/vpRBBundleAdjustment.h>

#define DEBUG_RB_BA 1


#include <set>

vpRBBundleAdjustment::vpRBBundleAdjustment(bool optimizeCameras, unsigned int numCams, const vpCameraParameters &cam, vpPointMap &map)
{
  m_optimizeCameras = optimizeCameras;
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
  params.resize(numParameters(), false);
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
  if (params.size() != numParameters()) {
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

void vpRBBundleAdjustment::jacobianSparsityPattern(std::vector<unsigned int> &rowIndices, std::vector<unsigned int> &columnIndices)
{
  rowIndices.reserve(numResiduals() * 9);
  columnIndices.reserve(numResiduals() * 9);

  unsigned int i = 0;
  unsigned int cameraIndex = 0;
  for (const CameraData &camera: m_cameras) {
    camera.jacobianSparsityPattern(m_mapView, rowIndices, columnIndices, cameraIndex, m_cameras.size(), i, m_optimizeCameras);
    i += camera.numResiduals();
    ++cameraIndex;
  }
}

////////// Camera data
vpRBBundleAdjustment::CameraData::CameraData(const vpCameraParameters &cam, const vpHomogeneousMatrix &cTw, const std::vector<unsigned int> &indices3d, const vpMatrix &uvs)
{
  m_r = cTw;
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

  unsigned int usedObsIndex = 0;
  const double *pointsParams = params.data + 6 * numCameras;
  vpColVector wX(3);
  vpColVector cX(3);
  unsigned int numResidu = numResiduals();
  for (unsigned int i = startResidual; i < startResidual + numResidu; i += 2) {
    unsigned int obsIndex = m_usedObservations[usedObsIndex];
    const unsigned int vpi = mapView.getViewIndex(m_indices3d[obsIndex]);
    const double *pp = pointsParams + 3 * vpi;
    wX[0] = pp[0];
    wX[1] = pp[1];
    wX[2] = pp[2];

    cX = cRw * wX;
    cX += t;

    double x = cX[0] / cX[2];
    double y = cX[1] / cX[2];

    e[i] = x - m_points2d[obsIndex][0];
    e[i + 1] = y - m_points2d[obsIndex][1];
    ++usedObsIndex;
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

  vpMatrix dxydcX(2, 3, 0.0);
  vpMatrix dxydwX(2, 3, 0.0);
  unsigned int usedObsIndex = 0;
  unsigned int numResidu = numResiduals();
  for (unsigned int i = startResidual; i < startResidual + numResidu; i += 2) {
    const unsigned int obsIndex = m_usedObservations[usedObsIndex];
    const unsigned int vpi = mapView.getViewIndex(m_indices3d[obsIndex]);
    const double *pp = pointsParams + 3 * vpi;

    wX[0] = pp[0];
    wX[1] = pp[1];
    wX[2] = pp[2];

    cX = cRw * wX;
    cX += t;

    const double x = cX[0] / cX[2];
    const double y = cX[1] / cX[2];
    const double Zinv = 1.0 / cX[2];

    double *Jx = J[i];
    double *Jy = J[i + 1];

    // Jacobian of the 2D projection of a 3D point (already in camera frame)
    dxydcX[0][0] = Zinv; dxydcX[0][2] = -(x * Zinv);
    dxydcX[1][1] = Zinv; dxydcX[1][2] = -(y * Zinv);

    // Camera Jacobian for x
    Jx[ci + 0] = dxydcX[0][0]; Jx[ci + 1] = dxydcX[0][1]; Jx[ci + 2] = dxydcX[0][2];

    Jx[ci + 3] = -(x * y); Jx[ci + 4] = (1.0 + x * x); Jx[ci + 5] = -y;
    // Camera Jacobian for y
    Jy[ci + 0] = dxydcX[1][0]; Jy[ci + 1] = dxydcX[1][1]; Jy[ci + 2] = dxydcX[1][2];

    Jy[ci + 3] = -(1.0 + y * y); Jy[ci + 4] = (x * y); Jy[ci + 5] = x;

    // Point Jacobian
    unsigned int pi = numCameras * 6 + vpi * 3;
    vpMatrix::mult2Matrices(dxydcX, cRw, dxydwX);
    Jx[pi + 0] = dxydwX[0][0]; Jx[pi + 1] = dxydwX[0][1]; Jx[pi + 2] = dxydwX[0][2];
    Jy[pi + 0] = dxydwX[1][0]; Jy[pi + 1] = dxydwX[1][1]; Jy[pi + 2] = dxydwX[1][2];

    ++usedObsIndex;
  }
}

void vpRBBundleAdjustment::CameraData::jacobianSparsityPattern(const MapIndexView &mapView, std::vector<unsigned int> &rowIndices, std::vector<unsigned int> &columnIndices, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual, bool optimizeCameras) const
{
  unsigned int usedObsIndex = 0;
  unsigned int numResidu = numResiduals();
  for (unsigned int i = startResidual; i < startResidual + numResidu; i += 2) {
    unsigned int obsIndex = m_usedObservations[usedObsIndex];
    unsigned int vpi = mapView.getViewIndex(m_indices3d[obsIndex]);
    unsigned pIndex = numCameras * 6 + vpi * 3;
    if (optimizeCameras) {
      for (unsigned int j = 0; j < 6; ++j) {
        rowIndices.push_back(i);
        columnIndices.push_back(cameraIndex * 6 + j);
      }
    }
    for (unsigned int j = 0; j < 3; ++j) {
      rowIndices.push_back(i);
      columnIndices.push_back(pIndex + j);
    }
    if (optimizeCameras) {
      for (unsigned int j = 0; j < 6; ++j) {
        rowIndices.push_back(i + 1);
        columnIndices.push_back(cameraIndex * 6 + j);
      }

    }
    for (unsigned int j = 0; j < 3; ++j) {
      rowIndices.push_back(i + 1);
      columnIndices.push_back(pIndex + j);
    }
    ++usedObsIndex;
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

void vpRBBundleAdjustment::MapIndexView::update(std::list<CameraData> &cameras)
{
  std::vector<std::vector<unsigned int>> usedPointsPerCam(cameras.size());


  // Retrieve all points that are used by at least 2 cameras
  std::vector<CameraData *> cams;
  for (CameraData &c: cameras) {
    cams.push_back(&c);
  }

  for (unsigned int index1 = 0; index1 < cams.size(); ++index1) {
    const CameraData *c1 = cams[index1];
    for (unsigned int index2 = index1 + 1; index2 < cams.size(); ++index2) {
      const CameraData *c2 = cams[index2];
      std::vector<unsigned int> sharedPoints;
      const std::vector<unsigned int> &cameraIndices = c1->getPointsIndices();
      const std::vector<unsigned int> &cameraIndices2 = c2->getPointsIndices();

      // Points that are used by both cameras
      std::set_intersection(cameraIndices.begin(), cameraIndices.end(),
                            cameraIndices2.begin(), cameraIndices2.end(), std::back_inserter(sharedPoints));

      // Update set of points by each cam with the new points
      std::vector<unsigned int> unionWithCam1, unionWithCam2;
      std::set_union(usedPointsPerCam[index1].begin(), usedPointsPerCam[index1].end(),
                    sharedPoints.begin(), sharedPoints.end(), std::back_inserter(unionWithCam1));
      std::set_union(usedPointsPerCam[index2].begin(), usedPointsPerCam[index2].end(),
                    sharedPoints.begin(), sharedPoints.end(), std::back_inserter(unionWithCam2));
      usedPointsPerCam[index1] = std::move(unionWithCam1);
      usedPointsPerCam[index2] = std::move(unionWithCam2);


      std::cout << "Camera " << index1 << " and camera " << index2 << " share " << sharedPoints.size() << " points.";
      std::cout << " Camera " << index1 << " has " << cameraIndices.size() << " observed points.";
      std::cout << " Camera " << index2 << " has " << cameraIndices2.size() << " observed points." << std::endl;
    }
  }

  unsigned int j = 0;
  for (std::vector<unsigned int> &cameraPoints: usedPointsPerCam) {
    cams[j]->setUsedObservationsFrom3DIndices(cameraPoints);
    std::cout << "Using " << cameraPoints.size() << " points for camera " << j << std::endl;
    ++j;
  }

  // Get all used 3D points
  std::set<unsigned int> usedPointIndices;
  for (std::vector<unsigned int> &pointsUsed: usedPointsPerCam) {
    usedPointIndices.insert(pointsUsed.begin(), pointsUsed.end());
  }


  std::vector<unsigned int> sortedPointIndices(usedPointIndices.begin(), usedPointIndices.end());
  std::sort(sortedPointIndices.begin(), sortedPointIndices.end());

  // Fill map

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
