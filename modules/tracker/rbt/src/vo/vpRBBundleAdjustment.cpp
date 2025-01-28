#include <visp3/rbt/vpRBBundleAdjustment.h>


void vpRBBundleAdjustment::asParamVector(vpColVector &params)
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
}


void vpRBBundleAdjustment::CameraData::error(MapIndexView &mapView, const vpColVector &params, vpColVector &e, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const
{
  double *cp = params.data + 6 * cameraIndex;
  const vpHomogeneousMatrix cTw(cp[0], cp[1], cp[2], cp[3], cp[4], cp[5]);
  const vpRotationMatrix cRw(cTw);
  const vpTranslationVector t(cTw);

  unsigned int pointIndex = 0;
  double *pointsParams = params.data + 6 * numCameras;
  vpColVector wX(3);
  vpColVector cX(3);
  for (unsigned int i = startResidual; i < startResidual + numResiduals(); i += 2) {
    unsigned int pi = mapView.getViewIndex(m_indices3d[pointIndex]);
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

void vpRBBundleAdjustment::CameraData::fillJacobianSparsity(const MapIndexView &mapView, vpMatrix &S, unsigned int cameraIndex, unsigned int numCameras, unsigned int startResidual) const
{
  unsigned int pointIndex = 0;
  for (unsigned int i = startResidual; i < startResidual + numResiduals(); i += 2) {
    for (unsigned int j = 0; j < 6; ++j) {
      S[i][cameraIndex * 6 + j] = 1;
      S[i + 1][cameraIndex * 6 + j] = 1;
    }


    unsigned int pi = mapView.getViewIndex(m_indices3d[pointIndex]);
    for (unsigned int j = 0; j < 3; ++j) {
      S[i][numCameras * 6 + pi * 3] = 1;
      S[i + 1][numCameras * 6 + pi * 3] = 1;
    }
    ++pointIndex;

  }
}


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
}
