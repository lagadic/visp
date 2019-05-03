/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Klt polygon, containing points of interest.
 *
 * Authors:
 * Romain Tallonneau
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp3/core/vpPolygon.h>
#include <visp3/mbt/vpMbtDistanceKltPoints.h>
#include <visp3/me/vpMeTracker.h>

#if defined(VISP_HAVE_MODULE_KLT) && (defined(VISP_HAVE_OPENCV) && (VISP_HAVE_OPENCV_VERSION >= 0x020100))

#if defined(VISP_HAVE_CLIPPER)
#include <clipper.hpp> // clipper private library
#endif

#if defined(__APPLE__) && defined(__MACH__) // Apple OSX and iOS (Darwin)
#include <TargetConditionals.h>             // To detect OSX or IOS using TARGET_OS_IPHONE or TARGET_OS_IOS macro
#endif

/*!
  Basic constructor.

*/
vpMbtDistanceKltPoints::vpMbtDistanceKltPoints()
  : H(), N(), N_cur(), invd0(1.), cRc0_0n(), initPoints(std::map<int, vpImagePoint>()),
    curPoints(std::map<int, vpImagePoint>()), curPointsInd(std::map<int, int>()), nbPointsCur(0), nbPointsInit(0),
    minNbPoint(4), enoughPoints(false), dt(1.), d0(1.), cam(), isTrackedKltPoints(true), polygon(NULL),
    hiddenface(NULL), useScanLine(false)
{
}

/*!
  Basic destructor.

*/
vpMbtDistanceKltPoints::~vpMbtDistanceKltPoints() {}

/*!
  Initialise the face to track. All the points in the map, representing all
  the map detected in the image, are parsed in order to extract the id of the
  points that are indeed in the face.

  \param _tracker : ViSP OpenCV KLT Tracker.
  \param mask: Mask image or NULL if not wanted. Mask values that are set to true are considered in the tracking. To disable a pixel, set false.
*/
void vpMbtDistanceKltPoints::init(const vpKltOpencv &_tracker, const vpImage<bool> *mask)
{
  // extract ids of the points in the face
  nbPointsInit = 0;
  nbPointsCur = 0;
  initPoints = std::map<int, vpImagePoint>();
  curPoints = std::map<int, vpImagePoint>();
  curPointsInd = std::map<int, int>();
  std::vector<vpImagePoint> roi;
  polygon->getRoiClipped(cam, roi);

  for (unsigned int i = 0; i < static_cast<unsigned int>(_tracker.getNbFeatures()); i++) {
    long id;
    float x_tmp, y_tmp;
    _tracker.getFeature((int)i, id, x_tmp, y_tmp);

    bool add = false;

    // Add points inside visibility mask only
    if (vpMeTracker::inMask(mask, (unsigned int) y_tmp, (unsigned int) x_tmp)) {
      if (useScanLine) {
        if ((unsigned int)y_tmp < hiddenface->getMbScanLineRenderer().getPrimitiveIDs().getHeight() &&
          (unsigned int)x_tmp < hiddenface->getMbScanLineRenderer().getPrimitiveIDs().getWidth() &&
          hiddenface->getMbScanLineRenderer().getPrimitiveIDs()[(unsigned int)y_tmp][(unsigned int)x_tmp] ==
          polygon->getIndex())
          add = true;
      }
      else if (vpPolygon::isInside(roi, y_tmp, x_tmp)) {
        add = true;
      }
    }

    if (add) {
#if TARGET_OS_IPHONE
      initPoints[(int)id] = vpImagePoint(y_tmp, x_tmp);
      curPoints[(int)id] = vpImagePoint(y_tmp, x_tmp);
      curPointsInd[(int)id] = (int)i;
#else
      initPoints[id] = vpImagePoint(y_tmp, x_tmp);
      curPoints[id] = vpImagePoint(y_tmp, x_tmp);
      curPointsInd[id] = (int)i;
#endif
    }
  }

  nbPointsInit = (unsigned int)initPoints.size();
  nbPointsCur = (unsigned int)curPoints.size();

  if (nbPointsCur >= minNbPoint)
    enoughPoints = true;
  else
    enoughPoints = false;

  // initialisation of the value for the computation in SE3
  vpPlane plan(polygon->getPoint(0), polygon->getPoint(1), polygon->getPoint(2));

  d0 = plan.getD();
  N = plan.getNormal();

  N.normalize();
  N_cur = N;
  invd0 = 1.0 / d0;
}

/*!
  compute the number of point in this instanciation of the tracker that
  corresponds to the points of the face

  \param _tracker : the KLT tracker
  \return the number of points that are tracked in this face and in this
  instanciation of the tracker
  \param mask: Mask image or NULL if not wanted. Mask values that are set to true are considered in the tracking. To disable a pixel, set false.
*/
unsigned int vpMbtDistanceKltPoints::computeNbDetectedCurrent(const vpKltOpencv &_tracker, const vpImage<bool> *mask)
{
  long id;
  float x, y;
  nbPointsCur = 0;
  curPoints = std::map<int, vpImagePoint>();
  curPointsInd = std::map<int, int>();

  for (unsigned int i = 0; i < static_cast<unsigned int>(_tracker.getNbFeatures()); i++) {
    _tracker.getFeature((int)i, id, x, y);
    if (isTrackedFeature((int)id) && vpMeTracker::inMask(mask, (unsigned int) y, (unsigned int) x)) {
#if TARGET_OS_IPHONE
      curPoints[(int)id] = vpImagePoint(static_cast<double>(y), static_cast<double>(x));
      curPointsInd[(int)id] = (int)i;
#else
      curPoints[id] = vpImagePoint(static_cast<double>(y), static_cast<double>(x));
      curPointsInd[id] = (int)i;
#endif
    }
  }

  nbPointsCur = (unsigned int)curPoints.size();

  if (nbPointsCur >= minNbPoint)
    enoughPoints = true;
  else
    enoughPoints = false;

  return nbPointsCur;
}

/*!
  Compute the interaction matrix and the residu vector for the face.
  The method assumes that these two objects are properly sized in order to be
  able to improve the speed with the use of SubCoVector and subMatrix.

  \warning The function preCompute must be called before the this method.

  \param _R : the residu vector
  \param _J : the interaction matrix
*/
void vpMbtDistanceKltPoints::computeInteractionMatrixAndResidu(vpColVector &_R, vpMatrix &_J)
{
  unsigned int index_ = 0;

  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for (; iter != curPoints.end(); ++iter) {
    int id(iter->first);
    double i_cur(iter->second.get_i()), j_cur(iter->second.get_j());

    double x_cur(0), y_cur(0);
    vpPixelMeterConversion::convertPoint(cam, j_cur, i_cur, x_cur, y_cur);

    vpImagePoint iP0 = initPoints[id];
    double x0(0), y0(0);
    vpPixelMeterConversion::convertPoint(cam, iP0, x0, y0);

    double x0_transform,
        y0_transform; // equivalent x and y in the first image (reference)
    computeP_mu_t(x0, y0, x0_transform, y0_transform, H);

    double invZ = compute_1_over_Z(x_cur, y_cur);

    _J[2 * index_][0] = -invZ;
    _J[2 * index_][1] = 0;
    _J[2 * index_][2] = x_cur * invZ;
    _J[2 * index_][3] = x_cur * y_cur;
    _J[2 * index_][4] = -(1 + x_cur * x_cur);
    _J[2 * index_][5] = y_cur;

    _J[2 * index_ + 1][0] = 0;
    _J[2 * index_ + 1][1] = -invZ;
    _J[2 * index_ + 1][2] = y_cur * invZ;
    _J[2 * index_ + 1][3] = (1 + y_cur * y_cur);
    _J[2 * index_ + 1][4] = -y_cur * x_cur;
    _J[2 * index_ + 1][5] = -x_cur;

    _R[2 * index_] = (x0_transform - x_cur);
    _R[2 * index_ + 1] = (y0_transform - y_cur);
    index_++;
  }
}

double vpMbtDistanceKltPoints::compute_1_over_Z(const double x, const double y)
{
  double num = cRc0_0n[0] * x + cRc0_0n[1] * y + cRc0_0n[2];
  double den = -(d0 - dt);
  return num / den;
}

/*!
  Compute the new coordinates of a point given by its \f$(x,y)\f$ coordinates
  after the homography.

  \f$ P_t = {}^{c_t}H_{c_0} . P_0 \f$

  \param x_in : the x coordinates of the input point
  \param y_in : the y coordinates of the input point
  \param x_out : the x coordinates of the output point
  \param y_out : the y coordinates of the output point
  \param _cHc0 : the homography used to transfer the point
*/
inline void vpMbtDistanceKltPoints::computeP_mu_t(const double x_in, const double y_in, double &x_out, double &y_out,
                                                  const vpMatrix &_cHc0)
{
  double p_mu_t_2 = x_in * _cHc0[2][0] + y_in * _cHc0[2][1] + _cHc0[2][2];

  if (fabs(p_mu_t_2) < std::numeric_limits<double>::epsilon()) {
    x_out = 0.0;
    y_out = 0.0;
    throw vpException(vpException::divideByZeroError, "the depth of the point is calculated to zero");
  }

  x_out = (x_in * _cHc0[0][0] + y_in * _cHc0[0][1] + _cHc0[0][2]) / p_mu_t_2;
  y_out = (x_in * _cHc0[1][0] + y_in * _cHc0[1][1] + _cHc0[1][2]) / p_mu_t_2;
}

/*!
  compute the homography using a displacement matrix.

  the homography is given by:

  \f$ {}^cH_{c_0} = {}^cR_{c_0} + \frac{{}^cT_{c_0} . {}^tN}{d_0} \f$

  Several internal variables are computed (dt, cRc0_0n)

  \param _cTc0 : the displacement matrix of the camera between the initial
  position of the camera and the current camera position \param _cHc0 : the
  homography of the plane
*/
void vpMbtDistanceKltPoints::computeHomography(const vpHomogeneousMatrix &_cTc0, vpHomography &_cHc0)
{
  vpRotationMatrix cRc0;
  vpTranslationVector ctransc0;

  _cTc0.extract(cRc0);
  _cTc0.extract(ctransc0);
  vpMatrix cHc0 = _cHc0.convert();

  //   vpGEMM(cRc0, 1.0, invd0, cRc0, -1.0, _cHc0, VP_GEMM_A_T);
  vpGEMM(ctransc0, N, -invd0, cRc0, 1.0, cHc0, VP_GEMM_B_T);
  cHc0 /= cHc0[2][2];

  H = cHc0;

  //   vpQuaternionVector NQuat(N[0], N[1], N[2], 0.0);
  //   vpQuaternionVector RotQuat(cRc0);
  //   vpQuaternionVector RotQuatConj(-RotQuat.x(), -RotQuat.y(),
  //   -RotQuat.z(), RotQuat.w()); vpQuaternionVector partial = RotQuat *
  //   NQuat; vpQuaternionVector resQuat = (partial * RotQuatConj);
  //
  //   cRc0_0n = vpColVector(3);
  //   cRc0_0n[0] = resQuat.x();
  //   cRc0_0n[1] = resQuat.y();
  //   cRc0_0n[2] = resQuat.z();

  cRc0_0n = cRc0 * N;

  //   vpPlane p(corners[0], corners[1], corners[2]);
  //   vpColVector Ncur = p.getNormal();
  //   Ncur.normalize();
  N_cur = cRc0_0n;
  dt = 0.0;
  for (unsigned int i = 0; i < 3; i += 1) {
    dt += ctransc0[i] * (N_cur[i]);
  }
}

/*!
  Test whether the feature with identifier id in paramters is in the list of
  tracked features.

  \param _id : the id of the current feature to test
  \return true if the id is in the list of tracked feature
*/
bool vpMbtDistanceKltPoints::isTrackedFeature(const int _id)
{
  //   std::map<int, vpImagePoint>::const_iterator iter = initPoints.begin();
  //   while(iter != initPoints.end()){
  //     if(iter->first == _id){
  //       return true;
  //     }
  //     iter++;
  //   }

  std::map<int, vpImagePoint>::iterator iter = initPoints.find(_id);
  if (iter != initPoints.end())
    return true;

  return false;
}

/*!
  Modification of all the pixels that are in the roi to the value of _nb (
  default is 255).

  \param mask : the mask to update (0, not in the object, _nb otherwise).
  \param nb : Optionnal value to set to the pixels included in the face.
  \param shiftBorder : Optionnal shift for the border in pixel (sort of
  built-in erosion) to avoid to consider pixels near the limits of the face.
*/
void vpMbtDistanceKltPoints::updateMask(
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
    cv::Mat &mask,
#else
    IplImage *mask,
#endif
    unsigned char nb, unsigned int shiftBorder)
{
#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  int width = mask.cols;
  int height = mask.rows;
#else
  int width = mask->width;
  int height = mask->height;
#endif

  int i_min, i_max, j_min, j_max;
  std::vector<vpImagePoint> roi;
  polygon->getRoiClipped(cam, roi);

  double shiftBorder_d = (double)shiftBorder;

#if defined(VISP_HAVE_CLIPPER)
  std::vector<vpImagePoint> roi_offset;

  ClipperLib::Path path;
  for (std::vector<vpImagePoint>::const_iterator it = roi.begin(); it != roi.end(); ++it) {
    path.push_back(ClipperLib::IntPoint((ClipperLib::cInt)it->get_u(), (ClipperLib::cInt)it->get_v()));
  }

  ClipperLib::Paths solution;
  ClipperLib::ClipperOffset co;
  co.AddPath(path, ClipperLib::jtRound, ClipperLib::etClosedPolygon);
  co.Execute(solution, -shiftBorder_d);

  // Keep biggest polygon by area
  if (!solution.empty()) {
    size_t index_max = 0;

    if (solution.size() > 1) {
      double max_area = 0;
      vpPolygon polygon_area;

      for (size_t i = 0; i < solution.size(); i++) {
        std::vector<vpImagePoint> corners;

        for (size_t j = 0; j < solution[i].size(); j++) {
          corners.push_back(vpImagePoint((double)(solution[i][j].Y), (double)(solution[i][j].X)));
        }

        polygon_area.buildFrom(corners);
        if (polygon_area.getArea() > max_area) {
          max_area = polygon_area.getArea();
          index_max = i;
        }
      }
    }

    for (size_t i = 0; i < solution[index_max].size(); i++) {
      roi_offset.push_back(vpImagePoint((double)(solution[index_max][i].Y), (double)(solution[index_max][i].X)));
    }
  } else {
    roi_offset = roi;
  }

  vpPolygon polygon_test(roi_offset);
  vpImagePoint imPt;
#endif

#if defined(VISP_HAVE_CLIPPER)
  vpPolygon3D::getMinMaxRoi(roi_offset, i_min, i_max, j_min, j_max);
#else
  vpPolygon3D::getMinMaxRoi(roi, i_min, i_max, j_min, j_max);
#endif

  /* check image boundaries */
  if (i_min > height) { // underflow
    i_min = 0;
  }
  if (i_max > height) {
    i_max = height;
  }
  if (j_min > width) { // underflow
    j_min = 0;
  }
  if (j_max > width) {
    j_max = width;
  }

#if (VISP_HAVE_OPENCV_VERSION >= 0x020408)
  for (int i = i_min; i < i_max; i++) {
    double i_d = (double)i;

    for (int j = j_min; j < j_max; j++) {
      double j_d = (double)j;

#if defined(VISP_HAVE_CLIPPER)
      imPt.set_ij(i_d, j_d);
      if (polygon_test.isInside(imPt)) {
        mask.ptr<uchar>(i)[j] = nb;
      }
#else
      if (shiftBorder != 0) {
        if (vpPolygon::isInside(roi, i_d, j_d) && vpPolygon::isInside(roi, i_d + shiftBorder_d, j_d + shiftBorder_d) &&
            vpPolygon::isInside(roi, i_d - shiftBorder_d, j_d + shiftBorder_d) &&
            vpPolygon::isInside(roi, i_d + shiftBorder_d, j_d - shiftBorder_d) &&
            vpPolygon::isInside(roi, i_d - shiftBorder_d, j_d - shiftBorder_d)) {
          mask.at<unsigned char>(i, j) = nb;
        }
      } else {
        if (vpPolygon::isInside(roi, i, j)) {
          mask.at<unsigned char>(i, j) = nb;
        }
      }
#endif
    }
  }
#else
  unsigned char *ptrData = (unsigned char *)mask->imageData + i_min * mask->widthStep + j_min;
  for (int i = i_min; i < i_max; i++) {
    double i_d = (double)i;
    for (int j = j_min; j < j_max; j++) {
      double j_d = (double)j;
      if (shiftBorder != 0) {
        if (vpPolygon::isInside(roi, i_d, j_d) && vpPolygon::isInside(roi, i_d + shiftBorder_d, j_d + shiftBorder_d) &&
            vpPolygon::isInside(roi, i_d - shiftBorder_d, j_d + shiftBorder_d) &&
            vpPolygon::isInside(roi, i_d + shiftBorder_d, j_d - shiftBorder_d) &&
            vpPolygon::isInside(roi, i_d - shiftBorder_d, j_d - shiftBorder_d)) {
          *(ptrData++) = nb;
        } else {
          ptrData++;
        }
      } else {
        if (vpPolygon::isInside(roi, i, j)) {
          *(ptrData++) = nb;
        } else {
          ptrData++;
        }
      }
    }
    ptrData += mask->widthStep - j_max + j_min;
  }
#endif
}

/*!
  This method removes the outliers. A point is considered as outlier when its
  associated weight is below a given threshold (threshold_outlier).

  \param _w : Vector containing the weight of all the tracked points.
  \param threshold_outlier : Threshold to specify wether or not a point has to
  be deleted.
*/
void vpMbtDistanceKltPoints::removeOutliers(const vpColVector &_w, const double &threshold_outlier)
{
  std::map<int, vpImagePoint> tmp;
  std::map<int, int> tmp2;
  unsigned int nbSupp = 0;
  unsigned int k = 0;

  nbPointsCur = 0;
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for (; iter != curPoints.end(); ++iter) {
    if (_w[k] > threshold_outlier && _w[k + 1] > threshold_outlier) {
      //     if(_w[k] > threshold_outlier || _w[k+1] > threshold_outlier){
      tmp[iter->first] = vpImagePoint(iter->second.get_i(), iter->second.get_j());
      tmp2[iter->first] = curPointsInd[iter->first];
      nbPointsCur++;
    } else {
      nbSupp++;
      initPoints.erase(iter->first);
    }

    k += 2;
  }

  if (nbSupp != 0) {
    curPoints = tmp;
    curPointsInd = tmp2;
    if (nbPointsCur >= minNbPoint)
      enoughPoints = true;
    else
      enoughPoints = false;
  }
}

/*!
  Display the primitives tracked for the face.

  \param _I : The image where to display.
*/
void vpMbtDistanceKltPoints::displayPrimitive(const vpImage<unsigned char> &_I)
{
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for (; iter != curPoints.end(); ++iter) {
    int id(iter->first);
    vpImagePoint iP;
    iP.set_i(static_cast<double>(iter->second.get_i()));
    iP.set_j(static_cast<double>(iter->second.get_j()));

    vpDisplay::displayCross(_I, iP, 10, vpColor::red);

    iP.set_i(vpMath::round(iP.get_i() + 7));
    iP.set_j(vpMath::round(iP.get_j() + 7));
    std::stringstream ss;
    ss << id;
    vpDisplay::displayText(_I, iP, ss.str(), vpColor::red);
  }
}

/*!
  Display the primitives tracked for the face.

  \param _I : The image where to display.
*/
void vpMbtDistanceKltPoints::displayPrimitive(const vpImage<vpRGBa> &_I)
{
  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for (; iter != curPoints.end(); ++iter) {
    int id(iter->first);
    vpImagePoint iP;
    iP.set_i(static_cast<double>(iter->second.get_i()));
    iP.set_j(static_cast<double>(iter->second.get_j()));

    vpDisplay::displayCross(_I, iP, 10, vpColor::red);

    iP.set_i(vpMath::round(iP.get_i() + 7));
    iP.set_j(vpMath::round(iP.get_j() + 7));
    std::stringstream ss;
    ss << id;
    vpDisplay::displayText(_I, iP, ss.str(), vpColor::red);
  }
}

void vpMbtDistanceKltPoints::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix & /*cMo*/,
                                     const vpCameraParameters &camera, const vpColor &col, const unsigned int thickness,
                                     const bool displayFullModel)
{
  std::vector<std::vector<double> > models = getModelForDisplay(camera, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);

    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

void vpMbtDistanceKltPoints::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix & /*cMo*/,
                                     const vpCameraParameters &camera, const vpColor &col, const unsigned int thickness,
                                     const bool displayFullModel)
{
  std::vector<std::vector<double> > models = getModelForDisplay(camera, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);

    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

/*!
  Return a list of features parameters for display.
  - Parameters are: `<feature id (here 1 for KLT)>`, `<pt.i()>`, `<pt.j()>`,
  `<klt_id.i()>`, `<klt_id.j()>`, `<klt_id.id>`
*/
std::vector<std::vector<double> > vpMbtDistanceKltPoints::getFeaturesForDisplay()
{
  std::vector<std::vector<double> > features;

  std::map<int, vpImagePoint>::const_iterator iter = curPoints.begin();
  for (; iter != curPoints.end(); ++iter) {
    int id(iter->first);
    vpImagePoint iP;
    iP.set_i(static_cast<double>(iter->second.get_i()));
    iP.set_j(static_cast<double>(iter->second.get_j()));

    vpImagePoint iP2;
    iP2.set_i(vpMath::round(iP.get_i() + 7));
    iP2.set_j(vpMath::round(iP.get_j() + 7));

#ifdef VISP_HAVE_CXX11
    std::vector<double> params = {1, //KLT
                                  iP.get_i(),
                                  iP.get_j(),
                                  iP2.get_i(),
                                  iP2.get_j(),
                                  static_cast<double>(id)};
#else
    std::vector<double> params;
    params.push_back(1); //KLT
    params.push_back(iP.get_i());
    params.push_back(iP.get_j());
    params.push_back(iP2.get_i());
    params.push_back(iP2.get_j());
    params.push_back(static_cast<double>(id));
#endif
    features.push_back(params);
  }

  return features;
}

/*!
  Return a list of line parameters to display the primitive at a given pose and camera parameters.
  Parameters are: <primitive id (here 0 for line)>, <pt_start.i()>, <pt_start.j()>
                  <pt_end.i()>, <pt_end.j()>

  \param camera : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<std::vector<double> > vpMbtDistanceKltPoints::getModelForDisplay(const vpCameraParameters &camera,
                                                                             const bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  if ((polygon->isVisible() && isTrackedKltPoints) || displayFullModel) {
    std::vector<std::pair<vpPoint, unsigned int> > roi;
    polygon->getPolygonClipped(roi);

    for (unsigned int j = 0; j < roi.size(); j += 1) {
      if (((roi[(j + 1) % roi.size()].second & roi[j].second & vpPolygon3D::NEAR_CLIPPING) == 0) &&
          ((roi[(j + 1) % roi.size()].second & roi[j].second & vpPolygon3D::FAR_CLIPPING) == 0) &&
          ((roi[(j + 1) % roi.size()].second & roi[j].second & vpPolygon3D::DOWN_CLIPPING) == 0) &&
          ((roi[(j + 1) % roi.size()].second & roi[j].second & vpPolygon3D::UP_CLIPPING) == 0) &&
          ((roi[(j + 1) % roi.size()].second & roi[j].second & vpPolygon3D::LEFT_CLIPPING) == 0) &&
          ((roi[(j + 1) % roi.size()].second & roi[j].second & vpPolygon3D::RIGHT_CLIPPING) == 0)) {

        vpImagePoint ip1, ip2;
        std::vector<std::pair<vpPoint, vpPoint> > linesLst;

        if (useScanLine && !displayFullModel)
          hiddenface->computeScanLineQuery(roi[j].first, roi[(j + 1) % roi.size()].first, linesLst, true);
        else
          linesLst.push_back(std::make_pair(roi[j].first, roi[(j + 1) % roi.size()].first));

        for (unsigned int i = 0; i < linesLst.size(); i++) {
          linesLst[i].first.project();
          linesLst[i].second.project();
          vpMeterPixelConversion::convertPoint(camera, linesLst[i].first.get_x(), linesLst[i].first.get_y(), ip1);
          vpMeterPixelConversion::convertPoint(camera, linesLst[i].second.get_x(), linesLst[i].second.get_y(), ip2);

#ifdef VISP_HAVE_CXX11
          std::vector<double> params = {0, //0 for line parameters
                                        ip1.get_i(),
                                        ip1.get_j(),
                                        ip2.get_i(),
                                        ip2.get_j()};
#else    
          std::vector<double> params;
          params.push_back(0); //0 for line parameters
          params.push_back(ip1.get_i());
          params.push_back(ip1.get_j());
          params.push_back(ip2.get_i());
          params.push_back(ip2.get_j());
#endif
          models.push_back(params);
        }
      }
    }
  }

  return models;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(vpMbtDistanceKltPoints.cpp.o)
// has no symbols
void dummy_vpMbtDistanceKltPoints(){};
#endif
