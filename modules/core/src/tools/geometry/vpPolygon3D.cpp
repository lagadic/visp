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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <limits.h>

#include <visp3/core/vpConfig.h>
/*!
 \file vpPolygon3D.cpp
 \brief Implements a polygon of the model used by the model-based tracker.
*/

#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpPolygon3D.h>

/*!
  Basic constructor.
*/
vpPolygon3D::vpPolygon3D()
  : nbpt(0), nbCornersInsidePrev(0), p(NULL), polyClipped(), clippingFlag(vpPolygon3D::NO_CLIPPING),
    distNearClip(0.001), distFarClip(100.)
{
}

vpPolygon3D::vpPolygon3D(const vpPolygon3D &mbtp)
  : nbpt(mbtp.nbpt), nbCornersInsidePrev(mbtp.nbCornersInsidePrev), p(NULL), polyClipped(mbtp.polyClipped),
    clippingFlag(mbtp.clippingFlag), distNearClip(mbtp.distNearClip), distFarClip(mbtp.distFarClip)
{
  if (p)
    delete[] p;
  p = new vpPoint[nbpt];
  for (unsigned int i = 0; i < nbpt; i++)
    p[i] = mbtp.p[i];
}

vpPolygon3D &vpPolygon3D::operator=(const vpPolygon3D &mbtp)
{
  nbpt = mbtp.nbpt;
  nbCornersInsidePrev = mbtp.nbCornersInsidePrev;
  polyClipped = mbtp.polyClipped;
  clippingFlag = mbtp.clippingFlag;
  distNearClip = mbtp.distNearClip;
  distFarClip = mbtp.distFarClip;

  if (p)
    delete[] p;
  p = new vpPoint[nbpt];
  for (unsigned int i = 0; i < nbpt; i++)
    p[i] = mbtp.p[i];

  return (*this);
}

/*!
  Basic destructor.
*/
vpPolygon3D::~vpPolygon3D()
{
  if (p != NULL) {
    delete[] p;
    p = NULL;
  }
}

/*!
  Get a reference to a corner.

  \throw vpException::dimensionError if the _index is out of range.

  \param _index : the index of the corner
*/
vpPoint &vpPolygon3D::getPoint(const unsigned int _index)
{
  if (_index >= nbpt) {
    throw vpException(vpException::dimensionError, "index out of range");
  }
  return p[_index];
}

/*!
  Set the number of points which are the corners of the polygon.

  \param nb : The number of corners.
*/
void vpPolygon3D::setNbPoint(const unsigned int nb)
{
  nbpt = nb;
  if (p != NULL)
    delete[] p;
  p = new vpPoint[nb];
}

/*!
  Add a corner point to the list of polygon's corners.

  \param n : The index of the corner.
  \param P : The point to add.
*/
void vpPolygon3D::addPoint(const unsigned int n, const vpPoint &P)
{
  // if( p!NULL && n < nbpt )
  p[n] = P;
}

/*!
  Project the 3D corner points into the image thanks to the pose of the
  camera.

  \param cMo : The pose of the camera.
*/
void vpPolygon3D::changeFrame(const vpHomogeneousMatrix &cMo)
{
  for (unsigned int i = 0; i < nbpt; i++) {
    p[i].changeFrame(cMo);
    p[i].projection();
  }
}

/*!
  Compute the region of interest in the image according to the used clipping.

  \warning If the FOV clipping is used, camera normals have to be precomputed.

  \param cam : camera parameters used to compute the field of view.
*/
void vpPolygon3D::computePolygonClipped(const vpCameraParameters &cam)
{
  polyClipped.clear();
  std::vector<vpColVector> fovNormals;
  std::vector<std::pair<vpPoint, unsigned int> > polyClippedTemp;
  std::vector<std::pair<vpPoint, unsigned int> > polyClippedTemp2;

  if (cam.isFovComputed() && clippingFlag > 3)
    fovNormals = cam.getFovNormals();

  for (unsigned int i = 0; i < nbpt; i++) {
    p[i % nbpt].projection();
    polyClippedTemp.push_back(std::make_pair(p[i % nbpt], vpPolygon3D::NO_CLIPPING));
  }

  if (clippingFlag != vpPolygon3D::NO_CLIPPING) {
    for (unsigned int i = 1; i < 64; i = i * 2) {
      if (((clippingFlag & i) == i) || ((clippingFlag > vpPolygon3D::FAR_CLIPPING) && (i == 1))) {
        if (i > vpPolygon3D::FAR_CLIPPING && !cam.isFovComputed()) // To make sure we do not compute FOV
                                                                   // clipping if camera normals are not
                                                                   // computed
          continue;

        for (unsigned int j = 0; j < polyClippedTemp.size(); j++) {
          vpPoint p1Clipped = polyClippedTemp[j].first;
          vpPoint p2Clipped = polyClippedTemp[(j + 1) % polyClippedTemp.size()].first;

          unsigned int p2ClippedInfoBefore = polyClippedTemp[(j + 1) % polyClippedTemp.size()].second;
          unsigned int p1ClippedInfo = polyClippedTemp[j].second;
          unsigned int p2ClippedInfo = polyClippedTemp[(j + 1) % polyClippedTemp.size()].second;

          bool problem = true;

          switch (i) {
          case 1:
            problem = !(vpPolygon3D::getClippedPointsDistance(p1Clipped, p2Clipped, p1Clipped, p2Clipped, p1ClippedInfo,
                                                              p2ClippedInfo, i, distNearClip));
            break;
          case 2:
            problem = !(vpPolygon3D::getClippedPointsDistance(p1Clipped, p2Clipped, p1Clipped, p2Clipped, p1ClippedInfo,
                                                              p2ClippedInfo, i, distFarClip));
            break;
          case 4:
            problem =
                !(vpPolygon3D::getClippedPointsFovGeneric(p1Clipped, p2Clipped, p1Clipped, p2Clipped, p1ClippedInfo,
                                                          p2ClippedInfo, fovNormals[0], vpPolygon3D::LEFT_CLIPPING));
            break;
          case 8:
            problem =
                !(vpPolygon3D::getClippedPointsFovGeneric(p1Clipped, p2Clipped, p1Clipped, p2Clipped, p1ClippedInfo,
                                                          p2ClippedInfo, fovNormals[1], vpPolygon3D::RIGHT_CLIPPING));
            break;
          case 16:
            problem =
                !(vpPolygon3D::getClippedPointsFovGeneric(p1Clipped, p2Clipped, p1Clipped, p2Clipped, p1ClippedInfo,
                                                          p2ClippedInfo, fovNormals[2], vpPolygon3D::UP_CLIPPING));
            break;
          case 32:
            problem =
                !(vpPolygon3D::getClippedPointsFovGeneric(p1Clipped, p2Clipped, p1Clipped, p2Clipped, p1ClippedInfo,
                                                          p2ClippedInfo, fovNormals[3], vpPolygon3D::DOWN_CLIPPING));
            break;
          }

          if (!problem) {
            p1Clipped.projection();
            polyClippedTemp2.push_back(std::make_pair(p1Clipped, p1ClippedInfo));

            if (p2ClippedInfo != p2ClippedInfoBefore) {
              p2Clipped.projection();
              polyClippedTemp2.push_back(std::make_pair(p2Clipped, p2ClippedInfo));
            }

            if (nbpt == 2) {
              if (p2ClippedInfo == p2ClippedInfoBefore) {
                p2Clipped.projection();
                polyClippedTemp2.push_back(std::make_pair(p2Clipped, p2ClippedInfo));
              }
              break;
            }
          }
        }

        polyClippedTemp = polyClippedTemp2;
        polyClippedTemp2.clear();
      }
    }
  }

  polyClipped = polyClippedTemp;
}

/*!
  Get the clipped points according to a plane equation.

  \param cam : camera parameters
  \param p1 : First extremity of the line.
  \param p2 : Second extremity of the line.
  \param p1Clipped : Resulting p1.
  \param p2Clipped : Resulting p2.
  \param p1ClippedInfo : Resulting clipping flag for p1.
  \param p2ClippedInfo : Resulting clipping flag for p2.
  \param A : Param A from plane equation.
  \param B : Param B from plane equation.
  \param C : Param C from plane equation.
  \param D : Param D from plane equation.
  \param flag : flag specifying the clipping used when calling this function.

  \return True if the points have been clipped, False otherwise
*/
bool vpPolygon3D::getClippedPointsFovGeneric(const vpPoint &p1, const vpPoint &p2, vpPoint &p1Clipped,
                                             vpPoint &p2Clipped, unsigned int &p1ClippedInfo,
                                             unsigned int &p2ClippedInfo, const vpColVector &normal,
                                             const unsigned int &flag)
{
  vpRowVector p1Vec(3);
  p1Vec[0] = p1.get_X();
  p1Vec[1] = p1.get_Y();
  p1Vec[2] = p1.get_Z();
  p1Vec.normalize();

  vpRowVector p2Vec(3);
  p2Vec[0] = p2.get_X();
  p2Vec[1] = p2.get_Y();
  p2Vec[2] = p2.get_Z();
  p2Vec.normalize();

  if ((clippingFlag & flag) == flag) {
    double beta1 = acos(p1Vec * normal);
    double beta2 = acos(p2Vec * normal);

    //    std::cout << beta1 << " && " << beta2 << std::endl;

    //    if(!(beta1 < M_PI / 2.0 && beta2 < M_PI / 2.0))
    if (beta1 < M_PI / 2.0 && beta2 < M_PI / 2.0)
      return false;
    else if (beta1 < M_PI / 2.0 || beta2 < M_PI / 2.0) {
      vpPoint pClipped;
      double t = -(normal[0] * p1.get_X() + normal[1] * p1.get_Y() + normal[2] * p1.get_Z());
      t = t / (normal[0] * (p2.get_X() - p1.get_X()) + normal[1] * (p2.get_Y() - p1.get_Y()) +
               normal[2] * (p2.get_Z() - p1.get_Z()));

      pClipped.set_X((p2.get_X() - p1.get_X()) * t + p1.get_X());
      pClipped.set_Y((p2.get_Y() - p1.get_Y()) * t + p1.get_Y());
      pClipped.set_Z((p2.get_Z() - p1.get_Z()) * t + p1.get_Z());

      if (beta1 < M_PI / 2.0) {
        p1ClippedInfo = p1ClippedInfo | flag;
        p1Clipped = pClipped;
      } else {
        p2ClippedInfo = p2ClippedInfo | flag;
        p2Clipped = pClipped;
      }
    }
  }

  return true;
}

bool vpPolygon3D::getClippedPointsDistance(const vpPoint &p1, const vpPoint &p2, vpPoint &p1Clipped, vpPoint &p2Clipped,
                                           unsigned int &p1ClippedInfo, unsigned int &p2ClippedInfo,
                                           const unsigned int &flag, const double &distance)
{
  // Since p1 and p1Clipped can be the same object as well as p2 and p2Clipped
  // to avoid a valgrind "Source and destination overlap in memcpy" error,
  // we introduce a two temporary points.
  vpPoint p1Clipped_, p2Clipped_;
  p1Clipped_ = p1;
  p2Clipped_ = p2;

  p1Clipped = p1Clipped_;
  p2Clipped = p2Clipped_;

  bool test1 = (p1Clipped.get_Z() < distance && p2Clipped.get_Z() < distance);
  if (flag == vpPolygon3D::FAR_CLIPPING)
    test1 = (p1Clipped.get_Z() > distance && p2Clipped.get_Z() > distance);

  bool test2 = (p1Clipped.get_Z() < distance || p2Clipped.get_Z() < distance);
  if (flag == vpPolygon3D::FAR_CLIPPING)
    test2 = (p1Clipped.get_Z() > distance || p2Clipped.get_Z() > distance);

  bool test3 = (p1Clipped.get_Z() < distance);
  if (flag == vpPolygon3D::FAR_CLIPPING)
    test3 = (p1Clipped.get_Z() > distance);

  if (test1)
    return false;

  else if (test2) {
    vpPoint pClippedNear;
    double t;
    t = (p2Clipped.get_Z() - p1Clipped.get_Z());
    t = (distance - p1Clipped.get_Z()) / t;

    pClippedNear.set_X((p2Clipped.get_X() - p1Clipped.get_X()) * t + p1Clipped.get_X());
    pClippedNear.set_Y((p2Clipped.get_Y() - p1Clipped.get_Y()) * t + p1Clipped.get_Y());
    pClippedNear.set_Z(distance);

    if (test3) {
      p1Clipped = pClippedNear;
      if (flag == vpPolygon3D::FAR_CLIPPING)
        p1ClippedInfo = p1ClippedInfo | vpPolygon3D::FAR_CLIPPING;
      else
        p1ClippedInfo = p1ClippedInfo | vpPolygon3D::NEAR_CLIPPING;
    } else {
      p2Clipped = pClippedNear;
      if (flag == vpPolygon3D::FAR_CLIPPING)
        p2ClippedInfo = p2ClippedInfo | vpPolygon3D::FAR_CLIPPING;
      else
        p2ClippedInfo = p2ClippedInfo | vpPolygon3D::NEAR_CLIPPING;
    }
  }

  return true;
}

/*!
  Get the region of interest in the image.

  \warning Suppose that changeFrame() has already been called.

  \param cam : camera parameters.

  \return Image point corresponding to the region of interest.
*/
std::vector<vpImagePoint> vpPolygon3D::getRoi(const vpCameraParameters &cam)
{
  std::vector<vpImagePoint> roi;
  for (unsigned int i = 0; i < nbpt; i++) {
    vpImagePoint ip;
    vpMeterPixelConversion::convertPoint(cam, p[i].get_x(), p[i].get_y(), ip);
    roi.push_back(ip);
  }

  return roi;
}

/*!
  Get the region of interest in the image.

  \param cam : camera parameters.
  \param cMo : pose.

  \return Image point corresponding to the region of interest.
*/
std::vector<vpImagePoint> vpPolygon3D::getRoi(const vpCameraParameters &cam, const vpHomogeneousMatrix &cMo)
{
  changeFrame(cMo);
  return getRoi(cam);
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  Get the 3D points of the clipped region of interest.

  \warning Suppose that changeFrame() and computePolygonClipped() have already
  been called.

  \param points : resulting points.
*/
void vpPolygon3D::getRoiClipped(std::vector<vpPoint> &points)
{
  for (unsigned int i = 0; i < polyClipped.size(); i++) {
    points.push_back(polyClipped[i].first);
  }
}
#endif

/*!
  Get the 3D clipped points and their clipping information.

  \warning Suppose that changeFrame() and computePolygonClipped() have already
  been called.

  \param poly : resulting points plus clipping information.
*/
void vpPolygon3D::getPolygonClipped(std::vector<std::pair<vpPoint, unsigned int> > &poly) { poly = polyClipped; }

/*!
  Get the 3D clipped points.

  \warning Suppose that changeFrame() and computePolygonClipped() have already
  been called.

  \param poly : resulting points.
*/
void vpPolygon3D::getPolygonClipped(std::vector<vpPoint> &poly)
{
  for (unsigned int i = 0; i < polyClipped.size(); i++) {
    poly.push_back(polyClipped[i].first);
  }
}

/*!
  Get the region of interest clipped in the image.

  \warning Suppose that changeFrame() and computePolygonClipped() have already
  been called.

  \param cam : camera parameters.
  \param roi : image point corresponding to the region of interest.
*/
void vpPolygon3D::getRoiClipped(const vpCameraParameters &cam, std::vector<vpImagePoint> &roi)
{
  for (unsigned int i = 0; i < polyClipped.size(); i++) {
    vpImagePoint ip;
    vpMeterPixelConversion::convertPoint(cam, polyClipped[i].first.get_x(), polyClipped[i].first.get_y(), ip);
    //    std::cout << "## " << ip.get_j() << " - " << ip.get_i() <<
    //    std::endl;
    roi.push_back(ip);
  }
}

/*!
  Get the region of interest clipped in the image.

  \param cam : camera parameters.
  \param cMo : pose.
  \param roi : image point corresponding to the region of interest.
*/
void vpPolygon3D::getRoiClipped(const vpCameraParameters &cam, std::vector<vpImagePoint> &roi,
                                const vpHomogeneousMatrix &cMo)
{
  changeFrame(cMo);
  computePolygonClipped(cam);
  getRoiClipped(cam, roi);
}

/*!
  Get the region of interest clipped in the image and the information to know
  if it's a clipped point.

  \warning Suppose that changeFrame() and computePolygonClipped() have already
  been called.

  \param cam : camera parameters.
  \param roi : image point corresponding to the region of interest with
  clipping information.
*/
void vpPolygon3D::getRoiClipped(const vpCameraParameters &cam, std::vector<std::pair<vpImagePoint, unsigned int> > &roi)
{
  for (unsigned int i = 0; i < polyClipped.size(); i++) {
    vpImagePoint ip;
    polyClipped[i].first.projection();
    vpMeterPixelConversion::convertPoint(cam, polyClipped[i].first.get_x(), polyClipped[i].first.get_y(), ip);
    roi.push_back(std::make_pair(ip, polyClipped[i].second));
  }
}

/*!
  Get the region of interest clipped in the image and the information to know
  if it's a clipped point.

  \param cam : camera parameters.
  \param roi : image point corresponding to the region of interest with
  clipping information. \param cMo : pose.
*/
void vpPolygon3D::getRoiClipped(const vpCameraParameters &cam, std::vector<std::pair<vpImagePoint, unsigned int> > &roi,
                                const vpHomogeneousMatrix &cMo)
{
  changeFrame(cMo);
  computePolygonClipped(cam);
  getRoiClipped(cam, roi);
}

/*!
  Static method to check the number of points of a region defined by the
  vector of image point that are inside the image.

  \param I : The image used for its size.
  \param cam : The camera parameters.
*/
unsigned int vpPolygon3D::getNbCornerInsideImage(const vpImage<unsigned char> &I, const vpCameraParameters &cam)
{
  unsigned int nbPolyIn = 0;
  for (unsigned int i = 0; i < nbpt; i++) {
    if (p[i].get_Z() > 0) {
      vpImagePoint ip;
      vpMeterPixelConversion::convertPoint(cam, p[i].get_x(), p[i].get_y(), ip);
      if ((ip.get_i() >= 0) && (ip.get_j() >= 0) && (ip.get_i() < I.getHeight()) && (ip.get_j() < I.getWidth()))
        nbPolyIn++;
    }
  }

  nbCornersInsidePrev = nbPolyIn;

  return nbPolyIn;
}

//###################################
//      Static functions
//###################################

/*!
  Static method to compute the clipped points from a set of initial points.

  \warning When using FOV clipping and personnal camera parameters, camera
  normals have to be computed before (see vpCameraParameters::computeFov())

  \param ptIn : Input points
  \param ptOut : Output points (result of the clipping).
  \param cMo : Pose considered for the clipping.
  \param clippingFlags: Clipping flag (see
  vpPolygon3D::vpPolygon3DClippingType). \param cam : Camera parameters (Only
  used if clipping flags contain FOV clipping). \param znear : Near clipping
  distance value (Only used if clipping flags contain Near clipping). \param
  zfar : Far clipping distance value (Only used if clipping flags contain Far
  clipping).
*/
void vpPolygon3D::getClippedPolygon(const std::vector<vpPoint> &ptIn, std::vector<vpPoint> &ptOut,
                                    const vpHomogeneousMatrix &cMo, const unsigned int &clippingFlags,
                                    const vpCameraParameters &cam, const double &znear, const double &zfar)
{
  ptOut.clear();
  vpPolygon3D poly;
  poly.setNbPoint((unsigned int)ptIn.size());
  poly.setClipping(clippingFlags);

  if ((clippingFlags & vpPolygon3D::NEAR_CLIPPING) == vpPolygon3D::NEAR_CLIPPING)
    poly.setNearClippingDistance(znear);

  if ((clippingFlags & vpPolygon3D::FAR_CLIPPING) == vpPolygon3D::FAR_CLIPPING)
    poly.setFarClippingDistance(zfar);

  for (unsigned int i = 0; i < ptIn.size(); i++)
    poly.addPoint(i, ptIn[i]);

  poly.changeFrame(cMo);
  poly.computePolygonClipped(cam);
  poly.getPolygonClipped(ptOut);
}

void vpPolygon3D::getMinMaxRoi(const std::vector<vpImagePoint> &iroi, int &i_min, int &i_max, int &j_min, int &j_max)
{
  // i_min_d = std::numeric_limits<double>::max(); // create an error under
  // Windows. To fix it we have to add #undef max
  double i_min_d = (double)INT_MAX;
  double i_max_d = 0;
  double j_min_d = (double)INT_MAX;
  double j_max_d = 0;

  for (unsigned int i = 0; i < iroi.size(); i += 1) {
    if (i_min_d > iroi[i].get_i())
      i_min_d = iroi[i].get_i();

    if (iroi[i].get_i() < 0)
      i_min_d = 1;

    if ((iroi[i].get_i() > 0) && (i_max_d < iroi[i].get_i()))
      i_max_d = iroi[i].get_i();

    if (j_min_d > iroi[i].get_j())
      j_min_d = iroi[i].get_j();

    if (iroi[i].get_j() < 0)
      j_min_d = 1; // border

    if ((iroi[i].get_j() > 0) && j_max_d < iroi[i].get_j())
      j_max_d = iroi[i].get_j();
  }
  i_min = static_cast<int>(i_min_d);
  i_max = static_cast<int>(i_max_d);
  j_min = static_cast<int>(j_min_d);
  j_max = static_cast<int>(j_max_d);
}

/*!
  Static method to check whether the region defined by the vector of image
  point is contained entirely in the image.

  \param I : The image used for its size.
  \param corners : The vector of points defining a region
*/
bool vpPolygon3D::roiInsideImage(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &corners)
{
  double nbPolyIn = 0;
  for (unsigned int i = 0; i < corners.size(); ++i) {
    if ((corners[i].get_i() >= 0) && (corners[i].get_j() >= 0) && (corners[i].get_i() < I.getHeight()) &&
        (corners[i].get_j() < I.getWidth())) {
      nbPolyIn++;
    }
  }

  if (nbPolyIn < 3 && nbPolyIn < 0.7 * corners.size())
    return false;

  return true;
}
