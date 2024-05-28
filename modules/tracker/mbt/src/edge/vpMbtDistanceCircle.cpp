/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Make the complete tracking of an object by using its CAD model. Circle
 * tracking.
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

/*!
 \file vpMbtDistanceCircle.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/

#include <algorithm>
#include <stdlib.h>

#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPlane.h>
#include <visp3/mbt/vpMbtDistanceCircle.h>
#include <visp3/vision/vpPose.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureEllipse.h>

BEGIN_VISP_NAMESPACE
/*!
  Basic constructor
*/
vpMbtDistanceCircle::vpMbtDistanceCircle()
  : name(), index(0), cam(), me(nullptr), wmean(1), featureEllipse(), isTrackedCircle(true), meEllipse(nullptr), circle(nullptr),
  radius(0.), p1(nullptr), p2(nullptr), p3(nullptr), L(), error(), nbFeature(0), Reinit(false), hiddenface(nullptr),
  index_polygon(-1), isvisible(false)
{ }

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbtDistanceCircle::~vpMbtDistanceCircle()
{
  if (meEllipse != nullptr)
    delete meEllipse;
  if (circle != nullptr)
    delete circle;
  if (p1 != nullptr)
    delete p1;
  if (p2 != nullptr)
    delete p2;
  if (p3 != nullptr)
    delete p3;
}

/*!
  Project the circle into the image.

  \param cMo : The pose of the camera used to project the circle into the
  image.
*/
void vpMbtDistanceCircle::project(const vpHomogeneousMatrix &cMo) { circle->project(cMo); }

/*!
  Build a vpMbtDistanceCircle thanks to its center, 3 points (including the
  center) with coordinates expressed in the object frame and defining the
  plane that contain the circle and its radius.

  \param _p1 : Center of the circle.
  \param _p2,_p3 : Two points on the plane containing the circle. With the
  center of the circle we have 3 points defining the plane that contains the
  circle. \param r : Radius of the circle.
*/
void vpMbtDistanceCircle::buildFrom(const vpPoint &_p1, const vpPoint &_p2, const vpPoint &_p3, double r)
{
  circle = new vpCircle;
  p1 = new vpPoint;
  p2 = new vpPoint;
  p3 = new vpPoint;

  // Get the points
  *p1 = _p1;
  *p2 = _p2;
  *p3 = _p3;

  // Get the radius
  radius = r;

  vpPlane plane(*p1, *p2, *p3, vpPlane::object_frame);

  // Build our circle
  circle->setWorldCoordinates(plane.getA(), plane.getB(), plane.getC(), _p1.get_oX(), _p1.get_oY(), _p1.get_oZ(), r);
}

/*!
  Set the moving edge parameters.

  \param _me : an instance of vpMe containing all the desired parameters
*/
void vpMbtDistanceCircle::setMovingEdge(vpMe *_me)
{
  me = _me;
  if (meEllipse != nullptr) {
    meEllipse->setMe(me);
  }
}

/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the
  circle.

  \param I : The image.
  \param cMo : The pose of the camera used to initialize the moving edges.
  \param doNotTrack : If true, ME are not tracked.
  \param mask: Mask image or nullptr if not wanted. Mask values that are set to true are considered in the tracking. To
  disable a pixel, set false. \return false if an error occur, true otherwise.
*/
bool vpMbtDistanceCircle::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                         bool doNotTrack, const vpImage<bool> *mask)
{
  if (isvisible) {
    // Perspective projection
    circle->changeFrame(cMo);

    try {
      circle->projection();
    }
    catch (...) {
      std::cout << "Problem when projecting circle\n";
      return false;
    }

    // Create the moving edges containers
    meEllipse = new vpMbtMeEllipse;
    meEllipse->setMask(*mask);
    meEllipse->setMe(me);

    // meEllipse->setDisplay(vpMeSite::RANGE_RESULT) ; // TODO only for debug
    meEllipse->setInitRange(me->getRange()); // TODO: check because set to zero for lines

    try {
      vpImagePoint ic;
      double n20_p, n11_p, n02_p;
      vpMeterPixelConversion::convertEllipse(cam, *circle, ic, n20_p, n11_p, n02_p);
      meEllipse->initTracking(I, ic, n20_p, n11_p, n02_p, doNotTrack);
    }
    catch (...) {
   // vpTRACE("the circle can't be initialized");
      return false;
    }
  }
  return true;
}

/*!
  Track the moving edges in the image.

  \param I : the image.
  \param cMo : The pose of the camera.
*/
void vpMbtDistanceCircle::trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix & /*cMo*/)
{
  if (isvisible) {
    try {
      meEllipse->track(I);
    }
    catch (...) {
   // std::cout << "Track meEllipse failed" << std::endl;
      meEllipse->reset();
      Reinit = true;
    }

    // Update the number of features
    nbFeature = (unsigned int)meEllipse->getMeList().size();
  }
}

/*!
  Update the moving edges internal parameters.

  \warning : Not implemented.

  \param I : the image.
  \param cMo : The pose of the camera.
*/
void vpMbtDistanceCircle::updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
  if (isvisible) {
    // Perspective projection
    circle->changeFrame(cMo);

    try {
      circle->projection();
    }
    catch (...) {
      std::cout << "Problem when projecting circle\n";
    }

    try {

      vpImagePoint ic;
      double n20_p, n11_p, n02_p;
      vpMeterPixelConversion::convertEllipse(cam, *circle, ic, n20_p, n11_p, n02_p);
      meEllipse->updateParameters(I, ic, n20_p, n11_p, n02_p);
    }
    catch (...) {
      Reinit = true;
    }
    nbFeature = (unsigned int)meEllipse->getMeList().size();
  }
}

/*!
  Reinitialize the circle if it is required.

  A circle is reinitialized if the ellipse do not match enough with the
  projected 3D circle.

  \param I : the image.
  \param cMo : The pose of the camera.
  \param mask: Mask image or nullptr if not wanted. Mask values that are set to true are considered in the tracking. To
  disable a pixel, set false.
*/
void vpMbtDistanceCircle::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                           const vpImage<bool> *mask)
{
  if (meEllipse != nullptr)
    delete meEllipse;

  meEllipse = nullptr;

  if (!initMovingEdge(I, cMo, false, mask))
    Reinit = true;

  Reinit = false;
}

/*!
  Display the circle. The 3D circle is projected into the image as an ellipse.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : When true, display the circle even if non visible.
  If false, display the circle only if visible.
*/
void vpMbtDistanceCircle::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                  const vpCameraParameters &camera, const vpColor &col, unsigned int thickness,
                                  bool displayFullModel)
{
  std::vector<double> params = getModelForDisplay(cMo, camera, displayFullModel);

  vpImagePoint center(params[0], params[1]);
  double n20_p = params[2];
  double n11_p = params[3];
  double n02_p = params[4];
  vpDisplay::displayEllipse(I, center, n20_p, n11_p, n02_p, true, col, thickness);
}

/*!
  Display the cylinder. The 3D cylinder is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : When true, display the circle even if non visible.
  If false, display the circle only if visible.
*/
void vpMbtDistanceCircle::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                                  const vpCameraParameters &camera, const vpColor &col, unsigned int thickness,
                                  bool displayFullModel)
{
  std::vector<double> params = getModelForDisplay(cMo, camera, displayFullModel);

  vpImagePoint center(params[1], params[2]);
  double n20_p = params[3];
  double n11_p = params[4];
  double n02_p = params[5];
  vpDisplay::displayEllipse(I, center, n20_p, n11_p, n02_p, true, col, thickness);
}

/*!
  Return a list of features parameters for display.
  - Parameters are: `<feature id (here 0 for ME)>`, `<pt.i()>`, `<pt.j()>`, `<state>`
*/
std::vector<std::vector<double> > vpMbtDistanceCircle::getFeaturesForDisplay()
{
  std::vector<std::vector<double> > features;

  if (meEllipse != nullptr) {
    for (std::list<vpMeSite>::const_iterator it = meEllipse->getMeList().begin(); it != meEllipse->getMeList().end();
         ++it) {
      vpMeSite p_me = *it;
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
      std::vector<double> params = { 0, //ME
                                    p_me.get_ifloat(),
                                    p_me.get_jfloat(),
                                    static_cast<double>(p_me.getState()) };
#else
      std::vector<double> params;
      params.push_back(0); //ME
      params.push_back(p_me.get_ifloat());
      params.push_back(p_me.get_jfloat());
      params.push_back(static_cast<double>(p_me.getState()));
#endif

      features.push_back(params);
    }
  }

  return features;
}

/*!
  Return a list of ellipse parameters to display the primitive at a given pose and camera parameters.
  - Parameters are: `<primitive id (here 1 for ellipse)>`, `<pt_center.i()>`, `<pt_center.j()>`,
  `<n20>`, `<n11>`, `<n02>` which are respectively the ellipse centroid coordinates and
  second order centered moments of the ellipse normalized by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$
  where \f$\mu_{ij}\f$ are the centered moments and a the area).

  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<double> vpMbtDistanceCircle::getModelForDisplay(const vpHomogeneousMatrix &cMo,
                                                            const vpCameraParameters &camera, bool displayFullModel)
{
  std::vector<double> params;

  if ((isvisible && isTrackedCircle) || displayFullModel) {
    // Perspective projection
    circle->changeFrame(cMo);

    try {
      circle->projection();
    }
    catch (...) {
      std::cout << "Cannot project the circle";
    }

    vpImagePoint center;
    double n20_p, n11_p, n02_p;
    vpMeterPixelConversion::convertEllipse(camera, *circle, center, n20_p, n11_p, n02_p);
    params.push_back(1); // 1 for ellipse parameters
    params.push_back(center.get_i());
    params.push_back(center.get_j());
    params.push_back(n20_p);
    params.push_back(n11_p);
    params.push_back(n02_p);
  }

  return params;
}

/*!
   Enable to display the points along the ellipse with a color corresponding
   to their state.

   - If green : The vpMeSite is a good point.
   - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
   - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
   - If blue : The point is removed because of the robust method in the virtual visual servoing.

   \param I : The image.
*/
void vpMbtDistanceCircle::displayMovingEdges(const vpImage<unsigned char> &I)
{
  if (meEllipse != nullptr) {
    meEllipse->display(I); // display the me
    if (vpDEBUG_ENABLE(3))
      vpDisplay::flush(I);
  }
}

void vpMbtDistanceCircle::displayMovingEdges(const vpImage<vpRGBa> &I)
{
  if (meEllipse != nullptr) {
    meEllipse->display(I); // display the me
    if (vpDEBUG_ENABLE(3))
      vpDisplay::flush(I);
  }
}

/*!
  Initialize the size of the interaction matrix and the error vector.
*/
void vpMbtDistanceCircle::initInteractionMatrixError()
{
  if (isvisible) {
    nbFeature = (unsigned int)meEllipse->getMeList().size();
    L.resize(nbFeature, 6);
    error.resize(nbFeature);
  }
  else
    nbFeature = 0;
}

/*!
  Compute the interaction matrix and the error vector corresponding to the
  point to ellipse algebraic distance.
*/
void vpMbtDistanceCircle::computeInteractionMatrixError(const vpHomogeneousMatrix &cMo)
{
  if (isvisible) {
    // Perspective projection
    circle->changeFrame(cMo);
    try {
      circle->projection();
    }
    catch (...) {
      std::cout << "Problem projection circle\n";
    }

    vpFeatureBuilder::create(featureEllipse, *circle);

    vpMatrix H1 = featureEllipse.interaction();

    vpRowVector H(5);
    double x = 0, y = 0;

    // Get the parameters of the ellipse in the image plane
    double xg = circle->p[0];
    double yg = circle->p[1];
    double n20 = circle->p[2];
    double n11 = circle->p[3];
    double n02 = circle->p[4];

    unsigned int j = 0;

    for (std::list<vpMeSite>::const_iterator it = meEllipse->getMeList().begin(); it != meEllipse->getMeList().end();
         ++it) {
      vpPixelMeterConversion::convertPoint(cam, it->m_j, it->m_i, x, y);
      // TRO Chaumette 2004 eq 25
      H[0] = 2 * (n11 * (y - yg) + n02 * (xg - x));
      H[1] = 2 * (n20 * (yg - y) + n11 * (x - xg));
      H[2] = vpMath::sqr(y - yg) - 4.0 * n02;
      H[3] = 2 * (yg * (x - xg) + y * xg + 4.0 * n11 - x * y);
      H[4] = vpMath::sqr(x - xg) - 4.0 * n20;

      for (unsigned int k = 0; k < 6; k++)
        L[j][k] = H[0] * H1[0][k] + H[1] * H1[1][k] + H[2] * H1[2][k] + H[3] * H1[3][k] + H[4] * H1[4][k];

      error[j] = n02 * vpMath::sqr(x) + n20 * vpMath::sqr(y) - 2 * n11 * x * y + 2 * (n11 * yg - n02 * xg) * x +
        2 * (n11 * xg - n20 * yg) * y + n02 * vpMath::sqr(xg) + n20 * vpMath::sqr(yg) - 2 * n11 * xg * yg +
        4.0 * vpMath::sqr(n11) - 4.0 * n20 * n02;
      j++;
    }
  }
}
END_VISP_NAMESPACE
