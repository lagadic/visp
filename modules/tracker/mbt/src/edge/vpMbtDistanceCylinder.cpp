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
 * Make the complete tracking of an object by using its CAD model. Cylinder
 * tracking.
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 * Bertrand Delabarre
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

/*!
 \file vpMbtDistanceCylinder.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/

#include <algorithm>
#include <stdlib.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpPlane.h>
#include <visp3/mbt/vpMbtDistanceCylinder.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/visual_features/vpFeatureEllipse.h>

#include <visp3/vision/vpPose.h>

/*!
  Basic constructor
*/
vpMbtDistanceCylinder::vpMbtDistanceCylinder()
  : name(), index(0), cam(), me(NULL), wmean1(1), wmean2(1), featureline1(), featureline2(), isTrackedCylinder(true),
    meline1(NULL), meline2(NULL), cercle1(NULL), cercle2(NULL), radius(0), p1(NULL), p2(NULL), L(), error(),
    nbFeature(0), nbFeaturel1(0), nbFeaturel2(0), Reinit(false), c(NULL), hiddenface(NULL), index_polygon(-1),
    isvisible(false)
{
}

/*!
  Basic destructor useful to deallocate the memory.
*/
vpMbtDistanceCylinder::~vpMbtDistanceCylinder()
{
  //	cout << "Deleting cylinder " << index << endl;

  if (p1 != NULL)
    delete p1;
  if (p2 != NULL)
    delete p2;
  if (c != NULL)
    delete c;
  if (meline1 != NULL)
    delete meline1;
  if (meline2 != NULL)
    delete meline2;
  if (cercle1 != NULL)
    delete cercle1;
  if (cercle2 != NULL)
    delete cercle2;
}

/*!
  Project the cylinder and the two points corresponding to its extremities
  into the image.

  \param cMo : The pose of the camera used to project the cylinder into the
  image.
*/
void vpMbtDistanceCylinder::project(const vpHomogeneousMatrix &cMo)
{
  c->project(cMo);
  p1->project(cMo);
  p2->project(cMo);
  cercle1->project(cMo);
  cercle2->project(cMo);
}

/*!
  Build a vpMbtDistanceCylinder thanks to two points corresponding to the
  extremities of its axis and its radius.

  \param _p1 : The first extremity on the axis.
  \param _p2 : The second extremity on the axis.
  \param r : Radius of the cylinder.
*/
void vpMbtDistanceCylinder::buildFrom(const vpPoint &_p1, const vpPoint &_p2, const double r)
{
  c = new vpCylinder;
  p1 = new vpPoint;
  p2 = new vpPoint;
  cercle1 = new vpCircle;
  cercle2 = new vpCircle;

  // Get the points
  *p1 = _p1;
  *p2 = _p2;

  // Get the radius
  radius = r;

  vpColVector ABC(3);
  vpColVector V1(3);
  vpColVector V2(3);

  V1[0] = _p1.get_oX();
  V1[1] = _p1.get_oY();
  V1[2] = _p1.get_oZ();
  V2[0] = _p2.get_oX();
  V2[1] = _p2.get_oY();
  V2[2] = _p2.get_oZ();

  // Get the axis of the cylinder
  ABC = V1 - V2;

  // Build our extremity circles
  cercle1->setWorldCoordinates(ABC[0], ABC[1], ABC[2], _p1.get_oX(), _p1.get_oY(), _p1.get_oZ(), r);
  cercle2->setWorldCoordinates(ABC[0], ABC[1], ABC[2], _p2.get_oX(), _p2.get_oY(), _p2.get_oZ(), r);

  // Build our cylinder
  c->setWorldCoordinates(ABC[0], ABC[1], ABC[2], (_p1.get_oX() + _p2.get_oX()) / 2.0,
                         (_p1.get_oY() + _p2.get_oY()) / 2.0, (_p1.get_oZ() + _p2.get_oZ()) / 2.0, r);
}

/*!
  Set the moving edge parameters.

  \param _me : an instance of vpMe containing all the desired parameters
*/
void vpMbtDistanceCylinder::setMovingEdge(vpMe *_me)
{
  me = _me;
  if (meline1 != NULL) {
    meline1->setMe(me);
  }
  if (meline2 != NULL) {
    meline2->setMe(me);
  }
}

/*!
  Initialize the moving edge thanks to a given pose of the camera.
  The 3D model is projected into the image to create moving edges along the
  lines.

  \param I : The image.
  \param cMo : The pose of the camera used to initialize the moving edges.
  \param doNotTrack : If true, ME are not tracked.
  \param mask: Mask image or NULL if not wanted. Mask values that are set to true are considered in the tracking. To disable a pixel, set false.
  \return false if an error occur, true otherwise.
*/
bool vpMbtDistanceCylinder::initMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const bool doNotTrack,
                                           const vpImage<bool> *mask)
{
  if (isvisible) {
    // Perspective projection
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);
    cercle1->changeFrame(cMo);
    cercle2->changeFrame(cMo);
    c->changeFrame(cMo);

    p1->projection();
    p2->projection();
    try {
      cercle1->projection();
    } catch (...) {
      // std::cout<<"Problem when projecting circle 1\n";
      return false;
    }
    try {
      cercle2->projection();
    } catch (...) {
      // std::cout<<"Problem when projecting circle 2\n";
      return false;
    }
    c->projection();

    double rho1, theta1;
    double rho2, theta2;

    // Create the moving edges containers
    meline1 = new vpMbtMeLine;
    meline1->setMask(*mask);
    meline1->setMe(me);
    meline2 = new vpMbtMeLine;
    meline1->setMask(*mask);
    meline2->setMe(me);

    //    meline->setDisplay(vpMeSite::RANGE_RESULT);
    meline1->setInitRange(0);
    meline2->setInitRange(0);

    // Conversion meter to pixels
    vpMeterPixelConversion::convertLine(cam, c->getRho1(), c->getTheta1(), rho1, theta1);
    vpMeterPixelConversion::convertLine(cam, c->getRho2(), c->getTheta2(), rho2, theta2);

    // Determine intersections between circles and limbos
    double i11, i12, i21, i22, j11, j12, j21, j22;
    vpCircle::computeIntersectionPoint(*cercle1, cam, rho1, theta1, i11, j11);
    vpCircle::computeIntersectionPoint(*cercle2, cam, rho1, theta1, i12, j12);
    vpCircle::computeIntersectionPoint(*cercle1, cam, rho2, theta2, i21, j21);
    vpCircle::computeIntersectionPoint(*cercle2, cam, rho2, theta2, i22, j22);

    // Create the image points
    vpImagePoint ip11, ip12, ip21, ip22;
    ip11.set_ij(i11, j11);
    ip12.set_ij(i12, j12);
    ip21.set_ij(i21, j21);
    ip22.set_ij(i22, j22);

    // update limits of the melines.
    int marge = /*10*/ 5; // ou 5 normalement
    if (ip11.get_j() < ip12.get_j()) {
      meline1->jmin = (int)ip11.get_j() - marge;
      meline1->jmax = (int)ip12.get_j() + marge;
    } else {
      meline1->jmin = (int)ip12.get_j() - marge;
      meline1->jmax = (int)ip11.get_j() + marge;
    }
    if (ip11.get_i() < ip12.get_i()) {
      meline1->imin = (int)ip11.get_i() - marge;
      meline1->imax = (int)ip12.get_i() + marge;
    } else {
      meline1->imin = (int)ip12.get_i() - marge;
      meline1->imax = (int)ip11.get_i() + marge;
    }

    if (ip21.get_j() < ip22.get_j()) {
      meline2->jmin = (int)ip21.get_j() - marge;
      meline2->jmax = (int)ip22.get_j() + marge;
    } else {
      meline2->jmin = (int)ip22.get_j() - marge;
      meline2->jmax = (int)ip21.get_j() + marge;
    }
    if (ip21.get_i() < ip22.get_i()) {
      meline2->imin = (int)ip21.get_i() - marge;
      meline2->imax = (int)ip22.get_i() + marge;
    } else {
      meline2->imin = (int)ip22.get_i() - marge;
      meline2->imax = (int)ip21.get_i() + marge;
    }

    // Initialize the tracking
    while (theta1 > M_PI) {
      theta1 -= M_PI;
    }
    while (theta1 < -M_PI) {
      theta1 += M_PI;
    }

    if (theta1 < -M_PI / 2.0)
      theta1 = -theta1 - 3 * M_PI / 2.0;
    else
      theta1 = M_PI / 2.0 - theta1;

    while (theta2 > M_PI) {
      theta2 -= M_PI;
    }
    while (theta2 < -M_PI) {
      theta2 += M_PI;
    }

    if (theta2 < -M_PI / 2.0)
      theta2 = -theta2 - 3 * M_PI / 2.0;
    else
      theta2 = M_PI / 2.0 - theta2;

    try {
      meline1->initTracking(I, ip11, ip12, rho1, theta1, doNotTrack);
    } catch (...) {
      // vpTRACE("the line can't be initialized");
      return false;
    }
    try {
      meline2->initTracking(I, ip21, ip22, rho2, theta2, doNotTrack);
    } catch (...) {
      // vpTRACE("the line can't be initialized");
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
void vpMbtDistanceCylinder::trackMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix & /*cMo*/)
{
  if (isvisible) {
    try {
      meline1->track(I);
    } catch (...) {
      // std::cout << "Track meline1 failed" << std::endl;
      meline1->reset();
      Reinit = true;
    }
    try {
      meline2->track(I);
    } catch (...) {
      // std::cout << "Track meline2 failed" << std::endl;
      meline2->reset();
      Reinit = true;
    }

    // Update the number of features
    nbFeaturel1 = (unsigned int)meline1->getMeList().size();
    nbFeaturel2 = (unsigned int)meline2->getMeList().size();
    nbFeature = nbFeaturel1 + nbFeaturel2;
  }
}

/*!
  Update the moving edges internal parameters.

  \param I : the image.
  \param cMo : The pose of the camera.
*/
void vpMbtDistanceCylinder::updateMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo)
{
  if (isvisible) {
    // Perspective projection
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);
    cercle1->changeFrame(cMo);
    cercle2->changeFrame(cMo);
    c->changeFrame(cMo);

    p1->projection();
    p2->projection();
    try {
      cercle1->projection();
    } catch (...) {
      std::cout << "Probleme projection cercle 1\n";
    }
    try {
      cercle2->projection();
    } catch (...) {
      std::cout << "Probleme projection cercle 2\n";
    }
    c->projection();

    // Get the limbos
    double rho1, theta1;
    double rho2, theta2;

    // Conversion meter to pixels
    vpMeterPixelConversion::convertLine(cam, c->getRho1(), c->getTheta1(), rho1, theta1);
    vpMeterPixelConversion::convertLine(cam, c->getRho2(), c->getTheta2(), rho2, theta2);

    // Determine intersections between circles and limbos
    double i11, i12, i21, i22, j11, j12, j21, j22;

    vpCircle::computeIntersectionPoint(*cercle1, cam, rho1, theta1, i11, j11);
    vpCircle::computeIntersectionPoint(*cercle2, cam, rho1, theta1, i12, j12);

    vpCircle::computeIntersectionPoint(*cercle1, cam, rho2, theta2, i21, j21);
    vpCircle::computeIntersectionPoint(*cercle2, cam, rho2, theta2, i22, j22);

    // Create the image points
    vpImagePoint ip11, ip12, ip21, ip22;
    ip11.set_ij(i11, j11);
    ip12.set_ij(i12, j12);
    ip21.set_ij(i21, j21);
    ip22.set_ij(i22, j22);

    // update limits of the meline.
    int marge = /*10*/ 5; // ou 5 normalement
    if (ip11.get_j() < ip12.get_j()) {
      meline1->jmin = (int)ip11.get_j() - marge;
      meline1->jmax = (int)ip12.get_j() + marge;
    } else {
      meline1->jmin = (int)ip12.get_j() - marge;
      meline1->jmax = (int)ip11.get_j() + marge;
    }
    if (ip11.get_i() < ip12.get_i()) {
      meline1->imin = (int)ip11.get_i() - marge;
      meline1->imax = (int)ip12.get_i() + marge;
    } else {
      meline1->imin = (int)ip12.get_i() - marge;
      meline1->imax = (int)ip11.get_i() + marge;
    }

    if (ip21.get_j() < ip22.get_j()) {
      meline2->jmin = (int)ip21.get_j() - marge;
      meline2->jmax = (int)ip22.get_j() + marge;
    } else {
      meline2->jmin = (int)ip22.get_j() - marge;
      meline2->jmax = (int)ip21.get_j() + marge;
    }
    if (ip21.get_i() < ip22.get_i()) {
      meline2->imin = (int)ip21.get_i() - marge;
      meline2->imax = (int)ip22.get_i() + marge;
    } else {
      meline2->imin = (int)ip22.get_i() - marge;
      meline2->imax = (int)ip21.get_i() + marge;
    }

    // Initialize the tracking
    while (theta1 > M_PI) {
      theta1 -= M_PI;
    }
    while (theta1 < -M_PI) {
      theta1 += M_PI;
    }

    if (theta1 < -M_PI / 2.0)
      theta1 = -theta1 - 3 * M_PI / 2.0;
    else
      theta1 = M_PI / 2.0 - theta1;

    while (theta2 > M_PI) {
      theta2 -= M_PI;
    }
    while (theta2 < -M_PI) {
      theta2 += M_PI;
    }

    if (theta2 < -M_PI / 2.0)
      theta2 = -theta2 - 3 * M_PI / 2.0;
    else
      theta2 = M_PI / 2.0 - theta2;

    try {
      // meline1->updateParameters(I,rho1,theta1);
      meline1->updateParameters(I, ip11, ip12, rho1, theta1);
    } catch (...) {
      Reinit = true;
    }
    try {
      // meline2->updateParameters(I,rho2,theta2);
      meline2->updateParameters(I, ip21, ip22, rho2, theta2);
    } catch (...) {
      Reinit = true;
    }

    // Update the numbers of features
    nbFeaturel1 = (unsigned int)meline1->getMeList().size();
    nbFeaturel2 = (unsigned int)meline2->getMeList().size();
    nbFeature = nbFeaturel1 + nbFeaturel2;
  }
}

/*!
  Reinitialize the cylinder if it is required.

  A line is reinitialized if the 2D lines do not match enough with the
  projected 3D lines.

  \param I : the image.
  \param cMo : The pose of the camera.
  \param mask: Mask image or NULL if not wanted. Mask values that are set to true are considered in the tracking. To disable a pixel, set false.
*/
void vpMbtDistanceCylinder::reinitMovingEdge(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                             const vpImage<bool> *mask)
{
  if (meline1 != NULL)
    delete meline1;
  if (meline2 != NULL)
    delete meline2;

  meline1 = NULL;
  meline2 = NULL;

  if (!initMovingEdge(I, cMo, false, mask))
    Reinit = true;

  Reinit = false;
}

/*!
  Display the cylinder. The 3D cylinder is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : When true, display the circle even if non visible.
*/
void vpMbtDistanceCylinder::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo,
                                    const vpCameraParameters &camera, const vpColor &col, const unsigned int thickness,
                                    const bool displayFullModel)
{
  std::vector<std::vector<double> > models = getModelForDisplay(I.getWidth(), I.getHeight(),
                                                                cMo, camera, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);

    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

/*!
  Display the cylinder. The 3D cylinder is projected into the image.

  \param I : The image.
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param col : The desired color.
  \param thickness : The thickness of the lines.
  \param displayFullModel : When true, display the circle even if non visible.
*/
void vpMbtDistanceCylinder::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo,
                                    const vpCameraParameters &camera, const vpColor &col, const unsigned int thickness,
                                    const bool displayFullModel)
{
  std::vector<std::vector<double> > models = getModelForDisplay(I.getWidth(), I.getHeight(),
                                                                cMo, camera, displayFullModel);

  for (size_t i = 0; i < models.size(); i++) {
    vpImagePoint ip1(models[i][1], models[i][2]);
    vpImagePoint ip2(models[i][3], models[i][4]);

    vpDisplay::displayLine(I, ip1, ip2, col, thickness);
  }
}

/*!
  Return a list of features parameters for display.
  - Parameters are: `<feature id (here 0 for ME)>`, `<pt.i()>`, `<pt.j()>`, `<state>`
*/
std::vector<std::vector<double> > vpMbtDistanceCylinder::getFeaturesForDisplay()
{
  std::vector<std::vector<double> > features;

  if (meline1 != NULL) {
    for (std::list<vpMeSite>::const_iterator it = meline1->getMeList().begin(); it != meline1->getMeList().end(); ++it) {
      vpMeSite p_me = *it;
#ifdef VISP_HAVE_CXX11
      std::vector<double> params = {0, //ME
                                    p_me.get_ifloat(),
                                    p_me.get_jfloat(),
                                    static_cast<double>(p_me.getState())};
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

  if (meline2 != NULL) {
    for (std::list<vpMeSite>::const_iterator it = meline2->getMeList().begin(); it != meline2->getMeList().end(); ++it) {
      vpMeSite p_me = *it;
#ifdef VISP_HAVE_CXX11
      std::vector<double> params = {0, //ME
                                    p_me.get_ifloat(),
                                    p_me.get_jfloat(),
                                    static_cast<double>(p_me.getState())};
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
  Return a list of line parameters to display the primitive at a given pose and camera parameters.
  - Parameters are: `<primitive id (here 0 for line)>`, `<pt_start.i()>`, `<pt_start.j()>`,
  `<pt_end.i()>`, `<pt_end.j()>`

  \param width, height Image size (unused parameters).
  \param cMo : Pose used to project the 3D model into the image.
  \param camera : The camera parameters.
  \param displayFullModel : If true, the line is displayed even if it is not
*/
std::vector<std::vector<double> > vpMbtDistanceCylinder::getModelForDisplay(unsigned int, unsigned int,
                                                                            const vpHomogeneousMatrix &cMo,
                                                                            const vpCameraParameters &camera,
                                                                            const bool displayFullModel)
{
  std::vector<std::vector<double> > models;

  if ((isvisible && isTrackedCylinder) || displayFullModel) {
    // Perspective projection
    p1->changeFrame(cMo);
    p2->changeFrame(cMo);
    cercle1->changeFrame(cMo);
    cercle2->changeFrame(cMo);
    c->changeFrame(cMo);

    p1->projection();
    p2->projection();
    try {
      cercle1->projection();
    } catch (...) {
      std::cout << "Problem projection circle 1";
    }
    try {
      cercle2->projection();
    } catch (...) {
      std::cout << "Problem projection circle 2";
    }
    c->projection();

    double rho1, theta1;
    double rho2, theta2;

    // Meters to pixels conversion
    vpMeterPixelConversion::convertLine(camera, c->getRho1(), c->getTheta1(), rho1, theta1);
    vpMeterPixelConversion::convertLine(camera, c->getRho2(), c->getTheta2(), rho2, theta2);

    // Determine intersections between circles and limbos
    double i11, i12, i21, i22, j11, j12, j21, j22;

    vpCircle::computeIntersectionPoint(*cercle1, cam, rho1, theta1, i11, j11);
    vpCircle::computeIntersectionPoint(*cercle2, cam, rho1, theta1, i12, j12);

    vpCircle::computeIntersectionPoint(*cercle1, cam, rho2, theta2, i21, j21);
    vpCircle::computeIntersectionPoint(*cercle2, cam, rho2, theta2, i22, j22);

    // Create the image points
    vpImagePoint ip11, ip12, ip21, ip22;
    ip11.set_ij(i11, j11);
    ip12.set_ij(i12, j12);
    ip21.set_ij(i21, j21);
    ip22.set_ij(i22, j22);

#ifdef VISP_HAVE_CXX11
    std::vector<double> params1 = {0,
                                   ip11.get_i(),
                                   ip11.get_j(),
                                   ip12.get_i(),
                                   ip12.get_j()};

    std::vector<double> params2 = {0,
                                   ip21.get_i(),
                                   ip21.get_j(),
                                   ip22.get_i(),
                                   ip22.get_j()};
#else
    std::vector<double> params1, params2;
    params1.push_back(0);
    params1.push_back(ip11.get_i());
    params1.push_back(ip11.get_j());
    params1.push_back(ip12.get_i());
    params1.push_back(ip12.get_j());

    params2.push_back(0); 
    params2.push_back(ip11.get_i());
    params2.push_back(ip11.get_j());
    params2.push_back(ip12.get_i());
    params2.push_back(ip12.get_j());
#endif

    models.push_back(params1);
    models.push_back(params2);
  }

  return models;
}

/*!
    Enable to display the points along the lines with a color corresponding to
   their state.

    - If green : The vpMeSite is a good point.
    - If blue : The point is removed because of the vpMeSite tracking phase
   (constrast problem).
    - If purple : The point is removed because of the vpMeSite tracking phase
   (threshold problem).
    - If blue : The point is removed because of the robust method in the
   virtual visual servoing.

    \param I : The image.
*/
void vpMbtDistanceCylinder::displayMovingEdges(const vpImage<unsigned char> &I)
{
  if (meline1 != NULL) {
    meline1->display(I);
  }
  if (meline2 != NULL) {
    meline2->display(I);
  }
}

void vpMbtDistanceCylinder::displayMovingEdges(const vpImage<vpRGBa> &I)
{
  if (meline1 != NULL) {
    meline1->display(I);
  }
  if (meline2 != NULL) {
    meline2->display(I);
  }
}

/*!
  Initialize the size of the interaction matrix and the error vector.
*/
void vpMbtDistanceCylinder::initInteractionMatrixError()
{
  if (isvisible) {
    nbFeaturel1 = (unsigned int)meline1->getMeList().size();
    nbFeaturel2 = (unsigned int)meline2->getMeList().size();
    nbFeature = nbFeaturel1 + nbFeaturel2;
    L.resize(nbFeature, 6);
    error.resize(nbFeature);
  } else {
    nbFeature = 0;
    nbFeaturel1 = 0;
    nbFeaturel2 = 0;
  }
}

/*!
  Compute the interaction matrix and the error vector corresponding to the
  cylinder.
*/
void vpMbtDistanceCylinder::computeInteractionMatrixError(const vpHomogeneousMatrix &cMo,
                                                          const vpImage<unsigned char> &I)
{
  if (isvisible) {
    // Perspective projection
    c->changeFrame(cMo);
    c->projection();
    cercle1->changeFrame(cMo);
    cercle1->changeFrame(cMo);
    try {
      cercle1->projection();
    } catch (...) {
      std::cout << "Problem projection circle 1\n";
    }
    try {
      cercle2->projection();
    } catch (...) {
      std::cout << "Problem projection circle 2\n";
    }

    bool disp = false;
    bool disp2 = false;
    if (disp || disp2)
      vpDisplay::flush(I);

    // Build the lines
    vpFeatureBuilder::create(featureline2, *c, vpCylinder::line2);
    vpFeatureBuilder::create(featureline1, *c, vpCylinder::line1);

    double rho1 = featureline1.getRho();
    double theta1 = featureline1.getTheta();
    double rho2 = featureline2.getRho();
    double theta2 = featureline2.getTheta();

    double co1 = cos(theta1);
    double si1 = sin(theta1);
    double co2 = cos(theta2);
    double si2 = sin(theta2);

    double mx = 1.0 / cam.get_px();
    double my = 1.0 / cam.get_py();
    double xc = cam.get_u0();
    double yc = cam.get_v0();

    vpMatrix H1;
    H1 = featureline1.interaction();
    vpMatrix H2;
    H2 = featureline2.interaction();

    vpMeSite p;
    unsigned int j = 0;
    for (std::list<vpMeSite>::const_iterator it = meline1->getMeList().begin(); it != meline1->getMeList().end();
         ++it) {
      double x = (double)it->j;
      double y = (double)it->i;

      x = (x - xc) * mx;
      y = (y - yc) * my;

      double alpha1 = x * si1 - y * co1;

      double *Lrho = H1[0];
      double *Ltheta = H1[1];
      // Calculate interaction matrix for a distance
      for (unsigned int k = 0; k < 6; k++) {
        L[j][k] = (Lrho[k] + alpha1 * Ltheta[k]);
      }
      error[j] = rho1 - (x * co1 + y * si1);

      if (disp)
        vpDisplay::displayCross(I, it->i, it->j, (unsigned int)(error[j] * 100), vpColor::orange, 1);

      j++;
    }

    for (std::list<vpMeSite>::const_iterator it = meline2->getMeList().begin(); it != meline2->getMeList().end();
         ++it) {
      double x = (double)it->j;
      double y = (double)it->i;

      x = (x - xc) * mx;
      y = (y - yc) * my;

      double alpha2 = x * si2 - y * co2;

      double *Lrho = H2[0];
      double *Ltheta = H2[1];
      // Calculate interaction matrix for a distance
      for (unsigned int k = 0; k < 6; k++) {
        L[j][k] = (Lrho[k] + alpha2 * Ltheta[k]);
      }
      error[j] = rho2 - (x * co2 + y * si2);

      if (disp)
        vpDisplay::displayCross(I, it->i, it->j, (unsigned int)(error[j] * 100), vpColor::red, 1);

      j++;
    }
  }
}
