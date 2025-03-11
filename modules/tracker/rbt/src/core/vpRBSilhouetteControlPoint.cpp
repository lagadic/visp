/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 */

#include <visp3/rbt/vpRBSilhouetteControlPoint.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/visual_features/vpFeatureBuilder.h>
#include <visp3/core/vpRobust.h>

#define VISP_DEBUG_RB_CONTROL_POINT 1

BEGIN_VISP_NAMESPACE

void vpRBSilhouetteControlPoint::init()
{
  m_valid = false;
}

vpRBSilhouetteControlPoint::vpRBSilhouetteControlPoint()
{
  init();
  m_me = nullptr;
  m_numCandidates = 1;
  m_candidates.resize(1);
  sign = 1;
  norm.resize(3);
  theta = 0;
  m_isSilhouette = false;
  m_valid = true;
}

vpRBSilhouetteControlPoint::vpRBSilhouetteControlPoint(const vpRBSilhouetteControlPoint &meTracker)
{
  init();
  *this = meTracker;
}

vpRBSilhouetteControlPoint &vpRBSilhouetteControlPoint::operator=(const vpRBSilhouetteControlPoint &meTracker)
{
  m_me = meTracker.m_me;
  s = meTracker.s;
  m_numCandidates = meTracker.m_numCandidates;
  m_cam = meTracker.m_cam;
  icpoint = meTracker.icpoint;
  cpoint = meTracker.cpoint;
  cpointo = meTracker.cpointo;
  norm = meTracker.norm;
  normw = meTracker.normw;
  xs = meTracker.xs;
  ys = meTracker.ys;
  nxs = meTracker.nxs;
  nys = meTracker.nys;
  Zs = meTracker.Zs;
  m_isSilhouette = meTracker.m_isSilhouette;
  rho = meTracker.rho;
  theta = meTracker.theta;
  thetaInit = meTracker.thetaInit;
  delta = meTracker.delta;
  sign = meTracker.sign;
  featureline = meTracker.featureline;
  line = meTracker.line;
  m_valid = meTracker.m_valid;
  return *this;
}

vpRBSilhouetteControlPoint::vpRBSilhouetteControlPoint(const vpRBSilhouetteControlPoint &&meTracker)
{
  *this = meTracker;
}

vpRBSilhouetteControlPoint &vpRBSilhouetteControlPoint::operator=(const vpRBSilhouetteControlPoint &&meTracker)
{
  m_me = std::move(meTracker.m_me);
  s = std::move(meTracker.s);
  thetaInit = std::move(meTracker.thetaInit);
  m_numCandidates = std::move(meTracker.m_numCandidates);
  m_cam = std::move(meTracker.m_cam);
  icpoint = std::move(meTracker.icpoint);
  cpoint = std::move(meTracker.cpoint);
  cpointo = std::move(meTracker.cpointo);
  norm = std::move(meTracker.norm);
  normw = std::move(meTracker.normw);
  xs = std::move(meTracker.xs);
  ys = std::move(meTracker.ys);
  nxs = std::move(meTracker.nxs);
  nys = std::move(meTracker.nys);
  Zs = std::move(meTracker.Zs);
  m_isSilhouette = std::move(meTracker.m_isSilhouette);
  rho = std::move(meTracker.rho);
  theta = std::move(meTracker.theta);
  delta = std::move(meTracker.delta);
  sign = std::move(meTracker.sign);
  featureline = std::move(meTracker.featureline);
  line = std::move(meTracker.line);
  m_valid = std::move(meTracker.m_valid);
  return *this;
}

int vpRBSilhouetteControlPoint::outOfImage(int i, int j, int half, int rows, int cols) const
{
  return (!((i> half+2) &&
            (i< rows -(half+2)) &&
            (j>half+2) &&
            (j<cols-(half+2)))
          );
}

int vpRBSilhouetteControlPoint::outOfImage(const vpImagePoint &iP, int half, int rows, int cols) const
{
  int i = vpMath::round(iP.get_i());
  int j = vpMath::round(iP.get_j());
  return (!((i> half+2) &&
            (i< rows -(half+2)) &&
            (j>half+2) &&
            (j<cols-(half+2)))
          );
}

void vpRBSilhouetteControlPoint::track(const vpImage<unsigned char> &I)
{

  if (s.getState() == vpMeSite::NO_SUPPRESSION) {
    try {
      if (s.m_convlt == 0) {
        s.track(I, m_me, false);
      }
      else {
        s.track(I, m_me, false);
      }
    }
    catch (vpTrackingException &) {
      vpERROR_TRACE("caught a tracking exception, ignoring me point...");
      s.setState(vpMeSite::THRESHOLD);
    }
  }
}

void vpRBSilhouetteControlPoint::trackMultipleHypotheses(const vpImage<unsigned char> &I)
{
  // If element hasn't been suppressed
  try {
    if (s.getState() == vpMeSite::NO_SUPPRESSION) {
      //const bool testContrast = s.m_convlt != 0.0;
      s.trackMultipleHypotheses(I, *m_me, false, m_candidates, m_numCandidates);
    }
  }
  catch (vpTrackingException &) {
    vpERROR_TRACE("caught a tracking exception, ignoring me point...");
    s.setState(vpMeSite::THRESHOLD);
  }
}

/*!
  Build a 3D plane thanks the 3D coordinate of the control point and the normal vector to the surface

  \param[in] pointn : A point on the plane with coordinates in the object frame (oX, oY, oZ).
  \param[in] normal : Normal of the plane.
  \param[out] plane : The vpPlane instance used to store the computed plane equation.
*/
void
vpRBSilhouetteControlPoint::buildPlane(const vpPoint &pointn, const vpColVector &normal, vpPlane &plane)
{
  plane.init(pointn, normal, vpPlane::object_frame);
}

void
vpRBSilhouetteControlPoint::buildPLine(const vpHomogeneousMatrix &oMc)
{
  vpPlane plane;
  vpPlane plane1;
  vpPlane plane2;
  buildPlane(cpoint, norm, plane);
  vpRotationMatrix R;
  oMc.extract(R);

  vpColVector V(3);
  vpColVector Vo(3);
  if (abs(theta) > 1e-2) {
    V[0] = ((cpoint.get_oX()/cpoint.get_oZ())+1)*cpoint.get_oZ()-cpoint.get_oX();
    V[1] = ((cpoint.get_oY()/cpoint.get_oZ())-cos(theta)/sin(theta))*cpoint.get_oZ()-cpoint.get_oY();
    V[2] = (-plane.getD()-V[0]*plane.getA()-V[1]*plane.getB())/plane.getC()-cpoint.get_oZ();
  }
  else {
    V[0] = ((cpoint.get_oX()/cpoint.get_oZ())+1)*cpoint.get_oZ()-cpoint.get_oX();
    V[1] = ((cpoint.get_oY()/cpoint.get_oZ()))*cpoint.get_oZ()-cpoint.get_oY();
    V[2] = (-plane.getD()-V[0]*plane.getA()-V[1]*plane.getB())/plane.getC()-cpoint.get_oZ();
  }

  Vo = R*V;
  vpColVector norm2 = vpColVector::cross(Vo, normw);
  buildPlane(cpointo, norm2, plane2);
  buildPlane(cpointo, normw, plane1);

  line.setWorldCoordinates(plane1.getA(), plane1.getB(), plane1.getC(), plane1.getD(),
                           plane2.getA(), plane2.getB(), plane2.getC(), plane2.getD());
}

void
vpRBSilhouetteControlPoint::buildPoint(int n, int m, const double &Z, double orient, const vpColVector &normo,
                                       const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc, const vpCameraParameters &cam, const vpMe &me)
{
  m_cam = &cam;
  m_me = &me;

  vpRotationMatrix R;
  cMo.extract(R);
  theta = orient;
  thetaInit = theta;
  double px = m_cam->get_px();
  double py = m_cam->get_py();
  int jc = m_cam->get_u0();
  int ic = m_cam->get_v0();
  icpoint.set_i(n);
  icpoint.set_j(m);
  double x, y;
  x = (m-jc)/px;
  y = (n-ic)/py;
  rho = x*cos(theta)+y*sin(theta);
  cpoint.setWorldCoordinates(x*Z, y*Z, Z);
  cpoint.changeFrame(oMc);
  cpointo.setWorldCoordinates(cpoint.get_X(), cpoint.get_Y(), cpoint.get_Z());
  normw = normo;
  norm = R*normo;
  nxs = cos(theta);
  nys = sin(theta);
  buildPLine(oMc);
  m_valid = isLineDegenerate();
}

void
vpRBSilhouetteControlPoint::buildSilhouettePoint(int n, int m, const double &Z, double orient, const vpColVector &normo,
                                              const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc, const vpCameraParameters &cam)
{
  m_cam = &cam;
  vpRotationMatrix R;
  cMo.extract(R);
  theta = orient;
  thetaInit = theta;
  double px = m_cam->get_px();
  double py = m_cam->get_py();
  int jc = m_cam->get_u0();
  int ic = m_cam->get_v0();
  icpoint.set_i(n);
  icpoint.set_j(m);
  xs = (m-jc)/px;
  ys = (n-ic)/py;
  Zs = Z;

  nxs = cos(theta);
  nys = sin(theta);
  double x, y;
  x = (m-jc)/px;
  y = (n-ic)/py;
  cpoint.setWorldCoordinates(x * Z, y * Z, Z);
  cpoint.changeFrame(oMc);
  cpointo.setWorldCoordinates(cpoint.get_X(), cpoint.get_Y(), cpoint.get_Z());
  normw = normo;
  norm = R * normo;
  buildPLine(oMc);
#if VISP_DEBUG_RB_CONTROL_POINT
  if (std::isnan(line.getTheta())) {
    std::cerr << "Line in camera frame = " << line.cP << std::endl;
    throw vpException(vpException::fatalError, "Incorrect line definition");

  }
#endif
  m_valid = isLineDegenerate();
}

void
vpRBSilhouetteControlPoint::update(const vpHomogeneousMatrix &_cMo)
{
  cpointo.changeFrame(_cMo);
  cpointo.projection();
  double px = m_cam->get_px();
  double py = m_cam->get_py();
  double uc = m_cam->get_u0();
  double vc = m_cam->get_v0();
  double u, v;
  v = py*cpointo.get_y()+vc;
  u = px*cpointo.get_x()+uc;
  icpoint.set_uv(u, v);
}

void
vpRBSilhouetteControlPoint::updateSilhouettePoint(const vpHomogeneousMatrix &cMo)
{
  cpointo.changeFrame(cMo);
  cpointo.projection();
  const double px = m_cam->get_px();
  const double py = m_cam->get_py();
  const double uc = m_cam->get_u0();
  const double vc = m_cam->get_v0();
  const double v = py * cpointo.get_y() + vc;
  const double u = px * cpointo.get_x() + uc;
  icpoint.set_uv(u, v);
  xs = cpointo.get_x();
  ys = cpointo.get_y();
  Zs = cpointo.get_Z();
  if (m_valid) {
    try {
      line.changeFrame(cMo);
      line.projection();
    }
    catch (vpException &e) {
      m_valid = false;
    }
    m_valid = !isLineDegenerate() && !std::isnan(line.getTheta());
    if (m_valid) {
      vpFeatureBuilder::create(featureline, line);
      theta = featureline.getTheta();
#if VISP_DEBUG_RB_CONTROL_POINT
      if (std::isnan(theta)) {
        throw vpException(vpException::fatalError, "Got nan theta in updateSilhouettePoint");
      }
#endif
      if (fabs(theta - thetaInit) < M_PI / 2.0) {
        nxs = cos(theta);
        nys = sin(theta);
      }
      else {
        nxs = -cos(theta);
        nys = -sin(theta);
      }
    }
  }
}

void vpRBSilhouetteControlPoint::initControlPoint(const vpImage<unsigned char> &I, double cvlt)
{
  double delta = theta;
  s.init((double)icpoint.get_i(), (double)icpoint.get_j(), delta, cvlt, sign);
  if (m_me != nullptr) {
    const double marginRatio = m_me->getThresholdMarginRatio();
    const double convolution = s.convolution(I, m_me);
    s.init((double)icpoint.get_i(), (double)icpoint.get_j(), delta, convolution, sign);
    const double contrastThreshold = fabs(convolution) * marginRatio;
    s.setContrastThreshold(contrastThreshold, *m_me);
  }
}

void vpRBSilhouetteControlPoint::detectSilhouette(const vpImage<float> &I)
{
  unsigned int k = 0;
  int range = 4;
  double c = cos(theta);
  double s = sin(theta);
  for (int n = -range; n <= range; n++) {
    unsigned int ii = static_cast<unsigned int>(round(icpoint.get_i() + s * n));
    unsigned int jj = static_cast<unsigned int>(round(icpoint.get_j() + c * n));
    unsigned int isBg = static_cast<unsigned int>(I[ii][jj] == 0.f);
    k += isBg;
  }
  m_isSilhouette = k > 2;
}

/*!
  Compute the interaction matrix and the error vector corresponding to the line.
*/
void
vpRBSilhouetteControlPoint::computeMeInteractionMatrixError(const vpHomogeneousMatrix &cMo, unsigned int i, vpMatrix &L, vpColVector &e)
{
  line.changeFrame(cMo);

  m_valid = false;
  if (!isLineDegenerate()) {
    line.projection();
    vpFeatureBuilder::create(featureline, line);

    double rho0 = featureline.getRho();
    double theta0 = featureline.getTheta();
#if VISP_DEBUG_RB_CONTROL_POINT
    if (std::isnan(theta0)) {
      std::cerr << "Line in camera frame = " << line.cP.t() << std::endl;
      std::cerr << "Line in object frame = " << line.oP.t() << std::endl;
      featureline.print();
      throw vpException(vpException::fatalError, "Got nan theta in computeInteractionMatrixError");
    }
#endif
    double co = cos(theta0);
    double si = sin(theta0);

    double mx = 1.0 / m_cam->get_px();
    double my = 1.0 / m_cam->get_py();
    double xc = m_cam->get_u0();
    double yc = m_cam->get_v0();

    vpMatrix H;
    H = featureline.interaction();
    double x = (double)s.m_j, y = (double)s.m_i;

    x = (x-xc)*mx;
    y = (y-yc)*my;

    const double alpha = x*si - y*co;

    double *Lrho = H[0];
    double *Ltheta = H[1];
    // Calculate interaction matrix for a distance
    for (unsigned int k = 0; k < 6; k++) {
      L[i][k] = (Lrho[k] + alpha*Ltheta[k]);
    }
    e[i] = rho0 - (x*co + y*si);
    m_valid = true;
  }
  else {
    m_valid = false;
    e[i] = 0;
    for (unsigned int k = 0; k < 6; k++) {
      L[i][k] = 0.0;
    }
  }
}

void
vpRBSilhouetteControlPoint::computeMeInteractionMatrixErrorMH(const vpHomogeneousMatrix &cMo, unsigned int i, vpMatrix &L, vpColVector &e)
{
  line.changeFrame(cMo);

  m_valid = false;
  if (!isLineDegenerate()) {
    line.projection();

    vpFeatureBuilder::create(featureline, line);
    const double rho0 = featureline.getRho();
    const double theta0 = featureline.getTheta();
#if VISP_DEBUG_RB_CONTROL_POINT
    if (std::isnan(theta0)) {
      throw vpException(vpException::fatalError, "Got nan theta in computeInteractionMatrixMH");
    }
#endif

    const double co = cos(theta0);
    const double si = sin(theta0);

    const double mx = 1.0 / m_cam->get_px();
    const double my = 1.0 / m_cam->get_py();
    const double xc = m_cam->get_u0();
    const double yc = m_cam->get_v0();
    const vpMatrix &H = featureline.interaction();
    double xmin, ymin;
    double errormin = std::numeric_limits<double>::max();

    const std::vector<vpMeSite> &cs = m_candidates;
    xmin = (s.m_j - xc) * mx;
    ymin = (s.m_i - yc) * my;
    for (unsigned int l = 0; l < m_numCandidates; l++) //for each candidate of P
    {
      const vpMeSite &Pk = cs[l];

      if ((Pk.getState() == vpMeSite::NO_SUPPRESSION)) {
        const double x = (Pk.m_j - xc) * mx;
        const double y = (Pk.m_i - yc) * my;
        const double err = fabs(rho0 - (x * co + y * si));
        if (err <= errormin) {
          errormin = err;
          xmin = x;
          ymin = y;
          m_valid = true;
        }
      }
    }
    if (m_valid) {
      e[i] = rho0 - (xmin * co + ymin * si);
      const double alpha = xmin * si - ymin * co;

      const double *Lrho = H[0];
      const double *Ltheta = H[1];
      // Calculate interaction matrix for a distance
      for (unsigned int k = 0; k < 6; k++) {
        L[i][k] = (Lrho[k] + alpha * Ltheta[k]);
      }
    }
    else {
      e[i] = 0;
      for (unsigned int k = 0; k < 6; k++) {
        L[i][k] = 0.0;
      }
    }
  }
}

bool vpRBSilhouetteControlPoint::isLineDegenerate() const
{
  double a, b, d;
  a = line.cP[4] * line.cP[3] - line.cP[0] * line.cP[7];
  b = line.cP[5] * line.cP[3] - line.cP[1] * line.cP[7];
  d = a*a + b*b;
  return d <= 1e-7;
}

END_VISP_NAMESPACE
