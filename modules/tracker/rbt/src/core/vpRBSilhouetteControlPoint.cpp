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
#define DEGENERATE_LINE_THRESHOLD 1e-5

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
  m_meMaskSign = 1;
  m_normal.resize(3);
  theta = 0;
  m_isSilhouette = false;
  m_valid = true;
}

vpRBSilhouetteControlPoint::vpRBSilhouetteControlPoint(const vpRBSilhouetteControlPoint &other)
{
  init();
  *this = other;
}

vpRBSilhouetteControlPoint &vpRBSilhouetteControlPoint::operator=(const vpRBSilhouetteControlPoint &other)
{
  m_me = other.m_me;
  m_site = other.m_site;
  m_numCandidates = other.m_numCandidates;
  m_cam = other.m_cam;
  icpoint = other.icpoint;
  cpoint = other.cpoint;
  cpointo = other.cpointo;
  m_normal = other.m_normal;
  m_normalO = other.m_normalO;
  xs = other.xs;
  ys = other.ys;
  nxs = other.nxs;
  nys = other.nys;
  Zs = other.Zs;
  m_isSilhouette = other.m_isSilhouette;
  rho = other.rho;
  theta = other.theta;
  thetaInit = other.thetaInit;
  m_meMaskSign = other.m_meMaskSign;
  m_lineFeature = other.m_lineFeature;
  m_line = other.m_line;
  m_valid = other.m_valid;
  return *this;
}

vpRBSilhouetteControlPoint::vpRBSilhouetteControlPoint(const vpRBSilhouetteControlPoint &&other)
{
  *this = other;
}

vpRBSilhouetteControlPoint &vpRBSilhouetteControlPoint::operator=(const vpRBSilhouetteControlPoint &&other)
{
  m_me = std::move(other.m_me);
  m_site = std::move(other.m_site);
  thetaInit = std::move(other.thetaInit);
  m_numCandidates = std::move(other.m_numCandidates);
  m_cam = std::move(other.m_cam);
  icpoint = std::move(other.icpoint);
  cpoint = std::move(other.cpoint);
  cpointo = std::move(other.cpointo);
  m_normal = std::move(other.m_normal);
  m_normalO = std::move(other.m_normalO);
  xs = std::move(other.xs);
  ys = std::move(other.ys);
  nxs = std::move(other.nxs);
  nys = std::move(other.nys);
  Zs = std::move(other.Zs);
  m_isSilhouette = std::move(other.m_isSilhouette);
  rho = std::move(other.rho);
  theta = std::move(other.theta);
  m_meMaskSign = std::move(other.m_meMaskSign);
  m_lineFeature = std::move(other.m_lineFeature);
  m_line = std::move(other.m_line);
  m_valid = std::move(other.m_valid);
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

  if (m_site.getState() == vpMeSite::NO_SUPPRESSION) {
    try {
      if (m_site.m_convlt == 0) {
        m_site.track(I, m_me, false);
      }
      else {
        m_site.track(I, m_me, false);
      }
    }
    catch (vpTrackingException &) {
      vpERROR_TRACE("caught a tracking exception, ignoring me point...");
      m_site.setState(vpMeSite::THRESHOLD);
    }
  }
}

void vpRBSilhouetteControlPoint::trackMultipleHypotheses(const vpImage<unsigned char> &I)
{
  // If element hasn't been suppressed
  try {
    if (m_site.getState() == vpMeSite::NO_SUPPRESSION) {
      //const bool testContrast = s.m_convlt != 0.0;
      m_site.trackMultipleHypotheses(I, *m_me, false, m_candidates, m_numCandidates);
      m_site = m_candidates[0];
    }
  }
  catch (vpTrackingException &) {
    vpERROR_TRACE("caught a tracking exception, ignoring me point...");
    m_site.setState(vpMeSite::THRESHOLD);
  }
}

/*!
  Build a 3D plane thanks the 3D coordinate of the control point and the normal vector to the surface.

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
  buildPlane(cpoint, m_normal, plane);
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
  vpColVector norm2 = vpColVector::cross(Vo, m_normalO);
  buildPlane(cpointo, norm2, plane2);
  buildPlane(cpointo, m_normalO, plane1);

  m_line.setWorldCoordinates(plane1.getA(), plane1.getB(), plane1.getC(), plane1.getD(),
                           plane2.getA(), plane2.getB(), plane2.getC(), plane2.getD());
}

void
vpRBSilhouetteControlPoint::buildPoint(int n, int m, const double &Z, double orient, const vpColVector &normo,
                                       const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc,
                                       const vpCameraParameters &cam, const vpMe &me, bool isSilhouette)
{
  m_cam = &cam;
  m_me = &me;
  m_isSilhouette = isSilhouette;
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
  m_normalO = normo;
  m_normal = R*normo;
  nxs = cos(theta);
  nys = sin(theta);
  buildPLine(oMc);
  m_valid = !isLineDegenerate();
}

void
vpRBSilhouetteControlPoint::buildSilhouettePoint(int n, int m, const double &Z, double orient, const vpColVector &normo,
                                                 const vpHomogeneousMatrix &cMo, const vpHomogeneousMatrix &oMc,
                                                 const vpCameraParameters &cam)
{
  m_isSilhouette = true;
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
  m_normalO = normo;
  m_normal = R * normo;
  buildPLine(oMc);
#if VISP_DEBUG_RB_CONTROL_POINT
  if (std::isnan(m_line.getTheta())) {
    std::cerr << "Line in camera frame = " << m_line.cP << std::endl;
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
      m_line.changeFrame(cMo);
      m_line.projection();
    }
    catch (vpException &e) {
      m_valid = false;
    }
    m_valid = !isLineDegenerate() && !std::isnan(m_line.getTheta());
    if (m_valid) {
      vpFeatureBuilder::create(m_lineFeature, m_line);
      theta = m_lineFeature.getTheta();
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
  m_site.init((double)icpoint.get_i(), (double)icpoint.get_j(), theta, cvlt, m_meMaskSign);
  if (m_me != nullptr) {
    const double marginRatio = m_me->getThresholdMarginRatio();
    const double convolution = m_site.convolution(I, m_me);
    m_site.init((double)icpoint.get_i(), (double)icpoint.get_j(), theta, convolution, m_meMaskSign);
    const double contrastThreshold = fabs(convolution) * marginRatio;
    m_site.setContrastThreshold(contrastThreshold, *m_me);
  }
}


/*!
  Compute the interaction matrix and the error vector corresponding to the line.
*/
void
vpRBSilhouetteControlPoint::computeMeInteractionMatrixError(const vpHomogeneousMatrix &cMo, unsigned int i, vpMatrix &L,
                                                            vpColVector &e)
{
  m_line.changeFrame(cMo);

  m_valid = false;
  if (!isLineDegenerate()) {
    m_line.projection();
    vpFeatureBuilder::create(m_lineFeature, m_line);

    double rho0 = m_lineFeature.getRho();
    double theta0 = m_lineFeature.getTheta();
#if VISP_DEBUG_RB_CONTROL_POINT
    if (std::isnan(theta0)) {
      std::cerr << "Line in camera frame = " << m_line.cP.t() << std::endl;
      std::cerr << "Line in object frame = " << m_line.oP.t() << std::endl;
      m_lineFeature.print();
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
    H = m_lineFeature.interaction();
    double x = (double)m_site.m_j, y = (double)m_site.m_i;

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
vpRBSilhouetteControlPoint::computeMeInteractionMatrixErrorMH(const vpHomogeneousMatrix &cMo, unsigned int i,
                                                              vpMatrix &L, vpColVector &e)
{
  m_line.changeFrame(cMo);

  m_valid = false;
  if (!isLineDegenerate()) {
    m_line.projection();

    vpFeatureBuilder::create(m_lineFeature, m_line);
    const double rho0 = m_lineFeature.getRho();
    const double theta0 = m_lineFeature.getTheta();
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
    const vpMatrix &H = m_lineFeature.interaction();
    double xmin, ymin;
    double errormin = std::numeric_limits<double>::max();

    const std::vector<vpMeSite> &cs = m_candidates;
    xmin = (m_site.m_j - xc) * mx;
    ymin = (m_site.m_i - yc) * my;
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


double vpRBSilhouetteControlPoint::getMaxMaskGradientAlongLine(const vpImage<float> &mask, int searchSize) const
{
  std::vector<float> maskValues(searchSize * 2 + 1);
  double c = cos(theta);
  double s = sin(theta);
  int index = 0;
  for (int n = -searchSize + 1; n < searchSize; ++n) {
    unsigned int ii = static_cast<unsigned int>(round(icpoint.get_i() + s * n));
    unsigned int jj = static_cast<unsigned int>(round(icpoint.get_j() + c * n));

    maskValues[index] = mask[ii][jj];
    ++index;
  }

  double maxGrad = 0.0;

  for (unsigned i = 1; i < maskValues.size() - 1; ++i) {
    double grad = abs(maskValues[i + 1] - maskValues[i - 1]);
    if (grad > maxGrad) {
      maxGrad = grad;
    }
  }
  return maxGrad;
}

bool vpRBSilhouetteControlPoint::tooCloseToBorder(unsigned int h, unsigned int w, int searchSize) const
{
  double cs = cos(theta) * static_cast<double>(searchSize);
  double ss = sin(theta) * static_cast<double>(searchSize);

  std::array<std::pair<int, int>, 2> extremities = {
      std::make_pair(static_cast<int>(round(icpoint.get_i() + ss)), static_cast<int>(round(icpoint.get_j() + cs))),
      std::make_pair(static_cast<int>(round(icpoint.get_i() - ss)), static_cast<int>(round(icpoint.get_j() - cs)))
  };

  int width = static_cast<int>(w);
  int height = static_cast<int>(h);
  for (unsigned int e = 0; e < 2; ++e) {
    int i = extremities[e].first;
    int j = extremities[e].second;
    if (i < 0 || i >= height || j < 0 || j >= width) {
      return true;
    }
  }

  return false;
}

bool vpRBSilhouetteControlPoint::isLineDegenerate() const
{
  double a, b, d;
  a = m_line.cP[4] * m_line.cP[3] - m_line.cP[0] * m_line.cP[7];
  b = m_line.cP[5] * m_line.cP[3] - m_line.cP[1] * m_line.cP[7];
  d = a*a + b*b;
  return d <= DEGENERATE_LINE_THRESHOLD;
}

END_VISP_NAMESPACE
