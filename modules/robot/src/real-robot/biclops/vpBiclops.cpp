/*
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
 * Interface for the Biclops robot.
 */

#include <math.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>
#include <visp3/robot/vpBiclops.h>
#include <visp3/robot/vpRobotException.h>

BEGIN_VISP_NAMESPACE
const unsigned int vpBiclops::ndof = 2;
const float vpBiclops::h = 0.048f;
const float vpBiclops::panJointLimit = (float)(M_PI);
const float vpBiclops::tiltJointLimit = (float)(M_PI / 4.5);
const float vpBiclops::speedLimit = (float)(M_PI / 3.0);

void vpBiclops::computeMGD(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{
  vpHomogeneousMatrix fMe = get_fMe(q);
  fMc = fMe * m_cMe.inverse();

  vpCDEBUG(6) << "camera position: " << std::endl << fMc;

  return;
}

void vpBiclops::get_fMc(const vpColVector &q, vpHomogeneousMatrix &fMc) const
{
  vpHomogeneousMatrix fMe = get_fMe(q);
  fMc = fMe * m_cMe.inverse();

  vpCDEBUG(6) << "camera position: " << std::endl << fMc;

  return;
}

vpHomogeneousMatrix vpBiclops::computeMGD(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;

  computeMGD(q, fMc);

  return fMc;
}

vpHomogeneousMatrix vpBiclops::get_fMc(const vpColVector &q) const
{
  vpHomogeneousMatrix fMc;

  get_fMc(q, fMc);

  return fMc;
}

vpHomogeneousMatrix vpBiclops::get_fMe(const vpColVector &q) const
{
  vpHomogeneousMatrix fMe;

  if (q.getRows() != 2) {
    throw(vpException(vpException::dimensionError, "Bad dimension for Biclops joint position vector"));
  }

  double q1 = q[0]; // pan
  double q2 = q[1]; // tilt

  double c1 = cos(q1);
  double s1 = sin(q1);
  double c2 = cos(q2);
  double s2 = sin(q2);

  if (m_dh_model == DH1) {
    fMe[0][0] = -c1 * s2;
    fMe[0][1] = -s1;
    fMe[0][2] = c1 * c2;
    fMe[0][3] = 0;

    fMe[1][0] = -s1 * s2;
    fMe[1][1] = c1;
    fMe[1][2] = s1 * c2;
    fMe[1][3] = 0;

    fMe[2][0] = -c2;
    fMe[2][1] = 0;
    fMe[2][2] = -s2;
    fMe[2][3] = 0;

    fMe[3][0] = 0;
    fMe[3][1] = 0;
    fMe[3][2] = 0;
    fMe[3][3] = 1;
  }
  else {
    fMe[0][0] = c1 * s2;
    fMe[0][1] = -s1;
    fMe[0][2] = c1 * c2;
    fMe[0][3] = 0;

    fMe[1][0] = s1 * s2;
    fMe[1][1] = c1;
    fMe[1][2] = s1 * c2;
    fMe[1][3] = 0;

    fMe[2][0] = -c2;
    fMe[2][1] = 0;
    fMe[2][2] = s2;
    fMe[2][3] = 0;

    fMe[3][0] = 0;
    fMe[3][1] = 0;
    fMe[3][2] = 0;
    fMe[3][3] = 1;
  }

  return fMe;
}

void vpBiclops::computeMGD(const vpColVector &q, vpPoseVector &fPc) const
{
  vpHomogeneousMatrix fMc;

  get_fMc(q, fMc);
  fPc.build(fMc.inverse());

  return;
}

void vpBiclops::get_fMc(const vpColVector &q, vpPoseVector &fPc) const
{
  vpHomogeneousMatrix fMc;

  get_fMc(q, fMc);
  fPc.build(fMc.inverse());

  return;
}

vpBiclops::vpBiclops(void) : m_dh_model(DH1), m_cMe() { init(); }

void vpBiclops::init()
{
  m_dh_model = DH1;
  set_cMe();
  return;
}

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpBiclops & /*constant*/)
{
  os << "Geometric parameters: " << std::endl
    << "h: "
    << "\t" << vpBiclops::h << std::endl;

  return os;
}

void vpBiclops::get_cVe(vpVelocityTwistMatrix &cVe) const { cVe.build(m_cMe); }

void vpBiclops::set_cMe()
{
  vpHomogeneousMatrix cMe;

  m_cMe[0][0] = 0;
  m_cMe[0][1] = 1;
  m_cMe[0][2] = 0;
  m_cMe[0][3] = 0;

  m_cMe[1][0] = -1;
  m_cMe[1][1] = 0;
  m_cMe[1][2] = 0;
  m_cMe[1][3] = h;

  m_cMe[2][0] = 0;
  m_cMe[2][1] = 0;
  m_cMe[2][2] = 1;
  m_cMe[2][3] = 0;

  m_cMe[3][0] = 0;
  m_cMe[3][1] = 0;
  m_cMe[3][2] = 0;
  m_cMe[3][3] = 1;
}

void vpBiclops::get_eJe(const vpColVector &q, vpMatrix &eJe) const
{
  eJe.resize(6, 2);

  if (q.getRows() != 2) {
    throw(vpException(vpException::dimensionError, "Bad dimension for Biclops joint position vector"));
  }

  double s2 = sin(q[1]);
  double c2 = cos(q[1]);

  eJe = 0;

  if (m_dh_model == DH1) {
    eJe[3][0] = -c2;
    eJe[4][1] = 1;
    eJe[5][0] = -s2;
  }
  else {
    eJe[3][0] = -c2;
    eJe[4][1] = -1;
    eJe[5][0] = s2;
  }
}

void vpBiclops::get_fJe(const vpColVector &q, vpMatrix &fJe) const
{
  if (q.getRows() != 2) {
    throw(vpException(vpException::dimensionError, "Bad dimension for Biclops joint position vector"));
  }

  fJe.resize(6, 2);

  double s1 = sin(q[0]);
  double c1 = cos(q[0]);

  fJe = 0;

  if (m_dh_model == DH1) {
    fJe[3][1] = -s1;
    fJe[4][1] = c1;
    fJe[5][0] = 1;
  }
  else {
    fJe[3][1] = s1;
    fJe[4][1] = -c1;
    fJe[5][0] = 1;
  }
}
END_VISP_NAMESPACE
