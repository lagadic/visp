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
 * Template tracker.
 *
 * Authors:
 * Amaury Dame
 * Aurelien Yol
 * Fabien Spindler
 *
 *****************************************************************************/
#include <visp3/tt/vpTemplateTrackerWarpRT.h>

vpTemplateTrackerWarpRT::vpTemplateTrackerWarpRT()
{
  nbParam = 3;
  dW.resize(2, nbParam);
}

// get the parameter corresponding to the lower level of a gaussian pyramid
void vpTemplateTrackerWarpRT::getParamPyramidDown(const vpColVector &p, vpColVector &pdown)
{
  pdown = p;
  pdown[1] = p[1] / 2.;
  pdown[2] = p[2] / 2.;
}

void vpTemplateTrackerWarpRT::getParamPyramidUp(const vpColVector &p, vpColVector &pup)
{
  pup = p;
  pup[1] = p[1] * 2.;
  pup[2] = p[2] * 2.;
}
/*calcul de di*dw(x,p0)/dp
 */
void vpTemplateTrackerWarpRT::getdW0(const int &i, const int &j, const double &dy, const double &dx, double *dIdW)
{
  //   std::cout << "getdW0" << std::endl;
  dIdW[0] = -i * dx + j * dy;
  dIdW[1] = dx;
  dIdW[2] = dy;
}
/*calcul de dw(x,p0)/dp
 */
void vpTemplateTrackerWarpRT::getdWdp0(const int &i, const int &j, double *dIdW)
{
  dIdW[0] = -i;
  dIdW[1] = 1.;
  dIdW[2] = 0;

  dIdW[3] = j;
  dIdW[4] = 0;
  dIdW[5] = 1.;
}

void vpTemplateTrackerWarpRT::warpX(const int &i, const int &j, double &i2, double &j2, const vpColVector &ParamM)
{
  j2 = (cos(ParamM[0]) * j) - (sin(ParamM[0]) * i) + ParamM[1];
  i2 = (sin(ParamM[0]) * j) + (cos(ParamM[0]) * i) + ParamM[2];
}

void vpTemplateTrackerWarpRT::warpX(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  vXres[0] = (cos(ParamM[0]) * vX[0]) - (sin(ParamM[0]) * vX[1]) + ParamM[1];
  vXres[1] = (sin(ParamM[0]) * vX[0]) + (cos(ParamM[0]) * vX[1]) + ParamM[2];
}

void vpTemplateTrackerWarpRT::dWarp(const vpColVector &X1, const vpColVector & /*X2*/, const vpColVector &ParamM,
                                    vpMatrix &dW_)
{
  double j = X1[0];
  double i = X1[1];
  dW_ = 0;
  dW_[0][0] = (-sin(ParamM[0]) * j) - (cos(ParamM[0]) * i);
  dW_[0][1] = 1;

  dW_[1][0] = cos(ParamM[0]) * j - sin(ParamM[0]) * i;
  dW_[1][2] = 1;
}

/*compute dw=dw/dx*dw/dp
 */
void vpTemplateTrackerWarpRT::dWarpCompo(const vpColVector & /*X1*/, const vpColVector & /*X2*/,
                                         const vpColVector &ParamM, const double *dwdp0, vpMatrix &dW_)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dW_[0][i] = (cos(ParamM[0]) * dwdp0[i]) - (sin(ParamM[0]) * dwdp0[i + nbParam]);
    dW_[1][i] = (sin(ParamM[0]) * dwdp0[i]) + (cos(ParamM[0]) * dwdp0[i + nbParam]);
  }
}

void vpTemplateTrackerWarpRT::warpXInv(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  //   std::cout << "warpXspe" << std::endl;
  vXres[0] = (cos(ParamM[0]) * vX[0]) - (sin(ParamM[0]) * vX[1]) + ParamM[1];
  vXres[1] = (sin(ParamM[0]) * vX[0]) + (cos(ParamM[0]) * vX[1]) + ParamM[2];
}

void vpTemplateTrackerWarpRT::getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const
{
  vpColVector Trans(2);
  vpMatrix MWrap(2, 2);
  Trans[0] = ParamM[1];
  Trans[1] = ParamM[2];
  MWrap[0][0] = cos(ParamM[0]);
  MWrap[0][1] = -sin(ParamM[0]);
  MWrap[1][0] = sin(ParamM[0]);
  MWrap[1][1] = cos(ParamM[0]);

  vpMatrix MWrapInv(2, 2);
  MWrapInv = MWrap.transpose();
  vpColVector TransInv(2);
  TransInv = (-1.0) * MWrapInv * Trans;

  ParamMinv[0] = atan2(MWrapInv[1][0], MWrapInv[1][1]);
  ParamMinv[1] = TransInv[0];
  ParamMinv[2] = TransInv[1];
}

void vpTemplateTrackerWarpRT::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const
{
  vpColVector Trans1(2);
  vpMatrix MWrap1(2, 2);
  Trans1[0] = p1[1];
  Trans1[1] = p1[2];

  MWrap1[0][0] = cos(p1[0]);
  MWrap1[0][1] = -sin(p1[0]);
  MWrap1[1][0] = sin(p1[0]);
  MWrap1[1][1] = cos(p1[0]);

  vpColVector Trans2(2);
  vpMatrix MWrap2(2, 2);
  Trans2[0] = p2[1];
  Trans2[1] = p2[1];

  MWrap2[0][0] = cos(p2[0]);
  MWrap2[0][1] = -sin(p2[0]);
  MWrap2[1][0] = sin(p2[0]);
  MWrap2[1][1] = cos(p2[0]);

  vpColVector TransRes(2);
  vpMatrix MWrapRes(2, 2);
  TransRes = MWrap1 * Trans2 + Trans1;
  MWrapRes = MWrap1 * MWrap2;

  pres[0] = atan2(MWrapRes[1][0], MWrapRes[1][1]);
  pres[1] = TransRes[0];
  pres[2] = TransRes[1];
}
