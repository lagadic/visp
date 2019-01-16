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
#include <visp3/tt/vpTemplateTrackerWarpSRT.h>

vpTemplateTrackerWarpSRT::vpTemplateTrackerWarpSRT()
{
  nbParam = 4;
  dW.resize(2, nbParam);
}

// get the parameter corresponding to the lower level of a gaussian pyramid
void vpTemplateTrackerWarpSRT::getParamPyramidDown(const vpColVector &p, vpColVector &pdown)
{
  pdown = p;
  pdown[2] = p[2] / 2.;
  pdown[3] = p[3] / 2.;
}

void vpTemplateTrackerWarpSRT::getParamPyramidUp(const vpColVector &p, vpColVector &pup)
{
  pup = p;
  pup[2] = p[2] * 2.;
  pup[3] = p[3] * 2.;
}
/*calcul de di*dw(x,p0)/dp
 */
void vpTemplateTrackerWarpSRT::getdW0(const int &i, const int &j, const double &dy, const double &dx, double *dIdW)
{
  //   std::cout << "getdW0" << std::endl;
  dIdW[0] = j * dx + i * dy;
  dIdW[1] = -i * dx + j * dy;
  dIdW[2] = dx;
  dIdW[3] = dy;
}
/*calcul de dw(x,p0)/dp
 */
void vpTemplateTrackerWarpSRT::getdWdp0(const int &i, const int &j, double *dIdW)
{
  dIdW[0] = j;
  dIdW[1] = -i;
  dIdW[2] = 1.;
  dIdW[3] = 0;

  dIdW[4] = i;
  dIdW[5] = j;
  dIdW[6] = 0;
  dIdW[7] = 1.;
}

void vpTemplateTrackerWarpSRT::warpX(const int &i, const int &j, double &i2, double &j2, const vpColVector &ParamM)
{
  j2 = ((1.0 + ParamM[0]) * cos(ParamM[1]) * j) - ((1.0 + ParamM[0]) * sin(ParamM[1]) * i) + ParamM[2];
  i2 = ((1.0 + ParamM[0]) * sin(ParamM[1]) * j) + ((1.0 + ParamM[0]) * cos(ParamM[1]) * i) + ParamM[3];
}

void vpTemplateTrackerWarpSRT::warpX(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  vXres[0] = ((1.0 + ParamM[0]) * cos(ParamM[1]) * vX[0]) - ((1.0 + ParamM[0]) * sin(ParamM[1]) * vX[1]) + ParamM[2];
  vXres[1] = ((1.0 + ParamM[0]) * sin(ParamM[1]) * vX[0]) + ((1.0 + ParamM[0]) * cos(ParamM[1]) * vX[1]) + ParamM[3];
}

void vpTemplateTrackerWarpSRT::dWarp(const vpColVector &X1, const vpColVector & /*X2*/, const vpColVector &ParamM,
                                     vpMatrix &dW_)
{
  double j = X1[0];
  double i = X1[1];
  dW_ = 0;
  dW_[0][0] = cos(ParamM[1]) * j - sin(ParamM[1]) * i;
  dW_[0][1] = (-(1.0 + ParamM[0]) * sin(ParamM[1]) * j) - ((1.0 + ParamM[0]) * cos(ParamM[1]) * i);
  dW_[0][2] = 1;

  dW_[1][0] = sin(ParamM[1]) * j + cos(ParamM[1]) * i;
  dW_[1][1] = (1.0 + ParamM[0]) * cos(ParamM[1]) * j - (1.0 + ParamM[0]) * sin(ParamM[1]) * i;
  dW_[1][3] = 1;
}

/*compute dw=dw/dx*dw/dp
 */
void vpTemplateTrackerWarpSRT::dWarpCompo(const vpColVector & /*X1*/, const vpColVector & /*X2*/,
                                          const vpColVector &ParamM, const double *dwdp0, vpMatrix &dW_)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dW_[0][i] =
        ((1. + ParamM[0]) * cos(ParamM[1]) * dwdp0[i]) - ((1.0 + ParamM[0]) * sin(ParamM[1]) * dwdp0[i + nbParam]);
    dW_[1][i] =
        ((1. + ParamM[0]) * sin(ParamM[1]) * dwdp0[i]) + ((1.0 + ParamM[0]) * cos(ParamM[1]) * dwdp0[i + nbParam]);
  }
}

void vpTemplateTrackerWarpSRT::warpXInv(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  //   std::cout << "warpXspe" << std::endl;
  vXres[0] = ((1.0 + ParamM[0]) * cos(ParamM[1]) * vX[0]) - ((1.0 + ParamM[0]) * sin(ParamM[1]) * vX[1]) + ParamM[2];
  vXres[1] = ((1.0 + ParamM[0]) * sin(ParamM[1]) * vX[0]) + ((1.0 + ParamM[0]) * cos(ParamM[1]) * vX[1]) + ParamM[3];
}

void vpTemplateTrackerWarpSRT::getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const
{
  vpColVector Trans(2);
  vpMatrix MWrap(2, 2);
  Trans[0] = ParamM[2];
  Trans[1] = ParamM[3];
  MWrap[0][0] = cos(ParamM[1]);
  MWrap[0][1] = -sin(ParamM[1]);
  MWrap[1][0] = sin(ParamM[1]);
  MWrap[1][1] = cos(ParamM[1]);

  vpMatrix MWrapInv(2, 2);
  MWrapInv = MWrap.transpose();
  vpColVector TransInv(2);
  TransInv = (-1.0 / (1.0 + ParamM[0])) * MWrapInv * Trans;

  ParamMinv[0] = 1.0 / (1.0 + ParamM[0]) - 1.0;
  ParamMinv[1] = atan2(MWrapInv[1][0], MWrapInv[1][1]);
  ParamMinv[2] = TransInv[0];
  ParamMinv[3] = TransInv[1];
}

void vpTemplateTrackerWarpSRT::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const
{
  vpColVector Trans1(2);
  vpMatrix MWrap1(2, 2);
  Trans1[0] = p1[2];
  Trans1[1] = p1[3];

  MWrap1[0][0] = cos(p1[1]);
  MWrap1[0][1] = -sin(p1[1]);
  MWrap1[1][0] = sin(p1[1]);
  MWrap1[1][1] = cos(p1[1]);

  vpColVector Trans2(2);
  vpMatrix MWrap2(2, 2);
  Trans2[0] = p2[2];
  Trans2[1] = p2[3];

  MWrap2[0][0] = cos(p2[1]);
  MWrap2[0][1] = -sin(p2[1]);
  MWrap2[1][0] = sin(p2[1]);
  MWrap2[1][1] = cos(p2[1]);

  vpColVector TransRes(2);
  vpMatrix MWrapRes(2, 2);
  TransRes = (1.0 + p1[0]) * MWrap1 * Trans2 + Trans1;
  MWrapRes = MWrap1 * MWrap2;

  pres[0] = (1.0 + p1[0]) * (1.0 + p2[0]) - 1.0;
  pres[1] = atan2(MWrapRes[1][0], MWrapRes[1][1]);

  pres[2] = TransRes[0];
  pres[3] = TransRes[1];
}
