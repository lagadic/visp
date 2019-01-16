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
#include <visp3/tt/vpTemplateTrackerWarpAffine.h>

vpTemplateTrackerWarpAffine::vpTemplateTrackerWarpAffine()
{
  nbParam = 6;
  dW.resize(2, nbParam);
}

// get the parameter corresponding to the lower level of a gaussian pyramid
void vpTemplateTrackerWarpAffine::getParamPyramidDown(const vpColVector &p, vpColVector &pdown)
{
  pdown = p;
  pdown[4] = p[4] / 2.;
  pdown[5] = p[5] / 2.;
}

void vpTemplateTrackerWarpAffine::getParamPyramidUp(const vpColVector &p, vpColVector &pup)
{
  pup = p;
  pup[4] = p[4] * 2.;
  pup[5] = p[5] * 2.;
}
/*calcul de di*dw(x,p0)/dp
 */
void vpTemplateTrackerWarpAffine::getdW0(const int &i, const int &j, const double &dy, const double &dx, double *dIdW)
{
  dIdW[0] = j * dx;
  dIdW[1] = j * dy;
  dIdW[2] = i * dx;
  dIdW[3] = i * dy;
  dIdW[4] = dx;
  dIdW[5] = dy;
}
/*calcul de dw(x,p0)/dp
 */
void vpTemplateTrackerWarpAffine::getdWdp0(const int &i, const int &j, double *dIdW)
{
  dIdW[0] = j;
  dIdW[1] = 0;
  dIdW[2] = i;
  dIdW[3] = 0;
  dIdW[4] = 1.;
  dIdW[5] = 0;

  dIdW[6] = 0;
  dIdW[7] = j;
  dIdW[8] = 0;
  dIdW[9] = i;
  dIdW[10] = 0;
  dIdW[11] = 1.;
}

void vpTemplateTrackerWarpAffine::warpX(const int &i, const int &j, double &i2, double &j2, const vpColVector &ParamM)
{
  j2 = (1 + ParamM[0]) * j + ParamM[2] * i + ParamM[4];
  i2 = ParamM[1] * j + (1 + ParamM[3]) * i + ParamM[5];
}

void vpTemplateTrackerWarpAffine::warpX(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  vXres[0] = (1.0 + ParamM[0]) * vX[0] + ParamM[2] * vX[1] + ParamM[4];
  vXres[1] = ParamM[1] * vX[0] + (1.0 + ParamM[3]) * vX[1] + ParamM[5];
}

void vpTemplateTrackerWarpAffine::dWarp(const vpColVector &X1, const vpColVector & /*X2*/,
                                        const vpColVector & /*ParamM*/, vpMatrix &dW_)
{
  double j = X1[0];
  double i = X1[1];
  dW_ = 0;
  dW_[0][0] = j;
  dW_[0][2] = i;
  dW_[0][4] = 1;
  dW_[1][1] = j;
  dW_[1][3] = i;
  dW_[1][5] = 1;
}

/*compute dw=dw/dx*dw/dp
 */
void vpTemplateTrackerWarpAffine::dWarpCompo(const vpColVector & /*X1*/, const vpColVector & /*X2*/,
                                             const vpColVector &ParamM, const double *dwdp0, vpMatrix &dW_)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dW_[0][i] = (1. + ParamM[0]) * dwdp0[i] + ParamM[2] * dwdp0[i + nbParam];
    dW_[1][i] = ParamM[1] * dwdp0[i] + (1. + ParamM[3]) * dwdp0[i + nbParam];
  }
}

void vpTemplateTrackerWarpAffine::warpXInv(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  vXres[0] = (1 + ParamM[0]) * vX[0] + ParamM[2] * vX[1] + ParamM[4];
  vXres[1] = ParamM[1] * vX[0] + (1 + ParamM[3]) * vX[1] + ParamM[5];
}
void vpTemplateTrackerWarpAffine::getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const
{
  vpColVector Trans(2);
  vpMatrix MWrap(2, 2);
  Trans[0] = ParamM[4];
  Trans[1] = ParamM[5];
  MWrap[0][0] = 1 + ParamM[0];
  MWrap[0][1] = ParamM[2];
  MWrap[1][0] = ParamM[1];
  MWrap[1][1] = 1 + ParamM[3];

  vpMatrix MWrapInv(2, 2);
  MWrapInv = MWrap.inverseByLU();
  vpColVector TransInv(2);
  TransInv = -1 * MWrapInv * Trans;

  ParamMinv[0] = MWrapInv[0][0] - 1;
  ParamMinv[2] = MWrapInv[0][1];
  ParamMinv[1] = MWrapInv[1][0];
  ParamMinv[3] = MWrapInv[1][1] - 1;
  ParamMinv[4] = TransInv[0];
  ParamMinv[5] = TransInv[1];
}

void vpTemplateTrackerWarpAffine::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const
{
  vpColVector Trans1(2);
  vpMatrix MWrap1(2, 2);
  Trans1[0] = p1[4];
  Trans1[1] = p1[5];
  MWrap1[0][0] = 1 + p1[0];
  MWrap1[0][1] = p1[2];
  MWrap1[1][0] = p1[1];
  MWrap1[1][1] = 1 + p1[3];

  vpColVector Trans2(2);
  vpMatrix MWrap2(2, 2);
  Trans2[0] = p2[4];
  Trans2[1] = p2[5];
  MWrap2[0][0] = 1 + p2[0];
  MWrap2[0][1] = p2[2];
  MWrap2[1][0] = p2[1];
  MWrap2[1][1] = 1 + p2[3];

  vpColVector TransRes(2);
  vpMatrix MWrapRes(2, 2);
  TransRes = MWrap1 * Trans2 + Trans1;
  MWrapRes = MWrap1 * MWrap2;

  pres[0] = MWrapRes[0][0] - 1;
  pres[2] = MWrapRes[0][1];
  pres[1] = MWrapRes[1][0];
  pres[3] = MWrapRes[1][1] - 1;
  pres[4] = TransRes[0];
  pres[5] = TransRes[1];
}
