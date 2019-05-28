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
#include <visp3/tt/vpTemplateTrackerWarpTranslation.h>

vpTemplateTrackerWarpTranslation::vpTemplateTrackerWarpTranslation()
{
  nbParam = 2;
  dW.resize(2, nbParam);
}

// get the parameter corresponding to the lower level of a gaussian pyramid
void vpTemplateTrackerWarpTranslation::getParamPyramidDown(const vpColVector &p, vpColVector &pdown)
{
  pdown[0] = p[0] / 2.;
  pdown[1] = p[1] / 2.;
}

void vpTemplateTrackerWarpTranslation::getParamPyramidUp(const vpColVector &p, vpColVector &pup)
{
  pup[0] = p[0] * 2.;
  pup[1] = p[1] * 2.;
}

/*calcul de di*dw(x,p0)/dp
 */
void vpTemplateTrackerWarpTranslation::getdW0(const int & /*i*/, const int & /*j*/, const double &dy, const double &dx,
                                              double *dIdW)
{
  dIdW[0] = dx;
  dIdW[1] = dy;
}
/*calcul de dw(x,p0)/dp
 */
void vpTemplateTrackerWarpTranslation::getdWdp0(const int & /*i*/, const int & /*j*/, double *dIdW)
{
  dIdW[0] = 1.;
  dIdW[1] = 0;

  dIdW[2] = 0;
  dIdW[3] = 1.;
}

void vpTemplateTrackerWarpTranslation::warpX(const int &i, const int &j, double &i2, double &j2,
                                             const vpColVector &ParamM)
{
  j2 = j + ParamM[0];
  i2 = i + ParamM[1];
}

void vpTemplateTrackerWarpTranslation::warpX(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  vXres[0] = vX[0] + ParamM[0];
  vXres[1] = vX[1] + ParamM[1];
}

void vpTemplateTrackerWarpTranslation::dWarp(const vpColVector & /*X1*/, const vpColVector & /*X2*/,
                                             const vpColVector & /*ParamM*/, vpMatrix &dW_)
{
  dW_[0][0] = 1;
  dW_[0][1] = 0;
  dW_[1][0] = 0;
  dW_[1][1] = 1;
}

/*compute dw=dw/dx*dw/dp
 */
void vpTemplateTrackerWarpTranslation::dWarpCompo(const vpColVector & /*X1*/, const vpColVector & /*X2*/,
                                                  const vpColVector & /*ParamM*/, const double *dwdp0, vpMatrix &dW_)
{
  for (unsigned int i = 0; i < nbParam; i++) {
    dW_[0][i] = dwdp0[i];
    dW_[1][i] = dwdp0[i + nbParam];
  }
}

void vpTemplateTrackerWarpTranslation::warpXInv(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  vXres[0] = vX[0] + ParamM[0];
  vXres[1] = vX[1] + ParamM[1];
}
void vpTemplateTrackerWarpTranslation::getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const
{
  ParamMinv[0] = -ParamM[0];
  ParamMinv[1] = -ParamM[1];
}

void vpTemplateTrackerWarpTranslation::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const
{
  pres[0] = p1[0] + p2[0];
  pres[1] = p1[1] + p2[1];
}
