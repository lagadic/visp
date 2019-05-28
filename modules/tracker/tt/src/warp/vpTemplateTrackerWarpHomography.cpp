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
#include <visp3/core/vpTrackingException.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>

vpTemplateTrackerWarpHomography::vpTemplateTrackerWarpHomography()
{
  nbParam = 8;
  dW.resize(2, nbParam);
}

// get the parameter corresponding to the lower level of a gaussian pyramid
void vpTemplateTrackerWarpHomography::getParamPyramidDown(const vpColVector &p, vpColVector &pdown)
{
  pdown = p;
  pdown[2] = p[2] * 2.;
  pdown[5] = p[5] * 2.;
  pdown[6] = p[6] / 2.;
  pdown[7] = p[7] / 2.;
}

void vpTemplateTrackerWarpHomography::getParamPyramidUp(const vpColVector &p, vpColVector &pup)
{
  pup = p;
  pup[2] = p[2] / 2.;
  pup[5] = p[5] / 2.;
  pup[6] = p[6] * 2.;
  pup[7] = p[7] * 2.;
}

/*calcul de di*dw(x,p0)/dp  */
void vpTemplateTrackerWarpHomography::getdW0(const int &i, const int &j, const double &dy, const double &dx,
                                             double *dIdW)
{
  dIdW[0] = j * dx;
  dIdW[1] = j * dy;
  dIdW[2] = -j * j * dx - i * j * dy;
  dIdW[3] = i * dx;
  dIdW[4] = i * dy;
  dIdW[5] = -i * j * dx - i * i * dy;
  dIdW[6] = dx;
  dIdW[7] = dy;
}
/*calcul de dw(x,p0)/dp  */
void vpTemplateTrackerWarpHomography::getdWdp0(const int &i, const int &j, double *dIdW)
{
  dIdW[0] = j;
  dIdW[1] = 0;
  dIdW[2] = -j * j;
  dIdW[3] = i;
  dIdW[4] = 0;
  dIdW[5] = -i * j;
  dIdW[6] = 1.;
  dIdW[7] = 0;

  dIdW[8] = 0;
  dIdW[9] = j;
  dIdW[10] = -i * j;
  dIdW[11] = 0;
  dIdW[12] = i;
  dIdW[13] = -i * i;
  dIdW[14] = 0;
  dIdW[15] = 1.;
}
void vpTemplateTrackerWarpHomography::computeDenom(vpColVector &vX, const vpColVector &ParamM)
{
  denom = (1. / (ParamM[2] * vX[0] + ParamM[5] * vX[1] + 1.));
}

void vpTemplateTrackerWarpHomography::warpX(const int &i, const int &j, double &i2, double &j2,
                                            const vpColVector &ParamM)
{
  j2 = ((1. + ParamM[0]) * j + ParamM[3] * i + ParamM[6]) * denom;
  i2 = (ParamM[1] * j + (1. + ParamM[4]) * i + ParamM[7]) * denom;
}

void vpTemplateTrackerWarpHomography::warpX(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{
  // if((ParamM[2]*vX[0]+ParamM[5]*vX[1]+1)>0)//si dans le plan image reel
  if ((denom) > 0) // FS optimisation
  {
    vXres[0] = ((1 + ParamM[0]) * vX[0] + ParamM[3] * vX[1] + ParamM[6]) * denom;
    vXres[1] = (ParamM[1] * vX[0] + (1 + ParamM[4]) * vX[1] + ParamM[7]) * denom;
  } else
    throw(vpTrackingException(vpTrackingException::fatalError,
                              "Division by zero in vpTemplateTrackerWarpHomography::warpX()"));
}

void vpTemplateTrackerWarpHomography::dWarp(const vpColVector &X1, const vpColVector &X2,
                                            const vpColVector & /*ParamM*/, vpMatrix &dW_)
{
  double j = X1[0];
  double i = X1[1];
  dW_ = 0;
  dW_[0][0] = j * denom;
  dW_[0][2] = -j * X2[0] * denom;
  dW_[0][3] = i * denom;
  dW_[0][5] = -i * X2[0] * denom;
  dW_[0][6] = denom;

  dW_[1][1] = j * denom;
  dW_[1][2] = -j * X2[1] * denom;
  dW_[1][4] = i * denom;
  dW_[1][5] = -i * X2[1] * denom;
  dW_[1][7] = denom;
}

/*compute dw=dw/dx*dw/dp  */
void vpTemplateTrackerWarpHomography::dWarpCompo(const vpColVector & /*X1*/, const vpColVector &X2,
                                                 const vpColVector &ParamM, const double *dwdp0, vpMatrix &dW_)
{
  double dwdx0, dwdx1;
  double dwdy0, dwdy1;

  dwdx0 = ((1. + ParamM[0]) - X2[0] * ParamM[2]) * denom;
  dwdx1 = (ParamM[1] - X2[1] * ParamM[2]) * denom;
  dwdy0 = (ParamM[3] - X2[0] * ParamM[5]) * denom;
  dwdy1 = ((1. + ParamM[4]) - X2[1] * ParamM[5]) * denom;
  for (unsigned int i = 0; i < nbParam; i++) {
    dW_[0][i] = dwdx0 * dwdp0[i] + dwdy0 * dwdp0[i + nbParam];
    dW_[1][i] = dwdx1 * dwdp0[i] + dwdy1 * dwdp0[i + nbParam];
  }
}

void vpTemplateTrackerWarpHomography::warpXInv(const vpColVector &vX, vpColVector &vXres, const vpColVector &ParamM)
{

  if ((ParamM[2] * vX[0] + ParamM[5] * vX[1] + 1) < 0) // si dans le plan image reel
  {
    vXres[0] = ((1 + ParamM[0]) * vX[0] + ParamM[3] * vX[1] + ParamM[6]) / (ParamM[2] * vX[0] + ParamM[5] * vX[1] + 1);
    vXres[1] = (ParamM[1] * vX[0] + (1 + ParamM[4]) * vX[1] + ParamM[7]) / (ParamM[2] * vX[0] + ParamM[5] * vX[1] + 1);
  } else
    throw(vpTrackingException(vpTrackingException::fatalError, "Division by zero in "
                                                               "vpTemplateTrackerWarpHomography::"
                                                               "warpXSpecialInv()"));
}
void vpTemplateTrackerWarpHomography::getParamInverse(const vpColVector &ParamM, vpColVector &ParamMinv) const
{
  vpHomography H = getHomography(ParamM);
  vpHomography Hinv = H.inverse();
  getParam(Hinv, ParamMinv);
}

vpHomography vpTemplateTrackerWarpHomography::getHomography(const vpColVector &ParamM) const
{
  vpHomography H;
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++) {
      if (i + 3 * j != 8) {
        H[i][j] = ParamM[i + 3 * j];
        if (i == j)
          H[i][j]++;
      } else
        H[i][j] = 1.;
    }

  return H;
}
void vpTemplateTrackerWarpHomography::getParam(const vpHomography &H, vpColVector &par) const
{
  par = 0;
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++) {
      if (i + 3 * j != 8) {
        par[i + 3 * j] = H[i][j] / H[2][2];
        if (i == j)
          par[i + 3 * j]--;
      }
    }
}

void vpTemplateTrackerWarpHomography::pRondp(const vpColVector &p1, const vpColVector &p2, vpColVector &pres) const
{
  vpHomography H1 = getHomography(p1);
  vpHomography H2 = getHomography(p2);
  vpHomography H = H1 * H2;
  getParam(H, pres);
}
