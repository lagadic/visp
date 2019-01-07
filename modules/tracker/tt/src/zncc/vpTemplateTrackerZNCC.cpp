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
#include <visp3/tt/vpTemplateTrackerZNCC.h>

vpTemplateTrackerZNCC::vpTemplateTrackerZNCC(vpTemplateTrackerWarp *warp) : vpTemplateTracker(warp), DI(), temp()
{
  dW.resize(2, nbParam);
  G.resize(nbParam);
  H.resize(nbParam, nbParam);
  HLM.resize(nbParam, nbParam);

  temp.resize(nbParam);

  X1.resize(2);
  X2.resize(2);
  DI.resize(2);
}

double vpTemplateTrackerZNCC::getCost(const vpImage<unsigned char> &I, const vpColVector &tp)
{
  double IW, Tij;
  int i, j;
  double i2, j2;
  int Nbpoint = 0;

  Warp->computeCoeff(tp);

  double moyTij = 0;
  double moyIW = 0;
  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);

    j2 = X2[0];
    i2 = X2[1];
    if ((j2 < I.getWidth() - 1) && (i2 < I.getHeight() - 1) && (i2 > 0) && (j2 > 0)) {
      Tij = ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);
      // IW=getSubPixBspline4(I,i2,j2);
      moyTij += Tij;
      moyIW += IW;
      Nbpoint++;
    }
  }
  ratioPixelIn = (double)Nbpoint / (double)templateSize;
  if (!Nbpoint) {
    throw(vpException(vpException::divideByZeroError, "Cannot get cost: size = 0"));
  }

  moyTij = moyTij / Nbpoint;
  moyIW = moyIW / Nbpoint;

  double nom = 0; //,denom=0;
  double var1 = 0, var2 = 0;
  for (unsigned int point = 0; point < templateSize; point++) {
    i = ptTemplate[point].y;
    j = ptTemplate[point].x;
    X1[0] = j;
    X1[1] = i;
    Warp->computeDenom(X1, tp);
    Warp->warpX(X1, X2, tp);

    j2 = X2[0];
    i2 = X2[1];
    if ((j2 < I.getWidth() - 1) && (i2 < I.getHeight() - 1) && (i2 > 0) && (j2 > 0)) {
      Tij = ptTemplate[point].val;
      if (!blur)
        IW = I.getValue(i2, j2);
      else
        IW = BI.getValue(i2, j2);
      // IW=getSubPixBspline4(I,i2,j2);
      nom += (Tij - moyTij) * (IW - moyIW);
      // denom+=(Tij-moyTij)*(Tij-moyTij)*(IW-moyIW)*(IW-moyIW);
      var1 += (IW - moyIW) * (IW - moyIW);
      var2 += (Tij - moyTij) * (Tij - moyTij);

      Nbpoint++;
    }
  }
  // if(Nbpoint==0)return 10e10; // cannot occur
  // return -nom/sqrt(denom);
  return -nom / sqrt(var1 * var2);
}
