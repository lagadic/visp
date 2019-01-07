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
#include <visp3/tt/vpTemplateTrackerWarp.h>

void vpTemplateTrackerWarp::warpTriangle(const vpTemplateTrackerTriangle &in, const vpColVector &p,
                                         vpTemplateTrackerTriangle &out)
{
  if (p.size() < 2) {
    vpCTRACE << "Bad template tracker warp parameters dimension. Should "
                "never occur. "
             << std::endl;
    throw(vpException(vpException::dimensionError, "Bad template tracker warp parameters dimension"));
  }
  vpColVector S1(2), S2(2), S3(2);
  vpColVector rS1(2), rS2(2), rS3(2);
  in.getCorners(S1, S2, S3);
  computeDenom(S1, p);
  warpX(S1, rS1, p);
  computeDenom(S2, p);
  warpX(S2, rS2, p);
  computeDenom(S3, p);
  warpX(S3, rS3, p);
  out.init(rS1, rS2, rS3);
}
void vpTemplateTrackerWarp::warpZone(const vpTemplateTrackerZone &in, const vpColVector &p, vpTemplateTrackerZone &out)
{
  vpTemplateTrackerTriangle TR, TT;
  out.clear();
  for (unsigned int i = 0; i < in.getNbTriangle(); i++) {
    in.getTriangle(i, TR);
    warpTriangle(TR, p, TT);
    out.add(TT);
  }
}

double vpTemplateTrackerWarp::getDistanceBetweenZoneAndWarpedZone(const vpTemplateTrackerZone &Z, const vpColVector &p)
{
  unsigned int nb_corners = Z.getNbTriangle() * 3;
  computeCoeff(p);
  vpColVector X1(2), X2(2);

  double res = 0;
  vpTemplateTrackerTriangle triangle;
  for (unsigned int i = 0; i < Z.getNbTriangle(); i++) {
    Z.getTriangle(i, triangle);
    for (unsigned int j = 0; j < 3; j++) {
      triangle.getCorner(j, X1[0], X1[1]);

      computeDenom(X1, p);
      warpX(X1, X2, p);
      res += sqrt((X2[0] - X1[0]) * (X2[0] - X1[0]) + (X2[1] - X1[1]) * (X2[1] - X1[1]));
    }
  }

  return res / nb_corners;
}

void vpTemplateTrackerWarp::warp(const double *ut0, const double *vt0, int nb_pt, const vpColVector &p, double *u,
                                 double *v)
{
  computeCoeff(p);
  vpColVector X1(2), X2(2);
  for (int i = 0; i < nb_pt; i++) {
    X1[0] = ut0[i];
    X1[1] = vt0[i];
    computeDenom(X1, p);
    warpX(X1, X2, p);
    u[i] = X2[0];
    v[i] = X2[1];
    // std::cout<<"warp "<<X2[0]<<","<<X2[1]<<std::endl;
  }
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
void vpTemplateTrackerWarp::findWarp(const double *ut0, const double *vt0, const double *u, const double *v, int nb_pt,
                                     vpColVector &p)
{
  vpMatrix dW_(2, nbParam);
  vpMatrix dX(2, 1);
  vpMatrix H(nbParam, nbParam), HLM(nbParam, nbParam);
  vpMatrix G(nbParam, 1);

  int cpt = 0;
  vpColVector X1(2);
  vpColVector fX1(2);
  vpColVector X2(2);
  double erreur = 0;
  double erreur_prec;
  double lambda = 0.01;
  do {
    erreur_prec = erreur;
    H = 0;
    G = 0;
    erreur = 0;
    computeCoeff(p);
    for (int i = 0; i < nb_pt; i++) {
      X1[0] = ut0[i];
      X1[1] = vt0[i];
      computeDenom(X1, p);
      warpX(X1, fX1, p);
      dWarp(X1, fX1, p, dW_);
      H += dW_.AtA();

      X2[0] = u[i];
      X2[1] = v[i];

      dX = X2 - fX1;
      G += dW_.t() * dX;

      erreur += ((u[i] - fX1[0]) * (u[i] - fX1[0]) + (v[i] - fX1[1]) * (v[i] - fX1[1]));
    }

    vpMatrix::computeHLM(H, lambda, HLM);
    try {
      p += (vpColVector)(HLM.inverseByLU() * G, 0u);
    } catch (const vpException &e) {
      // std::cout<<"Cannot inverse the matrix by LU " << std::endl;
      throw(e);
    }
    cpt++;
  } while ((cpt < 150) && (sqrt((erreur_prec - erreur) * (erreur_prec - erreur)) > 1e-20));
  // std::cout<<"erreur apres transformation="<<erreur<<std::endl;
}
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
