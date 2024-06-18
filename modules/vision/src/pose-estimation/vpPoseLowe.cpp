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
 *
 * Description:
 * Pose computation.
 */

#include <float.h>
#include <limits> // numeric_limits
#include <math.h>
#include <string.h>

// besoin de la librairie mathematique, en particulier des
// fonctions de minimization de Levenberg Marquartd
#include "private/vpLevenbergMarquartd.h"
#include <visp3/vision/vpPose.h>

#define NBR_PAR 6
#define X3_SIZE 3
#define MINIMUM 0.000001

/*
 * MACRO  : MIJ
 *
 * ENTREE  :
 * m    Matrice.
 * i    Indice ligne   de l'element.
 * j    Indice colonne de l'element.
 * s    Taille en nombre d'elements d'une ligne de la matrice "m".
 *
 * DESCRIPTION  :
 * La macro-instruction calcule l'adresse de l'element de la "i"eme ligne et
 * de la "j"eme colonne de la matrice "m", soit &m[i][j].
 *
 * RETOUR  :
 * L'adresse de m[i][j] est retournee.
 *
 * HISTORIQUE  :
 * 1.00 - 11/02/93 - Original.
 */
#define MIJ(m, i, j, s) ((m) + ((long)(i) * (long)(s)) + (long)(j))
#define NBPTMAX 50
#define MINI 0.001
#define MINIMUM 0.000001

BEGIN_VISP_NAMESPACE

// ------------------------------------------------------------------------
//   FONCTION LOWE :
// ------------------------------------------------------------------------
// Calcul de la pose pour un objet 3D
// ------------------------------------------------------------------------

// Je hurle d'horreur devant ces variable globale...
static double XI[NBPTMAX], YI[NBPTMAX];
static double XO[NBPTMAX], YO[NBPTMAX], ZO[NBPTMAX];

void eval_function(int npt, double *xc, double *f);
void fcn(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag);

void eval_function(int npt, double *xc, double *f)
{
  int i;
  const unsigned int sizeU = 3;
  double u[sizeU];
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;

  u[index_0] = xc[index_3]; /* Rx   */
  u[index_1] = xc[index_4]; /* Ry   */
  u[index_2] = xc[index_5]; /* Rz   */

  vpRotationMatrix rd(u[index_0], u[index_1], u[index_2]);
  // --comment:  rot_mat(u,rd) matrice de rotation correspondante
  for (i = 0; i < npt; ++i) {
    double x = (rd[index_0][index_0] * XO[i]) + (rd[index_0][index_1] * YO[i]) + (rd[index_0][index_2] * ZO[i]) + xc[index_0];
    double y = (rd[index_1][index_0] * XO[i]) + (rd[index_1][index_1] * YO[i]) + (rd[index_1][index_2] * ZO[i]) + xc[index_1];
    double z = (rd[index_2][index_0] * XO[i]) + (rd[index_2][index_1] * YO[i]) + (rd[index_2][index_2] * ZO[i]) + xc[index_2];
    f[i] = (x / z) - XI[i];
    f[npt + i] = (y / z) - YI[i];
    // --comment: write fi and fi+1
  }
}

/*
 * PROCEDURE  : fcn
 *
 * ENTREES  :
 * m    Nombre d'equations.
 * n    Nombre de variables.
 * xc    Valeur courante des parametres.
 * fvecc  Resultat de l'evaluation de la fonction.
 * ldfjac  Plus grande dimension de la matrice jac.
 * iflag  Choix du calcul de la fonction ou du jacobien.
 *
 * SORTIE  :
 * jac    Jacobien de la fonction.
 *
 * DESCRIPTION  :
 * La procedure calcule la fonction et le jacobien.
 * Si iflag == 1, la procedure calcule la fonction en "xc" et le resultat est
 *       stocke dans "fvecc" et "fjac" reste inchange.
 * Si iflag == 2, la procedure calcule le jacobien en "xc" et le resultat est
 *       stocke dans "fjac" et "fvecc" reste inchange.
 *
 *  HISTORIQUE     :
 * 1.00 - xx/xx/xx - Original.
 * 1.01 - 06/07/95 - Modifications.
 * 2.00 - 24/10/95 - Tableau jac monodimensionnel.
 */
void fcn(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag)
{
  double u[X3_SIZE]; // rd[X3_SIZE][X3_SIZE],
  vpRotationMatrix rd;
  int npt;

  if (m < n) {
    printf("pas assez de points\n");
  }
  const int half = 2;
  npt = m / half;

  const int flagFunc = 1, flagJacobian = 2;
  if (iflag == flagFunc) {
    eval_function(npt, xc, fvecc);
  }
  else if (iflag == flagJacobian) {
    double u1, u2, u3;
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    const unsigned int index_3 = 3;
    const unsigned int index_4 = 4;
    const unsigned int index_5 = 5;
    u[index_0] = xc[index_3];
    u[index_1] = xc[index_4];
    u[index_2] = xc[index_5];

    rd.build(u[index_0], u[index_1], u[index_2]);
    /* a partir de l'axe de rotation, calcul de la matrice de rotation. */
    // --comment: rot_mat of u rd

    double tt = sqrt((u[index_0] * u[index_0]) + (u[index_1] * u[index_1]) + (u[index_2] * u[index_2])); /* angle de rot */
    if (tt >= MINIMUM) {
      u1 = u[index_0] / tt;
      u2 = u[index_1] / tt; /* axe de rotation unitaire  */
      u3 = u[index_2] / tt;
    }
    else {
      u1 = 0.0;
      u2 = 0.0;
      u3 = 0.0;
    }
    double co = cos(tt);
    double mco = 1.0 - co;
    double si = sin(tt);

    for (int i = 0; i < npt; ++i) {
      double x = XO[i];
      double y = YO[i]; /* coordonnees du point i  */
      double z = ZO[i];

      /* coordonnees du point i dans le repere camera  */
      double rx = (rd[index_0][index_0] * x) + (rd[index_0][index_1] * y) + (rd[index_0][index_2] * z) + xc[index_0];
      double ry = (rd[index_1][index_0] * x) + (rd[index_1][index_1] * y) + (rd[index_1][index_2] * z) + xc[index_1];
      double rz = (rd[index_2][index_0] * x) + (rd[index_2][index_1] * y) + (rd[index_2][index_2] * z) + xc[index_2];

      /* derive des fonctions rx, ry et rz par rapport
       * a tt, u1, u2, u3.
       */
      double drxt = (((si * u1 * u3) + (co * u2)) * z) + (((si * u1 * u2) - (co * u3)) * y) + (((si * u1 * u1) - si) * x);
      double drxu1 = (mco * u3 * z) + (mco * u2 * y) + (2. * mco * u1 * x);
      double drxu2 = (si * z) + (mco * u1 * y);
      double drxu3 = (mco * u1 * z) - (si * y);

      double dryt = (((si * u2 * u3) - (co * u1)) * z) + (((si * u2 * u2) - si) * y) + (((co * u3) + (si * u1 * u2)) * x);
      double dryu1 = (mco * u2 * x) - (si * z);
      double dryu2 = (mco * u3 * z) + (2 * mco * u2 * y) + (mco * u1 * x);
      double dryu3 = (mco * u2 * z) + (si * x);

      double drzt = (((si * u3 * u3) - si) * z) + (((si * u2 * u3) + (co * u1)) * y) + (((si * u1 * u3) - (co * u2)) * x);
      double drzu1 = (si * y) + (mco * u3 * x);
      double drzu2 = (mco * u3 * y) - (si * x);
      double drzu3 = (2 * mco * u3 * z) + (mco * u2 * y) + (mco * u1 * x);

      /* derive de la fonction representant le modele de la
       * camera (sans distortion) par rapport a tt, u1, u2 et u3.
       */
      double dxit = (drxt / rz) - ((rx * drzt) / (rz * rz));

      double dyit = (dryt / rz) - ((ry * drzt) / (rz * rz));

      double dxiu1 = (drxu1 / rz) - ((drzu1 * rx) / (rz * rz));
      double dyiu1 = (dryu1 / rz) - ((drzu1 * ry) / (rz * rz));

      double dxiu2 = (drxu2 / rz) - ((drzu2 * rx) / (rz * rz));
      double dyiu2 = (dryu2 / rz) - ((drzu2 * ry) / (rz * rz));

      double dxiu3 = (drxu3 / rz) - ((drzu3 * rx) / (rz * rz));
      double dyiu3 = (dryu3 / rz) - ((drzu3 * ry) / (rz * rz));

      /* calcul du jacobien : le jacobien represente la
       * derivee de la fonction representant le modele de la
       * camera par rapport aux parametres.
       */
      *MIJ(jac, index_0, i, ldfjac) = 1. / rz;
      *MIJ(jac, index_1, i, ldfjac) = 0.0;
      *MIJ(jac, index_2, i, ldfjac) = -rx / (rz * rz);
      if (tt >= MINIMUM) {
        *MIJ(jac, index_3, i, ldfjac) = (((u1 * dxit) + (((1. - (u1 * u1)) * dxiu1) / tt)) - ((u1 * u2 * dxiu2) / tt)) - ((u1 * u3 * dxiu3) / tt);
        *MIJ(jac, index_4, i, ldfjac) = (((u2 * dxit) - ((u1 * u2 * dxiu1) / tt)) + (((1. - (u2 * u2)) * dxiu2) / tt)) - ((u2 * u3 * dxiu3) / tt);

        *MIJ(jac, index_5, i, ldfjac) = (((u3 * dxit) - ((u1 * u3 * dxiu1) / tt)) - ((u2 * u3 * dxiu2) / tt)) + (((1. - (u3 * u3)) * dxiu3) / tt);
      }
      else {
        *MIJ(jac, index_3, i, ldfjac) = 0.0;
        *MIJ(jac, index_4, i, ldfjac) = 0.0;
        *MIJ(jac, index_5, i, ldfjac) = 0.0;
      }
      *MIJ(jac, index_0, npt + i, ldfjac) = 0.0;
      *MIJ(jac, index_1, npt + i, ldfjac) = 1 / rz;
      *MIJ(jac, index_2, npt + i, ldfjac) = -ry / (rz * rz);
      if (tt >= MINIMUM) {
        *MIJ(jac, index_3, npt + i, ldfjac) =
          (((u1 * dyit) + (((1 - (u1 * u1)) * dyiu1) / tt)) - ((u1 * u2 * dyiu2) / tt)) - ((u1 * u3 * dyiu3) / tt);
        *MIJ(jac, index_4, npt + i, ldfjac) =
          (((u2 * dyit) - ((u1 * u2 * dyiu1) / tt)) + (((1. - (u2 * u2)) * dyiu2) / tt)) - ((u2 * u3 * dyiu3) / tt);
        *MIJ(jac, index_5, npt + i, ldfjac) =
          (((u3 * dyit) - ((u1 * u3 * dyiu1) / tt)) - ((u2 * u3 * dyiu2) / tt)) + (((1. - (u3 * u3)) * dyiu3) / tt);
      }
      else {
        *MIJ(jac, index_3, npt + i, ldfjac) = 0.0;
        *MIJ(jac, index_4, npt + i, ldfjac) = 0.0;
        *MIJ(jac, index_5, npt + i, ldfjac) = 0.0;
      }
    }
  } /* fin else if iflag ==2  */
}

void vpPose::poseLowe(vpHomogeneousMatrix &cMo)
{
  /* nombre d'elements dans la matrice jac */
  const int n = NBR_PAR; /* nombres d'inconnues  */
  const int m = static_cast<int>(2 * npt); /* nombres d'equations  */

  const int lwa = (2 * NBPTMAX) + 50;    /* taille du vecteur wa */
  const int ldfjac = 2 * NBPTMAX; /* taille maximum d'une ligne de jac */
  int info, ipvt[NBR_PAR];
  int tst_lmder;
  double f[ldfjac], sol[NBR_PAR];
  double tol, jac[NBR_PAR][ldfjac], wa[lwa];
  // --comment:  double  u of 3  (vecteur de rotation)
  // --comment:  double  rd of 3 by 3 (matrice de rotation)

  tol = std::numeric_limits<double>::epsilon(); /* critere d'arret  */

  // --comment: c eq cam
  // --comment: for i eq 0 to 3
  // --comment: for j eq 0 to 3 rd[i][j] = cMo[i][j]
  // --comment:  mat_rot of rd and u
  vpRotationMatrix cRo;
  cMo.extract(cRo);
  vpThetaUVector u(cRo);
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    sol[i] = cMo[i][val_3];
    sol[i + val_3] = u[i];
  }

  vpPoint P;
  unsigned int i_ = 0;
  std::list<vpPoint>::const_iterator listp_end = listP.end();
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listp_end; ++it) {
    P = *it;
    XI[i_] = P.get_x(); // --comment: *cam.px plus cam.xc
    YI[i_] = P.get_y(); // --comment: *cam.py plus cam.yc
    XO[i_] = P.get_oX();
    YO[i_] = P.get_oY();
    ZO[i_] = P.get_oZ();
    ++i_;
  }
  tst_lmder = lmder1(&fcn, m, n, sol, f, &jac[0][0], ldfjac, tol, &info, ipvt, lwa, wa);
  if (tst_lmder == -1) {
    std::cout << " in CCalculPose::PoseLowe(...) : ";
    std::cout << "pb de minimization,  returns FATAL_ERROR";
    // --comment: return FATAL ERROR
  }

  for (unsigned int i = 0; i < val_3; ++i) {
    u[i] = sol[i + val_3];
  }

  for (unsigned int i = 0; i < val_3; ++i) {
    cMo[i][val_3] = sol[i];
    u[i] = sol[i + val_3];
  }

  vpRotationMatrix rd(u);
  cMo.insert(rd);
}

END_VISP_NAMESPACE

#undef MINI
#undef MINIMUM
