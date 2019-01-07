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
 * Pose computation.
 *
 * Authors:
 * Eric Marchand
 * Francois Chaumette
 *
 *****************************************************************************/

#include <float.h>
#include <limits> // numeric_limits
#include <math.h>
#include <string.h>

// besoin de la librairie mathematique, en particulier des
// fonctions de minimisation de Levenberg Marquartd
#include <visp3/vision/vpLevenbergMarquartd.h>
#include <visp3/vision/vpPose.h>

#define NBR_PAR 6
#define X3_SIZE 3
#define MINIMUM 0.000001

#define DEBUG_LEVEL1 0

// ------------------------------------------------------------------------
//   FONCTION LOWE :
// ------------------------------------------------------------------------
// Calcul de la pose pour un objet 3D
// ------------------------------------------------------------------------

/*
 * MACRO	: MIJ
 *
 * ENTREE	:
 * m		Matrice.
 * i		Indice ligne   de l'element.
 * j		Indice colonne de l'element.
 * s		Taille en nombre d'elements d'une ligne de la matrice "m".
 *
 * DESCRIPTION	:
 * La macro-instruction calcule l'adresse de l'element de la "i"eme ligne et
 * de la "j"eme colonne de la matrice "m", soit &m[i][j].
 *
 * RETOUR	:
 * L'adresse de m[i][j] est retournee.
 *
 * HISTORIQUE	:
 * 1.00 - 11/02/93 - Original.
 */
#define MIJ(m, i, j, s) ((m) + ((long)(i) * (long)(s)) + (long)(j))
#define NBPTMAX 50

// Je hurle d'horreur devant ces variable globale...
static double XI[NBPTMAX], YI[NBPTMAX];
static double XO[NBPTMAX], YO[NBPTMAX], ZO[NBPTMAX];

#define MINI 0.001
#define MINIMUM 0.000001

void eval_function(int npt, double *xc, double *f);
void fcn(int m, int n, double *xc, double *fvecc, double *jac, int ldfjac, int iflag);

void eval_function(int npt, double *xc, double *f)
{
  int i;
  double u[3];

  u[0] = xc[3]; /* Rx   */
  u[1] = xc[4]; /* Ry   */
  u[2] = xc[5]; /* Rz   */

  vpRotationMatrix rd(u[0], u[1], u[2]);
  //  rot_mat(u,rd);          /* matrice de rotation correspondante   */
  for (i = 0; i < npt; i++) {
    double x = rd[0][0] * XO[i] + rd[0][1] * YO[i] + rd[0][2] * ZO[i] + xc[0];
    double y = rd[1][0] * XO[i] + rd[1][1] * YO[i] + rd[1][2] * ZO[i] + xc[1];
    double z = rd[2][0] * XO[i] + rd[2][1] * YO[i] + rd[2][2] * ZO[i] + xc[2];
    f[i] = x / z - XI[i];
    f[npt + i] = y / z - YI[i];
    //    std::cout << f[i] << "   " << f[i+1] << std::endl ;
  }
}

/*
 * PROCEDURE	: fcn
 *
 * ENTREES	:
 * m		Nombre d'equations.
 * n		Nombre de variables.
 * xc		Valeur courante des parametres.
 * fvecc	Resultat de l'evaluation de la fonction.
 * ldfjac	Plus grande dimension de la matrice jac.
 * iflag	Choix du calcul de la fonction ou du jacobien.
 *
 * SORTIE	:
 * jac		Jacobien de la fonction.
 *
 * DESCRIPTION	:
 * La procedure calcule la fonction et le jacobien.
 * Si iflag == 1, la procedure calcule la fonction en "xc" et le resultat est
 * 		  stocke dans "fvecc" et "fjac" reste inchange.
 * Si iflag == 2, la procedure calcule le jacobien en "xc" et le resultat est
 * 		  stocke dans "fjac" et "fvecc" reste inchange.
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

  if (m < n)
    printf("pas assez de points\n");
  npt = m / 2;

  if (iflag == 1)
    eval_function(npt, xc, fvecc);
  else if (iflag == 2) {
    double u1, u2, u3;
    u[0] = xc[3];
    u[1] = xc[4];
    u[2] = xc[5];

    rd.buildFrom(u[0], u[1], u[2]);
    /* a partir de l'axe de rotation, calcul de la matrice de rotation. */
    //   rot_mat(u, rd);

    double tt = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]); /* angle de rot */
    if (tt >= MINIMUM) {
      u1 = u[0] / tt;
      u2 = u[1] / tt; /* axe de rotation unitaire  */
      u3 = u[2] / tt;
    } else
      u1 = u2 = u3 = 0.0;
    double co = cos(tt);
    double mco = 1.0 - co;
    double si = sin(tt);

    for (int i = 0; i < npt; i++) {
      double x = XO[i];
      double y = YO[i]; /* coordonnees du point i	*/
      double z = ZO[i];

      /* coordonnees du point i dans le repere camera	*/
      double rx = rd[0][0] * x + rd[0][1] * y + rd[0][2] * z + xc[0];
      double ry = rd[1][0] * x + rd[1][1] * y + rd[1][2] * z + xc[1];
      double rz = rd[2][0] * x + rd[2][1] * y + rd[2][2] * z + xc[2];

      /* derive des fonctions rx, ry et rz par rapport
       * a tt, u1, u2, u3.
       */
      double drxt = (si * u1 * u3 + co * u2) * z + (si * u1 * u2 - co * u3) * y + (si * u1 * u1 - si) * x;
      double drxu1 = mco * u3 * z + mco * u2 * y + 2 * mco * u1 * x;
      double drxu2 = si * z + mco * u1 * y;
      double drxu3 = mco * u1 * z - si * y;

      double dryt = (si * u2 * u3 - co * u1) * z + (si * u2 * u2 - si) * y + (co * u3 + si * u1 * u2) * x;
      double dryu1 = mco * u2 * x - si * z;
      double dryu2 = mco * u3 * z + 2 * mco * u2 * y + mco * u1 * x;
      double dryu3 = mco * u2 * z + si * x;

      double drzt = (si * u3 * u3 - si) * z + (si * u2 * u3 + co * u1) * y + (si * u1 * u3 - co * u2) * x;
      double drzu1 = si * y + mco * u3 * x;
      double drzu2 = mco * u3 * y - si * x;
      double drzu3 = 2 * mco * u3 * z + mco * u2 * y + mco * u1 * x;

      /* derive de la fonction representant le modele de la
       * camera (sans distortion) par rapport a tt, u1, u2 et u3.
       */
      double dxit = drxt / rz - rx * drzt / (rz * rz);

      double dyit = dryt / rz - ry * drzt / (rz * rz);

      double dxiu1 = drxu1 / rz - drzu1 * rx / (rz * rz);
      double dyiu1 = dryu1 / rz - drzu1 * ry / (rz * rz);

      double dxiu2 = drxu2 / rz - drzu2 * rx / (rz * rz);
      double dyiu2 = dryu2 / rz - drzu2 * ry / (rz * rz);

      double dxiu3 = drxu3 / rz - drzu3 * rx / (rz * rz);
      double dyiu3 = dryu3 / rz - drzu3 * ry / (rz * rz);

      /* calcul du jacobien : le jacobien represente la
       * derivee de la fonction representant le modele de la
       * camera par rapport aux parametres.
       */
      *MIJ(jac, 0, i, ldfjac) = 1 / rz;
      *MIJ(jac, 1, i, ldfjac) = 0.0;
      *MIJ(jac, 2, i, ldfjac) = -rx / (rz * rz);
      if (tt >= MINIMUM) {
        *MIJ(jac, 3, i, ldfjac) = u1 * dxit + (1 - u1 * u1) * dxiu1 / tt - u1 * u2 * dxiu2 / tt - u1 * u3 * dxiu3 / tt;
        *MIJ(jac, 4, i, ldfjac) = u2 * dxit - u1 * u2 * dxiu1 / tt + (1 - u2 * u2) * dxiu2 / tt - u2 * u3 * dxiu3 / tt;

        *MIJ(jac, 5, i, ldfjac) = u3 * dxit - u1 * u3 * dxiu1 / tt - u2 * u3 * dxiu2 / tt + (1 - u3 * u3) * dxiu3 / tt;
      } else {
        *MIJ(jac, 3, i, ldfjac) = 0.0;
        *MIJ(jac, 4, i, ldfjac) = 0.0;
        *MIJ(jac, 5, i, ldfjac) = 0.0;
      }
      *MIJ(jac, 0, npt + i, ldfjac) = 0.0;
      *MIJ(jac, 1, npt + i, ldfjac) = 1 / rz;
      *MIJ(jac, 2, npt + i, ldfjac) = -ry / (rz * rz);
      if (tt >= MINIMUM) {
        *MIJ(jac, 3, npt + i, ldfjac) =
            u1 * dyit + (1 - u1 * u1) * dyiu1 / tt - u1 * u2 * dyiu2 / tt - u1 * u3 * dyiu3 / tt;
        *MIJ(jac, 4, npt + i, ldfjac) =
            u2 * dyit - u1 * u2 * dyiu1 / tt + (1 - u2 * u2) * dyiu2 / tt - u2 * u3 * dyiu3 / tt;
        *MIJ(jac, 5, npt + i, ldfjac) =
            u3 * dyit - u1 * u3 * dyiu1 / tt - u2 * u3 * dyiu2 / tt + (1 - u3 * u3) * dyiu3 / tt;
      } else {
        *MIJ(jac, 3, npt + i, ldfjac) = 0.0;
        *MIJ(jac, 4, npt + i, ldfjac) = 0.0;
        *MIJ(jac, 5, npt + i, ldfjac) = 0.0;
      }
    }
  } /* fin else if iflag ==2	*/
}

/*!
\brief  Compute the pose using the Lowe non linear approach
it consider the minimization of a residual using
the levenberg marquartd approach.

The approach has been proposed by D.G Lowe in 1992 paper \cite Lowe92a.

*/
void vpPose::poseLowe(vpHomogeneousMatrix &cMo)
{
#if (DEBUG_LEVEL1)
  std::cout << "begin CCalcuvpPose::PoseLowe(...) " << std::endl;
#endif
  int n, m;   /* nombre d'elements dans la matrice jac */
  int lwa;    /* taille du vecteur wa */
  int ldfjac; /* taille maximum d'une ligne de jac */
  int info, ipvt[NBR_PAR];
  int tst_lmder;
  double f[2 * NBPTMAX], sol[NBR_PAR];
  double tol, jac[NBR_PAR][2 * NBPTMAX], wa[2 * NBPTMAX + 50];
  //  double	u[3];	/* vecteur de rotation */
  //  double	rd[3][3]; /* matrice de rotation */

  n = NBR_PAR;                                  /* nombres d'inconnues	*/
  m = (int)(2 * npt);                           /* nombres d'equations	*/
  lwa = 2 * NBPTMAX + 50;                       /* taille du vecteur de travail	*/
  ldfjac = 2 * NBPTMAX;                         /* nombre d'elements max sur une ligne	*/
  tol = std::numeric_limits<double>::epsilon(); /* critere d'arret	*/

  //  c = cam ;
  // for (i=0;i<3;i++)
  //   for (j=0;j<3;j++) rd[i][j] = cMo[i][j];
  //  mat_rot(rd,u);
  vpRotationMatrix cRo;
  cMo.extract(cRo);
  vpThetaUVector u(cRo);
  for (unsigned int i = 0; i < 3; i++) {
    sol[i] = cMo[i][3];
    sol[i + 3] = u[i];
  }

  vpPoint P;
  unsigned int i_ = 0;
  for (std::list<vpPoint>::const_iterator it = listP.begin(); it != listP.end(); ++it) {
    P = *it;
    XI[i_] = P.get_x(); //*cam.px + cam.xc ;
    YI[i_] = P.get_y(); //;*cam.py + cam.yc ;
    XO[i_] = P.get_oX();
    YO[i_] = P.get_oY();
    ZO[i_] = P.get_oZ();
    ++i_;
  }
  tst_lmder = lmder1(&fcn, m, n, sol, f, &jac[0][0], ldfjac, tol, &info, ipvt, lwa, wa);
  if (tst_lmder == -1) {
    std::cout << " in CCalculPose::PoseLowe(...) : ";
    std::cout << "pb de minimisation,  returns FATAL_ERROR";
    // return FATAL_ERROR ;
  }

  for (unsigned int i = 0; i < 3; i++)
    u[i] = sol[i + 3];

  for (unsigned int i = 0; i < 3; i++) {
    cMo[i][3] = sol[i];
    u[i] = sol[i + 3];
  }

  vpRotationMatrix rd(u);
  cMo.insert(rd);
//  rot_mat(u,rd);
//  for (i=0;i<3;i++) for (j=0;j<3;j++) cMo[i][j] = rd[i][j];

#if (DEBUG_LEVEL1)
  std::cout << "end CCalculPose::PoseLowe(...) " << std::endl;
#endif
  //  return OK ;
}

#undef MINI
#undef MINIMUM

#undef DEBUG_LEVEL1

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
