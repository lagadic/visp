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
 * Le module "projection.c" contient les procedures de calcul
 * des matrices de projection perspective et parallele.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include "vpProjection.h"
#include <math.h>
#include <stdio.h>

/*
 * La procedure "View_to_Matrix" constuit la matrice homogene de projection
 * a partir des parametres de la prise de vue.
 * Entree :
 * vp		Parametres de la prise de vue.
 * m		Matrice homogene a construire.
 */
void View_to_Matrix(View_parameters *vp, Matrix m)
{
  static char proc_name[] = "View_to_Matrix";

  switch (vp->type) {
  case PARALLEL:
    set_parallel(vp, m);
    break;
  case PERSPECTIVE:
    set_perspective(vp, m);
    break;
  default:
    fprintf(stderr, "%s: bad view type\n", proc_name);
    set_perspective(vp, m);
    break;
  }
}

/*
 * La procedure "set_zy" initialise la matrice par une composition :
 * 	1 - aligne le premier vecteur sur l'axe Z dans le sens negatif.
 * 	2 - aligne la projection du second vecteur sur l'axe Y.
 * Entree :
 * m		Matrice a initialiser.
 * v0		Premier vecteur.
 * v1		Second  vecteur.
 */
static void set_zy(Matrix m, Vector *v0, Vector *v1)
{
  Vector rx, ry, rz;

  SET_COORD3(rz, -v0->x, -v0->y, -v0->z);
  CROSS_PRODUCT(rx, *v0, *v1);
  norm_vector(&rx);
  norm_vector(&rz);
  CROSS_PRODUCT(ry, rz, rx); /* ry est norme	*/

  m[0][0] = rx.x;
  m[0][1] = ry.x;
  m[0][2] = rz.x;
  m[0][3] = 0.0;
  m[1][0] = rx.y;
  m[1][1] = ry.y;
  m[1][2] = rz.y;
  m[1][3] = 0.0;
  m[2][0] = rx.z;
  m[2][1] = ry.z;
  m[2][2] = rz.z;
  m[2][3] = 0.0;
  m[3][0] = 0.0;
  m[3][1] = 0.0;
  m[3][2] = 0.0;
  m[3][3] = 1.0;
}

/*
 * La procedure "set_parallel" iniatilise la matrice de projection
 * parallel "wc" par les parametres de visualisation "vp".
 * Pour plus de renseignements :
 *	"Fundamentals of Interactive Computer Graphics"
 *	J.D. FOLEY, A. VAN DAM, Addison-Wesley. 1982, pp 285-290.
 * Entree :
 * vp		Parametres de visualisation.
 * wc		Matrice a initialiser.
 */
void set_parallel(View_parameters *vp, Matrix wc)
{
  Matrix m = IDENTITY_MATRIX;
  Point3f cop;
  Point4f doprim;
  Vector dop, v;

  /*
   * 1 : Translation du point de reference VRP a l'origine.
   */
  SET_COORD3(v, -vp->vrp.x, -vp->vrp.y, -vp->vrp.z);
  Translate_to_Matrix(&v, wc);
  /*
   * 2 : Rotation pour rendre VPN parallele a l'axe des Z negatifs.
   * 3 : Rotation pour rendre la projection de VUP sur le plan de
   *     projection parallele a l'axe Y.
   */
  set_zy(m, &vp->vpn, &vp->vup);
  /*
   * 4 : Passer d'un repere droit (absolu) a un repere gauche (vision).
   */
  postleft_matrix(m, 'z');
  postmult_matrix(wc, m);
  /*
   * 5 : Alignement de l'axe central du volume de vision sur l'axe Z.
   *     COP = DOP = Direction of Projection.
   *     DOPRIM = DOP * R_TRL
   * Pas de translation dans la matrice R_TRL pour la transformation
   * du vecteur DOP.
   */
  SET_COORD3(dop, vp->vrp.x - vp->cop.x, vp->vrp.y - vp->cop.y, vp->vrp.z - vp->cop.z);
  norm_vector(&dop);
  SET_COORD3(cop, dop.x, dop.y, dop.z);
  point_matrix(&doprim, &cop, m);
  ident_matrix(m);
  m[2][0] = -doprim.x / doprim.z;
  m[2][1] = -doprim.y / doprim.z;
  postmult_matrix(wc, m);
  /*
   * 6 : Translation et Mise a l'echelle de la pyramide.
   * Remarque : contrairement a la reference qui donne
   *	0 < x < 1, 0 < y < 1, 0 < z < 1
   * je prefere, afin de rester coherent avec la projection perspective,
   *	-1 < x < 1, -1 < y < 1, 0 < z < 1 (w = 1)
   */
  SET_COORD3(v, (float)(-(vp->vwd.umax + vp->vwd.umin) / 2.0), (float)(-(vp->vwd.vmax + vp->vwd.vmin) / 2.0),
             (float)(-vp->depth.front));
  posttrans_matrix(wc, &v);
  SET_COORD3(v, (float)(2.0 / (vp->vwd.umax - vp->vwd.umin)), (float)(2.0 / (vp->vwd.vmax - vp->vwd.vmin)),
             (float)(1.0 / (vp->depth.back - vp->depth.front)));
  postscale_matrix(wc, &v);
}

/*
 * La procedure "set_perspective" iniatilise la matrice de projection
 * perspective "wc" par les parametres de visualisation "vp".
 * Pour plus de renseignements :
 *	"Fundamentals of Interactive Computer Graphics"
 *	J.D. FOLEY, A. VAN DAM, Addison-Wesley. 1982, pp 290-302.
 * Entree :
 * vp		Parametres de visualisation.
 * wc		Matrice a initialiser.
 */
void set_perspective(View_parameters *vp, Matrix wc)
{
  Matrix m = IDENTITY_MATRIX;
  Point4f vrprim, cw;
  float zmin;
  Vector v;

  /*
   * 1 : Translation du centre de projection COP a l'origine.
   */
  SET_COORD3(v, -vp->cop.x, -vp->cop.y, -vp->cop.z);
  Translate_to_Matrix(&v, wc);
  /*
   * 2 : Rotation pour rendre VPN parallele a l'axe des Z negatifs.
   * 3 : Rotation pour rendre la projection de VUP sur le plan de
   *     projection parallele a l'axe Y.
   */
  set_zy(m, &vp->vpn, &vp->vup);
  postmult_matrix(wc, m);
  /*
   * 4 : Passer d'un repere droit (absolu) a un repere gauche (vision).
   */
  postleft_matrix(wc, 'z');
  /*
   * 5 : Alignement de l'axe central du volume de vision sur l'axe Z.
   */
  point_matrix(&vrprim, &vp->vrp, wc);
  cw.x = (float)(vrprim.x + (vp->vwd.umin + vp->vwd.umax) / 2.0);
  cw.y = (float)(vrprim.y + (vp->vwd.vmin + vp->vwd.vmax) / 2.0);
  cw.z = (float)(vrprim.z);
  ident_matrix(m);
  m[2][0] = -cw.x / cw.z;
  m[2][1] = -cw.y / cw.z;
  postmult_matrix(wc, m);
  /*
   * 6 : Mise a l'echelle de la pyramide.
   */
  SET_COORD3(v, (float)((2.0 * vrprim.z) / ((vp->vwd.umax - vp->vwd.umin) * (vrprim.z + vp->depth.back))),
             (float)((2.0 * vrprim.z) / ((vp->vwd.vmax - vp->vwd.vmin) * (vrprim.z + vp->depth.back))),
             (float)(1.0 / (vrprim.z + vp->depth.back)));
  postscale_matrix(wc, &v);
  /*
   * 7 : Transformation perspective.
   */
  zmin = (vrprim.z + vp->depth.front) / (vrprim.z + vp->depth.back);
  ident_matrix(m);
  m[2][2] = (float)(1.0 / (1.0 - zmin));
  m[2][3] = 1.0;
  m[3][2] = (float)(-zmin / (1.0 - zmin));
  m[3][3] = 0.0;
  postmult_matrix(wc, m);
}

#endif
