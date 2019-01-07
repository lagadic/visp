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
 * Le module "arit.c" contient les procedures arithmetiques.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include "vpArit.h"
#include "vpMy.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <visp3/core/vpMath.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*
 * La procedure "fprintf_matrix" affiche une matrice sur un fichier.
 * Entree :
 * fp		Fichier en sortie.
 * m		Matrice a ecrire.
 */
void fprintf_matrix(FILE *fp, Matrix m)
{
  int i;

  fprintf(fp, "(matrix\n");
  for (i = 0; i < 4; i++)
    fprintf(fp, "\t%.4f\t%.4f\t%.4f\t%.4f\n", m[i][0], m[i][1], m[i][2], m[i][3]);
  fprintf(fp, ")\n");
}

/*
 * La procedure "ident_matrix" initialise la matrice par la matrice identite.
 * Entree :
 * m		Matrice a initialiser.
 */
void ident_matrix(Matrix m)
{
  static Matrix identity = IDENTITY_MATRIX;

  // bcopy ((char *) identity, (char *) m, sizeof (Matrix));
  memmove((char *)m, (char *)identity, sizeof(Matrix));
  /*
   * Version moins rapide.
   *
   *	int	i, j;
   *
   *	for (i = 0; i < 4; i++)
   *	for (j = 0; j < 4; j++)
   *		m[i][j] = (i == j) ? 1.0 : 0.0;
   */
}

/*
 * La procedure "premult_matrix" pre multiplie la matrice par la seconde.
 * Entree :
 * a		Premiere matrice du produit a = b * a.
 * b		Seconde  matrice du produit.
 */
void premult_matrix(Matrix a, Matrix b)
{
  Matrix m;
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      m[i][j] = b[i][0] * a[0][j] + b[i][1] * a[1][j] + b[i][2] * a[2][j] + b[i][3] * a[3][j];
  // bcopy ((char *) m, (char *) a, sizeof (Matrix));
  memmove((char *)a, (char *)m, sizeof(Matrix));
}

/*
 * La procedure "premult3_matrix" premultiplie la matrice par une matrice 3x3.
 * Note : La procedure "premult3_matrix" optimise "premutl_matrix".
 * Entree :
 * a		Premiere matrice du produit a = b * a.
 * b		Seconde  matrice du produit 3x3.
 */
void premult3_matrix(Matrix a, Matrix b)
{
  Matrix m;
  int i, j;

  // bcopy ((char *) a, (char *) m, sizeof (Matrix));
  memmove((char *)m, (char *)a, sizeof(Matrix));
  for (i = 0; i < 3; i++)
    for (j = 0; j < 4; j++)
      a[i][j] = b[i][0] * m[0][j] + b[i][1] * m[1][j] + b[i][2] * m[2][j];
}

/*
 * La procedure "prescale_matrix" premultiplie la matrice par l'homothetie.
 * Entree :
 * m		Matrice a multiplier m = vp * m.
 * vp		Vecteur d'homothetie.
 */
void prescale_matrix(Matrix m, Vector *vp)
{
  int i;

  for (i = 0; i < 4; i++) {
    m[0][i] *= vp->x;
    m[1][i] *= vp->y;
    m[2][i] *= vp->z;
  }
}

/*
 * La procedure "pretrans_matrix" premultiplie la matrice par la translation.
 * Entree :
 * m		Matrice a multiplier m = vp * m.
 * vp		Vecteur de translation.
 */
void pretrans_matrix(Matrix m, Vector *vp)
{
  int i;

  for (i = 0; i < 4; i++)
    m[3][i] += vp->x * m[0][i] + vp->y * m[1][i] + vp->z * m[2][i];
}

/*
 * La procedure "postleft_matrix" postmultiplie la matrice
 * par une matrice gauche sur un des axes.
 * Entree :
 * m		Matrice a rendre gauche m = m * left.
 * axis		Axe de la matrice gauche 'x', 'y' ou 'z'.
 */
void postleft_matrix(Matrix m, char axis)
{

  int i;

  switch (axis) {
  case 'x':
    for (i = 0; i < 4; i++)
      m[i][0] = -m[i][0];
    break;
  case 'y':
    for (i = 0; i < 4; i++)
      m[i][1] = -m[i][1];
    break;
  case 'z':
    for (i = 0; i < 4; i++)
      m[i][2] = -m[i][2];
    break;
  default: {
    static char proc_name[] = "postleft_matrix";
    fprintf(stderr, "%s: axis unknown\n", proc_name);
    break;
  }
  }
}

/*
 * La procedure "postmult_matrix" post multiplie la matrice par la seconde.
 * Entree :
 * a		Premiere matrice du produit a = a * b.
 * b		Seconde  matrice du produit.
 */
void postmult_matrix(Matrix a, Matrix b)
{
  Matrix m;
  int i, j;

  for (i = 0; i < 4; i++)
    for (j = 0; j < 4; j++)
      m[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j];
  // bcopy ((char *) m, (char *) a, sizeof (Matrix));
  memmove((char *)a, (char *)m, sizeof(Matrix));
}

/*
 * La procedure "postmult3_matrix" postmultiplie la matrice par une matrice
 * 3x3. Note : La procedure "postmult3_matrix" optimise "postmutl_matrix".
 * Entree :
 * a		Premiere matrice du produit a = a * b.
 * b		Seconde  matrice du produit 3x3.
 */
void postmult3_matrix(Matrix a, Matrix b)
{
  Matrix m;
  int i, j;

  // bcopy ((char *) a, (char *) m, sizeof (Matrix));
  memmove((char *)m, (char *)a, sizeof(Matrix));
  for (i = 0; i < 4; i++)
    for (j = 0; j < 3; j++)
      a[i][j] = m[i][0] * b[0][j] + m[i][1] * b[1][j] + m[i][2] * b[2][j];
}

/*
 * La procedure "postscale_matrix" post multiplie la matrice par l'homothetie.
 * Entree :
 * m		Matrice a multiplier m = m * vp.
 * vp		Vecteur d'homothetie.
 */
void postscale_matrix(Matrix m, Vector *vp)
{
  int i;

  for (i = 0; i < 4; i++) {
    m[i][0] *= vp->x;
    m[i][1] *= vp->y;
    m[i][2] *= vp->z;
  }
}

/*
 * La procedure "posttrans_matrix" post mutiplie la matrice par la
 * translation. Entree : m		Matrice a multiplier m = m * vp. vp
 * Vecteur de translation.
 */
void posttrans_matrix(Matrix m, Vector *vp)
{
  int i;

  for (i = 0; i < 4; i++) {
    m[i][0] += m[i][3] * vp->x;
    m[i][1] += m[i][3] * vp->y;
    m[i][2] += m[i][3] * vp->z;
  }
}

/*
 * La procedure "transpose_matrix" transpose la matrice.
 * Entree :
 * m		Matrice a transposer.
 */
void transpose_matrix(Matrix m)
{
  unsigned int i, j;
  float t;

  for (i = 0; i < 4; i++)
    for (j = 0; j < i; j++)
      SWAP(m[i][j], m[j][i], t);
}

/*
 * La procedure "cosin_to_angle" calcule un angle a partir d'un cosinus
 * et d'un sinus.
 * Entree :
 * ca, sa	Cosinus et Sinus de l'angle.
 * Sortie :
 *		Angle en radians.
 */
float cosin_to_angle(float ca, float sa)
{
  float a; /* angle a calculer	*/

  if (FABS(ca) < M_EPSILON) {
    a = (sa > (float)0.0) ? (float)M_PI_2 : (float)(-M_PI_2);
  } else {
    a = (float)atan((double)(sa / ca));
    if (ca < (float)0.0)
      a += (sa > (float)0.0) ? (float)M_PI : (float)(-M_PI);
  }
  return (a);
}

/*
 * La procedure "cosin_to_lut" precalcule les tables des "cosinus" et "sinus".
 * Les tables possedent "2 ** level" entrees pour M_PI_2 radians.
 * Entree :
 * level	Niveau de decomposition.
 * coslut	Table pour la fonction "cosinus".
 * sinlut	Table pour la fonction "sinus".
 */
void cosin_to_lut(Index level, float *coslut, float *sinlut)
{
  int i;
  int i_pi_2 = TWO_POWER(level);
  int quad; /* quadrant courant	*/
  double a; /* angle    courant	*/
  double step = M_PI_2 / (double)i_pi_2;

  quad = 0;
  coslut[quad] = 1.0;
  sinlut[quad] = 0.0; /* 0	*/
  quad += i_pi_2;
  coslut[quad] = 0.0;
  sinlut[quad] = 1.0; /* PI/2	*/
  quad += i_pi_2;
  coslut[quad] = -1.0;
  sinlut[quad] = 0.0; /* PI 	*/
  quad += i_pi_2;
  coslut[quad] = 0.0;
  sinlut[quad] = -1.0; /* 3PI/2*/

  for (i = 1, a = step; i < i_pi_2; i++, a += step) {
    float ca = (float)cos(a);
    quad = 0;
    coslut[quad + i] = ca; /* cos(a)	*/
    quad += i_pi_2;
    sinlut[quad - i] = ca; /* sin(PI/2-a)	*/
    sinlut[quad + i] = ca; /* sin(PI/2+a)	*/
    quad += i_pi_2;
    coslut[quad - i] = -ca; /* cos(PI-a)	*/
    coslut[quad + i] = -ca; /* cos(PI+a)	*/
    quad += i_pi_2;
    sinlut[quad - i] = -ca; /* sin(3PI/2-a)	*/
    sinlut[quad + i] = -ca; /* sin(3PI/2+a)	*/
    quad += i_pi_2;
    coslut[quad - i] = ca; /* cos(2PI-a)	*/
  }
}

/*
 * La procedure "norm_vector" normalise le vecteur.
 * Si la norme est nulle la normalisation n'est pas effectuee.
 * Entree :
 * vp		Le vecteur a norme.
 * Sortie :
 * 		La norme du vecteur.
 */
float norm_vector(Vector *vp)
{
  float norm; /* norme du vecteur 	*/

  if ((norm = (float)sqrt((double)DOT_PRODUCT(*vp, *vp))) > M_EPSILON) {
    vp->x /= norm;
    vp->y /= norm;
    vp->z /= norm;
  } else {
    static char proc_name[] = "norm_vector";
    fprintf(stderr, "%s: nul vector\n", proc_name);
  }
  return (norm);
}

/*
 * La procedure "plane_norme" calcule le vecteur norme orthogonal au plan
 * defini par les 3 points.
 * Entree :
 * np		Le vecteur norme orthogonal au plan.
 * ap, bp, cp	Points formant un repere du plan.
 */
void plane_norme(Vector *np, Point3f *ap, Point3f *bp, Point3f *cp)
{
  Vector u, v;

  DIF_COORD3(u, *bp, *ap); /* base orthonorme (ap, u, v)	*/
  DIF_COORD3(v, *cp, *ap);
  norm_vector(&u);
  norm_vector(&v);
  CROSS_PRODUCT(*np, u, v);
}

/*
 * La procedure "point_matrix" deplace un point 3D dans un espace 4D.
 * Une matrice homogene 4x4 effectue le changement de repere.
 * Entree :
 * p4		Point   homogene resultat = p3 x m.
 * p3		Point   a deplacer.
 * m		Matrice de changement de repere.
 */
void point_matrix(Point4f *p4, Point3f *p3, Matrix m)
{
  float x = p3->x, y = p3->y, z = p3->z;

  p4->x = COORD3_COL(x, y, z, m, 0);
  p4->y = COORD3_COL(x, y, z, m, 1);
  p4->z = COORD3_COL(x, y, z, m, 2);
  p4->w = COORD3_COL(x, y, z, m, 3);
}

/*
 * La procedure "point_3D_3D" deplace un tableau de points 3D dans un espace
 * 3D. Une matrice 4x3 effectue le changement de repere. La quatrieme colonne
 * de la matrice vaut [0, 0, 0, 1] et n'est pas utilisee. Entree : ip
 * Tableau de points 3D a deplacer. size		Taille  du tableau
 * "ip". m		Matrice de changement de repere. Entree/Sortie : op
 * Tableau de points 3D resultat.
 */
void point_3D_3D(Point3f *ip, int size, Matrix m, Point3f *op)
{
  Point3f *pend = ip + size; /* borne de ip	*/

  for (; ip < pend; ip++, op++) {
    float x = ip->x;
    float y = ip->y;
    float z = ip->z;

    op->x = COORD3_COL(x, y, z, m, 0);
    op->y = COORD3_COL(x, y, z, m, 1);
    op->z = COORD3_COL(x, y, z, m, 2);
  }
}

/*
 * La procedure "point_3D_4D" deplace un tableau de points 3D dans un espace
 * 4D. Une matrice homogene 4x4 effectue le changement de repere. Entree : p3
 * Tableau de points 3D a deplacer. size		Taille  du tableau
 * "p3". m		Matrice de changement de repere. Entree/Sortie : p4
 * Tableau de points 4D resultat.
 */
void point_3D_4D(Point3f *p3, int size, Matrix m, Point4f *p4)
{
  Point3f *pend = p3 + size; /* borne de p3	*/

  for (; p3 < pend; p3++, p4++) {
    float x = p3->x;
    float y = p3->y;
    float z = p3->z;

    p4->x = COORD3_COL(x, y, z, m, 0);
    p4->y = COORD3_COL(x, y, z, m, 1);
    p4->z = COORD3_COL(x, y, z, m, 2);
    p4->w = COORD3_COL(x, y, z, m, 3);
  }
}

/*
 * La procedure "rotate_vector" transforme le vecteur
 * par la rotation de sens trigonometrique d'angle et d'axe donnes.
 * Entree :
 * vp		Vecteur a transformer.
 * a		Angle de rotation en degres.
 * axis		Vecteur directeur de l'axe de rotation.
 */
void rotate_vector(Vector *vp, float a, Vector *axis)
{
  Vector n, u, v, cross;
  float f;

  a *= (float)M_PI / (float)180.0; /* passage en radians		*/

  n = *axis; /* norme le vecteur directeur	*/
  norm_vector(&n);

  /*
   * Avant rotation, vp vaut :
   *   u + v
   * Apres rotation, vp vaut :
   *   u + cos(a) * v + sin(a) * (n^vp)
   * = u + cos(a) * v + sin(a) * (n^v)
   * avec u = (vp.n) * n, v = vp-u;
   * ou "u" est la projection de "vp" sur l'axe "axis",
   * et "v" est la composante de "vp" perpendiculaire a "axis".
   */
  f = DOT_PRODUCT(*vp, n);
  u = n;
  MUL_COORD3(u, f, f, f); /* (vp.n) * n		*/

  DIF_COORD3(v, *vp, u); /* calcule "v"		*/

  f = (float)cos((double)a);
  MUL_COORD3(v, f, f, f); /* v * cos(a)		*/

  CROSS_PRODUCT(cross, n, *vp);
  f = (float)sin((double)a);
  MUL_COORD3(cross, f, f, f); /* (n^v) * sin(a)	*/

  SET_COORD3(*vp, u.x + v.x + cross.x, u.y + v.y + cross.y, u.z + v.z + cross.z);
}

/*
 * La procedure "upright_vector" calcule un vecteur perpendiculaire.
 * Les vecteurs ont un produit scalaire nul.
 * Entree :
 * vp		Vecteur origine.
 * Entree/Sortie :
 * up		Vecteur perpendiculaire a vp.
 */
void upright_vector(Vector *vp, Vector *up)
{
  if (FABS(vp->z) > M_EPSILON) { /* x et y sont fixes	*/
    up->z = -(vp->x + vp->y) / vp->z;
    up->x = up->y = 1.0;
  } else if (FABS(vp->y) > M_EPSILON) { /* x et z sont fixes	*/
    up->y = -(vp->x + vp->z) / vp->y;
    up->x = up->z = 1.0;
  } else if (FABS(vp->x) > M_EPSILON) { /* y et z sont fixes	*/
    up->x = -(vp->y + vp->z) / vp->x;
    up->y = up->z = 1.0;
  } else {
    static char proc_name[] = "upright_vector";
    up->x = up->y = up->z = 0.0;
    fprintf(stderr, "%s: nul vector\n", proc_name);
    return;
  }
}

/*
 * La procedure "Matrix_to_Position" initialise la position par la matrice.
 * Si M est la matrice, et P la position : M = R.Sid.T, P = (R,Sid,T).
 * On suppose que la matrice de rotation 3x3 de M est unitaire.
 * Entree :
 * m		Matrice de rotation et de translation.
 * pp		Position a initialiser.
 */
void Matrix_to_Position(Matrix m, AritPosition *pp)
{
  Matrix_to_Rotate(m, &pp->rotate);
  SET_COORD3(pp->scale, 1.0, 1.0, 1.0);
  SET_COORD3(pp->translate, m[3][0], m[3][1], m[3][2]);
}

/*
 * La procedure "Matrix_to_Rotate" initialise la rotation par la matrice.
 * Si M est la matrice, si R est la matrice de rotation :
 *
 *		 	| m00	m01	m02	0 |
 * M = Rx.Ry.Rz =	| m10	m11	m12	0 |
 *		 	| m20	m21	m22	0 |
 *		 	| 0	0	0	1 |
 *
 * et	m00 = cy.cz		m01 = cy.sz		m02 = -sy
 *	m10 = sx.sy.cz-cx.sz 	m11 = sx.sy.sz+cx.cz	m12 = sx.cy
 *	m20 = cx.sy.cz+sx.sz	m21 = cx.sy.sz-sx.cz	m22 = cx.cy
 * avec	ci = cos Oi et si = sin Oi.
 *
 * R = Rx.Ry.Rz
 * Rx rotation autour de Ox d'angle O1
 * Ry rotation autour de Oy d'angle O2
 * Rz rotation autour de Oz d'angle O3
 *
 * Singularite : si |ry| == 90 degres alors rz = 0,
 * 		 soit une rotation d'axe 0z et d'angle "rx + rz".
 *
 * Entree :
 * m		Matrice contenant la composition des rotations.
 * vp		Rotations par rapport aux axes d'un repere droit en degres.
 */
void Matrix_to_Rotate(Matrix m, Vector *vp)
{
  float sy = -m[0][2];
  float cy = (float)sqrt(1.0 - (double)(sy * sy));
  float cx, sx;

  if (FABS(cy) > M_EPSILON) {
    float sz = m[0][1] / cy;
    float cz = m[0][0] / cy;

    sx = m[1][2] / cy;
    cx = m[2][2] / cy;

    SET_COORD3(*vp, cosin_to_angle(cx, sx), cosin_to_angle(cy, sy), cosin_to_angle(cz, sz));
  } else { /* RZ = 0 =>  Ry = +/- 90 degres	*/
    sx = m[1][1];
    cx = -m[2][1];

    SET_COORD3(*vp, cosin_to_angle(cx, sx), (sy > (float)0.0) ? (float)M_PI_2 : (float)(-M_PI_2), (float)0.0);
  }
  vp->x *= (float)180.0 / (float)M_PI; /* passage en degres	*/
  vp->y *= (float)180.0 / (float)M_PI;
  vp->z *= (float)180.0 / (float)M_PI;
}

/*
 * La procedure "Position_to_Matrix" initialise la matrice par la position.
 * Matrice resultat : M = Sx.Sy.Sz.Rx.Ry.Rz.Tx.Ty.Tz
 * Entree :
 * pp		Position de reference.
 * m		Matrice a initialiser.
 */
void Position_to_Matrix(AritPosition *pp, Matrix m)
{
  Rotate_to_Matrix(&pp->rotate, m); /* rotation	*/
  prescale_matrix(m, &pp->scale);   /* homothetie	*/
  m[3][0] = pp->translate.x;        /* translation	*/
  m[3][1] = pp->translate.y;
  m[3][2] = pp->translate.z;
}

/*
 * La procedure "Rotate_to_Matrix" initialise la matrice par la rotation.
 *
 *		 	| m00	m01	m02	0 |
 * M = Rx.Ry.Rz =	| m10	m11	m12	0 |
 *		 	| m20	m21	m22	0 |
 *		 	| 0	0	0	1 |
 *
 * Rx rotation autour de Ox d'angle O1
 * Ry rotation autour de Oy d'angle O2
 * Rz rotation autour de Oz d'angle O3
 * et	m00 = cy.cz		m01 = cy.sz		m02 = -sy
 *	m10 = sx.sy.cz-cx.sz 	m11 = sx.sy.sz+cx.cz	m12 = sx.cy
 *	m20 = cx.sy.cz+sx.sz	m21 = cx.sy.sz-sx.cz	m22 = cx.cy
 * avec	ci = cos Oi et si = sin Oi.
 *
 * Entree :
 * vp		Rotations par rapport aux axes d'un repere droit en degres.
 * m		Matrice a initialiser.
 */
void Rotate_to_Matrix(Vector *vp, Matrix m)
{
  float rx = vp->x * (float)M_PI / (float)180.0, /* passage en radians	*/
      ry = vp->y * (float)M_PI / (float)180.0, rz = vp->z * (float)M_PI / (float)180.0;
  float cx = (float)cos((double)rx), sx = (float)sin((double)rx), cy = (float)cos((double)ry),
        sy = (float)sin((double)ry), cz = (float)cos((double)rz), sz = (float)sin((double)rz);

  m[0][0] = cy * cz;
  m[1][0] = (sx * sy * cz) - (cx * sz);
  m[2][0] = (cx * sy * cz) + (sx * sz);

  m[0][1] = cy * sz;
  m[1][1] = (sx * sy * sz) + (cx * cz);
  m[2][1] = (cx * sy * sz) - (sx * cz);

  m[0][2] = -sy;
  m[1][2] = sx * cy;
  m[2][2] = cx * cy;

  m[0][3] = m[1][3] = m[2][3] = 0.0;
  m[3][0] = m[3][1] = m[3][2] = 0.0;
  m[3][3] = 1.0;
}

/*
 * La procedure "Rotaxis_to_Matrix" initialise la matrice par la rotation
 * d'angle et d'axe donnes.
 * Si M est la matrice, O l'angle et N le vecteur directeur de l'axe :
 *
 *	M = cos(O) Id3 + (1 - cosO) Nt N + sinO N~
 *
 *	| NxNxverO+  cosO NxNyverO+NzsinO NxNzverO-NxsinO 0 |
 * M =	| NxNyverO-NzsinO NyNyverO+  cosO NyNzverO+NxsinO 0 |
 *	| NxNzverO+NysinO NyNzverO-NxsinO NzNzverO+  cosO 0 |
 *	| 0		  0		  0		  1 |
 *
 *	O	angle de rotation.
 *	N	Vecteur directeur norme de l'axe de rotation.
 *	Nt	Vecteur transpose.
 *	N~	| 0	 Nz	-Ny|
 *		|-Nz	 0	 Nx|
 *		| Ny	-Nx	 0 |
 * Entree :
 * a		Angle de rotation en degres.
 * axis		Vecteur directeur de l'axe de la rotation.
 * m		Matrice a initialiser.
 */
void Rotaxis_to_Matrix(float a, Vector *axis, Matrix m)
{
  float cosa;
  float sina;
  float vera;   /* 1 - cosa	*/
  Vector n;     /* vecteur norme*/
  Vector conv;  /* verO n	*/
  Vector tilde; /* sinO n	*/

  a *= (float)M_PI / (float)180.0; /* passage en radians	*/

  cosa = (float)cos((double)a);
  sina = (float)sin((double)a);
  vera = (float)1.0 - cosa;

  n = *axis; /* norme le vecteur directeur	*/
  norm_vector(&n);
  tilde = conv = n;
  MUL_COORD3(conv, vera, vera, vera);
  MUL_COORD3(tilde, sina, sina, sina);

  m[0][0] = conv.x * n.x + cosa;
  m[0][1] = conv.x * n.y + tilde.z;
  m[0][2] = conv.x * n.z - tilde.y;

  m[1][0] = conv.y * n.x - tilde.z;
  m[1][1] = conv.y * n.y + cosa;
  m[1][2] = conv.y * n.z + tilde.x;

  m[2][0] = conv.z * n.x + tilde.y;
  m[2][1] = conv.z * n.y - tilde.x;
  m[2][2] = conv.z * n.z + cosa;

  m[0][3] = m[2][3] = m[1][3] = 0.0;
  m[3][0] = m[3][1] = m[3][2] = 0.0;
  m[3][3] = 1.0;
}

/*
 * La procedure "Rotrans_to_Matrix" initialise la matrice par la rotation
 * et de la translation.
 * Entree :
 * rp		Vecteur des angles de rotation en degres.
 * tp		Vecteur des coordonnees de translation.
 * m		Matrice a initialiser.
 */
void Rotrans_to_Matrix(Vector *rp, Vector *tp, Matrix m)
{
  Rotate_to_Matrix(rp, m); /* matrice de rotation		*/
  m[3][0] = tp->x;         /* matrice de translation	*/
  m[3][1] = tp->y;
  m[3][2] = tp->z;
}

/*
 * La procedure "Scale_to_Matrix" initialise la matrice par l'homothetie.
 * Entree :
 * vp		Vecteur des coordonnees d'homothetie.
 * m		Matrice a initialiser.
 */
void Scale_to_Matrix(Vector *vp, Matrix m)
{
  ident_matrix(m);
  m[0][0] = vp->x;
  m[1][1] = vp->y;
  m[2][2] = vp->z;
}

/*
 * La procedure "Translate_to_Matrix" initialise la matrice par la
 * translation. Entree : vp		Vecteur des coordonnees de
 * translation. m		Matrice a initialiser.
 */
void Translate_to_Matrix(Vector *vp, Matrix m)
{
  ident_matrix(m);
  m[3][0] = vp->x;
  m[3][1] = vp->y;
  m[3][2] = vp->z;
}

#endif
