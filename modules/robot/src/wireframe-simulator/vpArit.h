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
 * Le module contient les procedures arithmetiques.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/
#ifndef vpArit_h
#define vpArit_h

#include <stdio.h>
#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpWireFrameSimulatorTypes.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define ADD_COORD2(r, a, b)                                                                                            \
  {                                                                                                                    \
    (r).x = (a).x + (b).x;                                                                                             \
    (r).y = (a).y + (b).y;                                                                                             \
  }

#define ADD_COORD3(r, a, b)                                                                                            \
  {                                                                                                                    \
    (r).x = (a).x + (b).x;                                                                                             \
    (r).y = (a).y + (b).y;                                                                                             \
    (r).z = (a).z + (b).z;                                                                                             \
  }

#define INC_COORD2(r, a)                                                                                               \
  {                                                                                                                    \
    (r).x += (a).x;                                                                                                    \
    (r).y += (a).y;                                                                                                    \
  }

#define INC_COORD3(r, a)                                                                                               \
  {                                                                                                                    \
    (r).x += (a).x;                                                                                                    \
    (r).y += (a).y;                                                                                                    \
    (r).z += (a).z;                                                                                                    \
  }

#define CROSS_PRODUCT(r, a, b)                                                                                         \
  {                                                                                                                    \
    (r).x = (a).y * (b).z - (a).z * (b).y;                                                                             \
    (r).y = (a).z * (b).x - (a).x * (b).z;                                                                             \
    (r).z = (a).x * (b).y - (a).y * (b).x;                                                                             \
  }

#define DIF_COORD2(r, a, b)                                                                                            \
  {                                                                                                                    \
    (r).x = (a).x - (b).x;                                                                                             \
    (r).y = (a).y - (b).y;                                                                                             \
  }

#define DIF_COORD3(r, a, b)                                                                                            \
  {                                                                                                                    \
    (r).x = (a).x - (b).x;                                                                                             \
    (r).y = (a).y - (b).y;                                                                                             \
    (r).z = (a).z - (b).z;                                                                                             \
  }

#define DOT_PRODUCT(a, b) (((a).x * (b).x) + ((a).y * (b).y) + ((a).z * (b).z))

#define LENGTH3(a) (sqrt((double)DOT_PRODUCT((a), (a))))

#define MID_COORD3(r, a, b)                                                                                            \
  {                                                                                                                    \
    (r).x = ((a).x + (b).x) / 2.0;                                                                                     \
    (r).y = ((a).y + (b).y) / 2.0;                                                                                     \
    (r).z = ((a).z + (b).z) / 2.0;                                                                                     \
  }

#define MUL_COORD3(r, a, b, c)                                                                                         \
  {                                                                                                                    \
    (r).x *= (a);                                                                                                      \
    (r).y *= (b);                                                                                                      \
    (r).z *= (c);                                                                                                      \
  }

#define PAR_COORD3(r, t, a, b)                                                                                         \
  {                                                                                                                    \
    (r).x = ((b).x - (a).x) * (t) + (a).x;                                                                             \
    (r).y = ((b).y - (a).y) * (t) + (a).y;                                                                             \
    (r).z = ((b).z - (a).z) * (t) + (a).z;                                                                             \
  }

#define SET_COORD2(r, a, b)                                                                                            \
  {                                                                                                                    \
    (r).x = (a);                                                                                                       \
    (r).y = (b);                                                                                                       \
  }

#define SET_COORD3(r, a, b, c)                                                                                         \
  {                                                                                                                    \
    (r).x = (a);                                                                                                       \
    (r).y = (b);                                                                                                       \
    (r).z = (c);                                                                                                       \
  }

#define SUB_COORD2(r, a)                                                                                               \
  {                                                                                                                    \
    (r).x -= (a).x;                                                                                                    \
    (r).y -= (a).y;                                                                                                    \
  }

#define SUB_COORD3(r, a)                                                                                               \
  {                                                                                                                    \
    (r).x -= (a).x;                                                                                                    \
    (r).y -= (a).y;                                                                                                    \
    (r).z -= (a).z;                                                                                                    \
  }

#define COORD3_COL(x, y, z, m, i) (((x) * (m)[0][i]) + ((y) * (m)[1][i]) + ((z) * (m)[2][i]) + (m)[3][i])

#define COORD4_COL(x, y, z, w, m, i) (((x) * (m)[0][i]) + ((y) * (m)[1][i]) + ((z) * (m)[2][i]) + ((w) * (m)[3][i]))

#define M_POLY1(x, a, b) ((a) * (x) + (b))
#define M_POLY2(x, a, b, c) (M_POLY1((x), (a), (b)) * (x) + (c))
#define M_POLY3(x, a, b, c, d) (M_POLY2((x), (a), (b), (c)) * (x) + (d))

typedef struct {
  int x, y;
} Point2i;

typedef struct {
  short x, y;
} Point2s;

typedef struct {
  int x, y, z;
} Point3i;

typedef struct {
  float x, y, z, w;
} Point4f;

typedef struct {
  float x, y, z;
} Vector;

#define IDENTITY_MATRIX                                                                                                \
  {                                                                                                                    \
    {1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, 1.0, 0.0}, { 0.0, 0.0, 0.0, 1.0 }                           \
  }

/*
 * 				POSITION
 *				________
 *
 * La structure "Position" definit le positionnement d'un objet.
 * Matrice de positionnement = R.S.T
 * avec R = Rx.Ry.Rz	Matrice de rotation autour des axes (Ox,Oy,Oz),
 *			Les angles sont donnes en degres;
 *	S = Sx.Sy.Sz	Matrice d'homothetie sur les axes;
 *	T = Tx.Ty.Tz	Matrice de translation sur les axes.
 */
typedef struct {
  Vector rotate;    /* vecteur rotation	*/
  Vector scale;     /* vecteur homothetie	*/
  Vector translate; /* vecteur translation	*/
} AritPosition;

#define IDENTITY_ROTATE                                                                                                \
  {                                                                                                                    \
    0.0, 0.0, 0.0                                                                                                      \
  }
#define IDENTITY_SCALE                                                                                                 \
  {                                                                                                                    \
    1.0, 1.0, 1.0                                                                                                      \
  }
#define IDENTITY_TRANSLATE                                                                                             \
  {                                                                                                                    \
    0.0, 0.0, 0.0                                                                                                      \
  }

#define IDENTITY_POSITION                                                                                              \
  {                                                                                                                    \
    IDENTITY_ROTATE, IDENTITY_SCALE, IDENTITY_TRANSLATE                                                                \
  }

void fprintf_matrix(FILE *fp, Matrix m);
void ident_matrix(Matrix m);
void premult_matrix(Matrix a, Matrix b);
void premult3_matrix(Matrix a, Matrix b);
void prescale_matrix(Matrix m, Vector *vp);
void pretrans_matrix(Matrix m, Vector *vp);
void postleft_matrix(Matrix m, char axis);
void postmult_matrix(Matrix a, Matrix b);
void postmult3_matrix(Matrix a, Matrix b);
void postscale_matrix(Matrix m, Vector *vp);
void posttrans_matrix(Matrix m, Vector *vp);
void transpose_matrix(Matrix m);

float cosin_to_angle(float ca, float sa);
float norm_vector(Vector *vp);
void point_matrix(Point4f *p4, Point3f *p3, Matrix m);
void plane_norme(Vector *np, Point3f *ap, Point3f *bp, Point3f *cp);
void point_3D_3D(Point3f *ip, int size, Matrix m, Point3f *op);
void point_3D_4D(Point3f *p3, int size, Matrix m, Point4f *p4);
void rotate_vector(Vector *vp, float a, Vector *axis);
void upright_vector(Vector *vp, Vector *up);

void Matrix_to_Position(Matrix m, AritPosition *pp);
void Matrix_to_Rotate(Matrix m, Vector *vp);
void Position_to_Matrix(AritPosition *pp, Matrix m);
void Rotate_to_Matrix(Vector *vp, Matrix m);
void Rotaxis_to_Matrix(float a, Vector *axis, Matrix m);
void Rotrans_to_Matrix(Vector *rp, Vector *tp, Matrix m);
void Scale_to_Matrix(Vector *vp, Matrix m);
void Translate_to_Matrix(Vector *vp, Matrix m);

void fscanf_Point3f(Point3f *pp);
void fscanf_Vector(Vector *vp);

#endif
#endif
