/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Le module contient les procedures arithmetiques.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define	ADD_COORD2(r,a,b)	{ (r).x = (a).x + (b).x;\
				  (r).y = (a).y + (b).y; }

#define	ADD_COORD3(r,a,b)	{ (r).x = (a).x + (b).x;\
				  (r).y = (a).y + (b).y;\
				  (r).z = (a).z + (b).z; }

#define	INC_COORD2(r,a)		{ (r).x += (a).x; (r).y += (a).y; }

#define	INC_COORD3(r,a)		{ (r).x += (a).x; (r).y += (a).y;\
				  (r).z += (a).z; }

#define	CROSS_PRODUCT(r,a,b)	{ (r).x = (a).y * (b).z - (a).z * (b).y;\
				  (r).y = (a).z * (b).x - (a).x * (b).z;\
				  (r).z = (a).x * (b).y - (a).y * (b).x; }

#define	DIF_COORD2(r,a,b)	{ (r).x = (a).x - (b).x;\
				  (r).y = (a).y - (b).y; }

#define	DIF_COORD3(r,a,b)	{ (r).x = (a).x - (b).x;\
				  (r).y = (a).y - (b).y;\
				  (r).z = (a).z - (b).z; }

#define	DOT_PRODUCT(a,b)	( ((a).x * (b).x) +\
				  ((a).y * (b).y) +\
				  ((a).z * (b).z) )

#define	LENGTH3(a)		(sqrt((double) DOT_PRODUCT((a),(a))))

#define	MID_COORD3(r,a,b)	{ (r).x = ((a).x + (b).x) / 2.0;\
				  (r).y = ((a).y + (b).y) / 2.0;\
				  (r).z = ((a).z + (b).z) / 2.0; }

#define	MUL_COORD3(r,a,b,c)	{ (r).x *= (a); (r).y *= (b); (r).z *= (c); }

#define	PAR_COORD3(r,t,a,b)	{ (r).x = ((b).x - (a).x) * (t) + (a).x;\
				  (r).y = ((b).y - (a).y) * (t) + (a).y;\
				  (r).z = ((b).z - (a).z) * (t) + (a).z; }

#define	SET_COORD2(r,a,b)	{ (r).x = (a); (r).y = (b); }

#define	SET_COORD3(r,a,b,c)	{ (r).x = (a); (r).y = (b); (r).z = (c); }

#define	SUB_COORD2(r,a)		{ (r).x -= (a).x; (r).y -= (a).y; }

#define	SUB_COORD3(r,a)		{ (r).x -= (a).x; (r).y -= (a).y;\
				  (r).z -= (a).z; }

#define	COORD3_COL(x,y,z,m,i)	( ((x) * (m)[0][i]) +\
				  ((y) * (m)[1][i]) +\
				  ((z) * (m)[2][i]) +\
				  	 (m)[3][i] )

#define	COORD4_COL(x,y,z,w,m,i)	( ((x) * (m)[0][i]) +\
				  ((y) * (m)[1][i]) +\
				  ((z) * (m)[2][i]) +\
				  ((w) * (m)[3][i]) )

#define	M_POLY1(x,a,b)		((a) * (x) +  (b))
#define	M_POLY2(x,a,b,c)	(M_POLY1((x),(a),(b)) * (x) + (c))
#define	M_POLY3(x,a,b,c,d)	(M_POLY2((x),(a),(b),(c)) * (x) + (d))


VISP_EXPORT typedef	struct	{
	int	x, y;
} Point2i;

typedef	struct	{
	short	x, y;
} Point2s;

typedef	struct	{
	int	x, y, z;
} Point3i;

typedef	struct	{
	float	x, y, z;
} Point3f;

typedef	struct	{
	float	x,y,z,w;
} Point4f;

typedef	struct	{	
	float	x,y,z;
} Vector;

/*
 *				MATRIX
 *				______
 *
 * Matrice homogene ou non.
 *				|   Rotation  | 0 |
 * Matrice non homogene = 	|     3x3     | 0 |
 *				|-------------| 0 |
 *				| Translation | 1 |
 */
typedef	float	Matrix[4][4];

#define	IDENTITY_MATRIX		{	{1.0,	0.0,	0.0,	0.0},\
					{0.0,	1.0,	0.0,	0.0},\
					{0.0,	0.0,	1.0,	0.0},\
					{0.0,	0.0,	0.0,	1.0}	}


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
typedef	struct	{
	Vector		rotate;		/* vecteur rotation	*/
	Vector		scale;		/* vecteur homothetie	*/
	Vector		translate;	/* vecteur translation	*/
} Position;

#define	IDENTITY_ROTATE		{	0.0,	0.0,	0.0	}	
#define	IDENTITY_SCALE		{	1.0,	1.0,	1.0	}
#define	IDENTITY_TRANSLATE	{	0.0,	0.0,	0.0	}

#define	IDENTITY_POSITION	{	IDENTITY_ROTATE,\
					IDENTITY_SCALE,\
					IDENTITY_TRANSLATE	}

extern	void	fprintf_matrix ();
extern	void	ident_matrix ();
extern	void	premult_matrix ();
extern	void	premult3_matrix ();
extern	void	prescale_matrix ();
extern	void	pretrans_matrix ();
extern	void	postleft_matrix (Matrix m, char axis);
extern	void	postmult_matrix (Matrix a, Matrix b);
extern	void	postmult3_matrix ();
extern	void	postscale_matrix ();
extern	void	posttrans_matrix ();
extern	void	transpose_matrix ();

extern	float	cosin_to_angle (float ca, float sa);
extern	void	cosin_lut ();
extern	float	norm_vector ();
extern	void	point_matrix ();
extern	void	plane_norme ();
extern	void	point_3D_3D ();
extern	void	point_3D_4D (Point3f *p3, int size, Matrix m, Point4f *p4);
extern	void	rotate_vector (Vector *vp, float a, Vector *axis);
extern	void	upright_vector ();

extern	void	Matrix_to_Position ();
extern	void	Matrix_to_Rotate ();
extern	void	Position_to_Matrix ();
extern	void	Rotate_to_Matrix ();
extern	void	Rotaxis_to_Matrix (float a, Vector *axis, Matrix m);
extern	void	Rotrans_to_Matrix ();
extern	void	Scale_to_Matrix ();
extern	void	Translate_to_Matrix ();

extern void fscanf_Point3f (Point3f *pp);
extern void fscanf_Vector (Vector *vp);
#endif
