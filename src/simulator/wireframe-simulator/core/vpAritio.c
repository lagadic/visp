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
 * Le module "aritio.c" contient les procedures d'entree/sortie
 *		  des types definis dans le module "arit.h".
 *		  Les entrees non specifiees sont effectuees
 *		  sur le fichier "source" du module "lex.c".
 *		  Pour les mots cles des "fprintf_..." voir "token.c".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<stdio.h>

#include	<visp/vpMy.h>
#include	<visp/vpArit.h>
#include	<visp/vpToken.h>
#include	<visp/vpLex.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*
 * La procedure "fprintf_Position" ecrit en ascii un positionnement.
 * Entree :
 * f		Fichier en sortie.
 * pp		Positionnement a ecrite.
 */
void fprintf_Position (FILE *f, AritPosition *pp)
{
	fprintf (f, "%.3f\t%.3f\t%.3f\n%.3f\t%.3f\t%.3f\n%.3f\t%.3f\t%.3f\n",
		pp->rotate.x,	 pp->rotate.y,	  pp->rotate.z,
		pp->scale.x,	 pp->scale.y,	  pp->scale.z,
		pp->translate.x, pp->translate.y, pp->translate.z);
}

/*
 * La procedure "fscanf_Point3f" lit en ascii un point flottant 3D.
 * Entree :
 * pp		Point flottant 3D a lire.
 */
void fscanf_Point3f (Point3f *pp)
{
static	 char	*err_tbl[] = {
"float expected (coordinate ",
" of point)"
};
	 int	t;

	/* Lecture de la premiere coordonnee du point.	*/

	if ((t = lex ()) != T_FLOAT && t != T_INT)
	  lexerr ("start",err_tbl[0], "X", err_tbl[1], NULL);	
	pp->x = (t == T_INT) ? (float) myint : myfloat;

	/* Lecture de la seconde coordonnee du point.	*/

	if ((t= lex ()) != T_FLOAT && t != T_INT)
		lexerr ("start",err_tbl[0], "Y", err_tbl[1], NULL);	
	pp->y = (t == T_INT) ? (float) myint : myfloat;

	/* Lecture de la troisieme coordonnee du point.	*/

	if ((t= lex ()) != T_FLOAT && t != T_INT)
		lexerr ("start",err_tbl[0], "Z", err_tbl[1], NULL);	
	pp->z = (t == T_INT) ? (float) myint : myfloat;
}

/*
 * La procedure "fscanf_Vector" lit en ascii un vecteur.
 * Entree :
 * vp		Vecteur a lire.
 */
void fscanf_Vector (Vector *vp)
{
static	 char	*err_tbl[] = {
"float expected (coordinate ",
" of vector)"
};

	 int	t;

	/* Lecture de la premiere coordonnee du vecteur.	*/

	if ((t= lex ()) != T_FLOAT && t != T_INT)
		lexerr ("start",err_tbl[0], "X", err_tbl[1], NULL);	
	vp->x = (t == T_INT) ? (float) myint : myfloat;

	/* Lecture de la seconde coordonnee du vecteur.		*/

	if ((t= lex ()) != T_FLOAT && t != T_INT)
		lexerr ("start",err_tbl[0], "Y", err_tbl[1], NULL);	
	vp->y = (t == T_INT) ? (float) myint : myfloat;

	/* Lecture de la troisieme coordonnee du vecteur.	*/

	if ((t= lex ()) != T_FLOAT && t != T_INT)
		lexerr ("start",err_tbl[0], "Z", err_tbl[1], NULL);	
	vp->z = (t == T_INT) ? (float) myint : myfloat;
}

/*
 * La procedure "fscanf_Position" lit en ascii un positionnement.
 * Entree :
 * pp		Positionnement a lire.
 */
void fscanf_Position (AritPosition *pp)
{
	pusherr ("rotate: ");
	fscanf_Vector (&pp->rotate);
	popuperr ("scale: ");
	fscanf_Vector (&pp->scale);
	popuperr ("translate: ");
	fscanf_Vector (&pp->translate);
	poperr ();
}

#endif
