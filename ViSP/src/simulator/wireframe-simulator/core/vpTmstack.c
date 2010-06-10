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
 * Le module "tmstack.c" contient les procedures de gestion
 * de la pile de matrices de transformation (Transformation
 * Matrix STACK).
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<math.h>
#include	<stdio.h>
#include	<string.h>

#include	<visp/vpMy.h>
#include	<visp/vpArit.h>
#include	<visp/vpTmstack.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define	STACKSIZE	32


static	Matrix	stack[STACKSIZE]/* = IDENTITY_MATRIX*/;	/* pile		*/
static	Matrix	*sp		 = stack;		/* sommet 	*/


/*
 * La procedure "get_tmstack" retourne la matrice au sommet
 * de la pile des matrices de transformation.
 * Sortie :
 *		Pointeur de la matrice au sommet de la pile.
 */
Matrix	*
get_tmstack ()
{
	return (sp);
}

/*
 * La procedure "load_tmstack" charge une matrice au sommet
 * de la pile des matrices de transformation.
 * Entree :
 * m		Matrice a charger.
 */
void
load_tmstack (Matrix m)
{
	//bcopy ((char *) m, (char *) *sp, sizeof (Matrix));
	memmove ((char *) *sp, (char *) m, sizeof (Matrix));
}

/*
 * La procedure "pop_tmstack" depile la matrice au sommet
 * de la pile des matrices de transformation.
 */
void
pop_tmstack ()
{
	static	char	proc_name[] = "pop_tmstack";

	if (sp == stack) {
		fprintf (stderr, "%s: stack underflow\n", proc_name);
		return;
	}
	else	sp--;
}

/*
 * La procedure "push_tmstack" empile et duplique le sommet
 * de la pile des matrices de transformation.
 */
void
push_tmstack ()
{
	static	char	proc_name[] = "push_tmstack";

	if (sp == stack + STACKSIZE - 1) {
		fprintf (stderr, "%s: stack overflow\n", proc_name);
		return;
	}
	sp++;
	//bcopy ((char *) (sp - 1), (char *) sp, sizeof (Matrix));
	memmove ((char *) sp, (char *) (sp - 1), sizeof (Matrix));
}

/*
 * La procedure "swap_tmstack" echange les deux premieres matrices
 * de la pile des matrices de transformation.
 */
void
swap_tmstack ()
{
	Matrix	*mp, tmp;

	mp = (sp == stack) ? sp + 1 : sp - 1; 
// 	bcopy ((char *) *sp, (char *) tmp, sizeof (Matrix));
// 	bcopy ((char *) *mp, (char *) *sp, sizeof (Matrix));
// 	bcopy ((char *) tmp, (char *) *mp, sizeof (Matrix));
	memmove ((char *) tmp, (char *) *sp, sizeof (Matrix));
	memmove ((char *) *sp, (char *) *mp, sizeof (Matrix));
	memmove ((char *) *mp, (char *) tmp, sizeof (Matrix)); 
}

/*
 * La procedure "postmult_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation.
 * Entree :
 * m		Matrice multiplicative.
 */
void
postmult_tmstack (Matrix m)
{
	postmult_matrix (*sp, m);
}

/*
 * La procedure "postrotate_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation par une rotation.
 * Entree :
 * vp		Vecteur de rotation.
 */
void
postrotate_tmstack (Vector *vp)
{
	Matrix	m;

	Rotate_to_Matrix (vp, m);
	postmult3_matrix (*sp, m);
}

/*
 * La procedure "postscale_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation par une homothetie.
 * Entree :
 * vp		Vecteur d'homothetie.
 */
void
postscale_tmstack (Vector *vp)
{
	postscale_matrix (*sp, vp);
}

/*
 * La procedure "posttranslate_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation par une translation.
 * Entree :
 * vp		Vecteur de translation.
 */
void
posttranslate_tmstack (Vector *vp)
{
	posttrans_matrix (*sp, vp);
}

/*
 * La procedure "premult_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation.
 * Entree :
 * m		Matrice multiplicative.
 */
void
premult_tmstack (Matrix m)
{
	premult_matrix (*sp, m);
}

/*
 * La procedure "prerotate_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation par une rotation.
 * Entree :
 * vp		Vecteur de rotation.
 */
void
prerotate_tmstack (Vector *vp)
{
	Matrix	m;

	Rotate_to_Matrix (vp, m);
	premult3_matrix (*sp, m);
}

/*
 * La procedure "prescale_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation par une homothetie.
 * Entree :
 * vp		Vecteur d'homothetie.
 */
void
prescale_tmstack (Vector *vp)
{
	prescale_matrix (*sp, vp);
}

/*
 * La procedure "pretranslate_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation par une translation.
 * Entree :
 * vp		Vecteur de translation.
 */
void
pretranslate_tmstack (Vector *vp)
{
	pretrans_matrix (*sp, vp);
}

#endif

