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
 * Le module "rfstack.c" contient les procedures de gestion
 * de la pile d'elimination de faces (Remove Faces STACK).
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<stdio.h>
#include	<string.h>

#include	<visp/vpMy.h>
#include	<visp/vpArit.h>
#include	<visp/vpView.h>
#include	<visp/vpRfstack.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define	STACKSIZE	32


static	int	stack[STACKSIZE] = {DEFAULT_REMOVE};	/* pile		*/
static	int	*sp		 = stack;		/* sommet 	*/


/*
 * La procedure "fprintf_rfstack" affiche le sommet
 * de la pile des drapeaux d'elimination de faces.
 * Entree :
 * fp		Fichier en sortie.
 */
void
fprintf_rfstack (FILE *fp)
{
	int	flg = 0;	/* nul si element unique	*/

	if (*sp == IS_INSIDE) {
		fprintf (fp, "(null)\n");
		return;
	}
	fprintf (fp, "(");
	if (*sp & IS_ABOVE) {
		if (flg++) fprintf (fp, " ");
		fprintf (fp, "above");
	}
	if (*sp & IS_BELOW) {
		if (flg++) fprintf (fp, " ");
		fprintf (fp, "below");
	}
	if (*sp & IS_RIGHT) {
		if (flg++) fprintf (fp, " ");
		fprintf (fp, "right");
	}
	if (*sp & IS_LEFT) {
		if (flg++) fprintf (fp, " ");
	 	fprintf (fp, "left");
	}
	if (*sp & IS_BACK) {
		if (flg++) fprintf (fp, " ");
		fprintf (fp, "back");
	}
	if (*sp & IS_FRONT) {
		if (flg++) fprintf (fp, " ");
		fprintf (fp, "front");
	}
	fprintf (fp, ")\n");
}

/*
 * La procedure "get_rfstack" retourne les drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 * Sortie :
 * 		Pointeur sur les drapeaux d'elimination du sommet de la pile.
 */
int	*
get_rfstack ()
{
	return (sp);
}

/*
 * La procedure "load_rfstack" charge des drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 * Entree :
 * i		Niveau a charger.
 */
void
load_rfstack (i)
int	i;
{
	*sp = i;
}

/*
 * La procedure "pop_rfstack" depile les drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void
pop_rfstack ()
{
	static	char	proc_name[] = "pop_rfstack";

	if (sp == stack) {
		fprintf (stderr, "%s: stack underflow\n", proc_name);
		return;
	}
	else	sp--;
}

/*
 * La procedure "push_rfstack" empile et duplique les drapeaux du sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void
push_rfstack ()
{
	static	char	proc_name[] = "push_rfstack";

	if (sp == stack + STACKSIZE - 1) {
		fprintf (stderr, "%s: stack overflow\n", proc_name);
		return;
	}
	sp++;
	*sp = *(sp - 1);
}

/*
 * La procedure "swap_rfstack" echange les deux premiers elements
 * de la pile des drapeaux d'elimination de faces.
 */
void
swap_rfstack ()
{
	int	*ip, tmp;

	ip = (sp == stack) ? sp + 1 : sp - 1; 
	SWAP(*sp, *ip, tmp);
}

/*
 * La procedure "add_rfstack" ajoute des drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void
add_rfstack (i)
int	i;
{
	*sp |= i;
}

/*
 * La procedure "sub_rfstack" soustrait des drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void
sub_rfstack (i)
int	i;
{
	*sp &= ~i;
}

#endif

