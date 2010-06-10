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
 * Le module "parser.c" contient les procedures de gestion
 * de l'analyse syntaxique d'un fichier source dont la grammaire
 * possede les symboles terminaux de "lex.c".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<stdio.h>

#include	<visp/vpMy.h>
#include	<visp/vpArit.h>
#include	<visp/vpView.h>
#include	<visp/vpBound.h>
#include	<visp/vpToken.h>
#include	<visp/vpLex.h>
#include	<visp/vpSkipio.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifdef	used
extern	Byte		*get_remove ();
extern	View_parameters	*get_view_parameters ();
#endif	/* used	*/

/*
 * La procedure "parser" fait l'analyse syntaxique du fichier source.
 * Entree/Sortie :
 * bsp		Scene surfacique polygonale a lire.
 */
void parser (Bound_scene *bsp)
{
	int	token;

	while ((token = lex ()) != T_EOF) 
	switch (token) {
	case '$' :
		switch (lex ()) {
		case T_IDENT	:	/* saute la commande inconnue	*/
			skip_cmd (stderr);
			unlex ();
			break;
		case T_EXIT	:
			return;
			break;
		case T_BOUND	:
			if (bsp->bound.nbr == BOUND_NBR) {
				fprintf (stderr, "mire: too much bound\n");
				return;
			}
			fscanf_Bound (
			&(bsp->bound.ptr[bsp->bound.nbr++]));
			break;
#ifdef	used
		case T_REMOVE	:
			fscanf_Remove (get_remove ());
			break;
		case T_VIEW	:
			fscanf_View_parameters (get_view_parameters ());
			set_projection ();
			break;
#endif	/* used	*/
		default		:
		  lexerr ("start", "keyword expected", NULL); 
			break;
		}
		break;
	default	:
	  lexerr ("start", "symbol '$' expected", NULL);
		break;
	}
}

#endif
