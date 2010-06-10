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
 * Le module "skipio.c" contient les procedures d'analyse
 * syntaxique du fichier "source" qui permettent de traiter
 * les commandes inconnues.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<stdio.h>

#include	<visp/vpMy.h>
#include	<visp/vpToken.h>
#include	<visp/vpLex.h>
#include	<visp/vpSkipio.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS


/*
 * La procedure "skip_cmd" saute les structures d'une commande
 * jusqu'a reconnaitre le debut d'une nouvelle commande.
 * Entree :
 * f		Fichier en sortie.
 */
void skip_cmd (/*FILE *f*/)
{
	int	token;

	fprintf (stderr, "\n$ ");
	fwrite (mytext, mylength, 1, stderr);
	while ((token = lexecho (stderr, '$')) !=  T_EOF && token != '$');
	unlex ();
}

/*
 * La procedure "skip_keyword" saute les structures des articles 
 * jusqu'a reconnaitre le mot cle de jeton "token".
 * Entree :
 * token	Jeton du mot cle a reconnaitre.
 * err		Message d'erreur si le mot cle n'est pas reconnu.
 */
void skip_keyword (int token, char *err)
{
	int	t;

	switch (t = lex ()) {
	case T_IDENT :		/* saute le mot cle inconnu	*/
		while ((t = lex ())) 
		switch (t) {
		case '$'   :	/* nouvelle commande		*/
		case T_EOF :	/* fin de fichier		*/
		  lexerr ("start", err, NULL);
			break;
		default	:
			if (t == token) return;
			break;
		}
		break;
	default	:
		if (t == token) return;
		break;
	}
	lexerr ("start", err, NULL);
}

#endif

