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
 * Le module "myio.c" contient les procedures d'entree/sortie
 * des types definis dans le module "my.h".
 * Les entrees non specifiees sont effectuees
 * sur le fichier "source" du module "lex.c".
 * Pour les mots cles des "fprintf_..." voir "token.c".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>

#include	<visp/vpMy.h>
#include	<visp/vpToken.h>
#include	<visp/vpLex.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS


extern	char	*mytext;	/* chaine du symbole courant	*/


/*
 * La procedure "fscanf_float" lit en ascii un nombre flottant.
 * Entree :
 * fp		Nombre flottant a lire.
 */
void fscanf_float (float *fp)
{
	int	t;

	if ((t = lex ()) != T_FLOAT && t != T_INT)
	  lexerr ("start", "float expected", NULL);
	*fp = (t == T_INT) ? (float) myint : myfloat;
}

/*
 * La procedure "fscanf_Index" lit en ascii un indice.
 * Entree :
 * ip		Indice a lire.
 */
void fscanf_Index (Index *ip)
{
	if (lex () != T_INT)
		lexerr ("start", "integer expected", NULL);
	*ip = (Index) myint;
}

/*
 * La procedure "fscanf_int" lit en ascii un nombre entier.
 * Entree :
 * ip		Nombre entier a lire.
 */
void fscanf_int (int *ip)
{
	if (lex () != T_INT)
		lexerr ("start", "integer expected", NULL);
	*ip = myint;
}

/*
 * La procedure "fscanf_string" lit en ascii une chaine de caracteres.
 * Entree :
 * str		Chaine a lire.
 */
void fscanf_string (char **str)
{
	if (lex () != T_STRING)
		lexerr ("start", "string expected", NULL);
	if (*str == NULL)
		*str = (char *) malloc ((mylength + 1) * sizeof (char));
	else	*str = (char *) realloc (*str, (mylength + 1) * sizeof (char));
	strncpy (*str, mytext, mylength);
}

/*
 * La procedure "fscanf_Type" lit en ascii un octet.
 * Entree :
 * ip		Type a lire.
 */
void fscanf_Type (Type *ip)
{
	if (lex () != T_INT)
		lexerr ("start", "integer expected", NULL);
	*ip = (Type ) myint;
}

#endif
