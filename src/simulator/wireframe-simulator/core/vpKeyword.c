/****************************************************************************
 *
 * $Id$
 *
* This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Le module "keyword.c" contient les procedures de gestion
 * des mots cles retournes par l'analyseur lexical "lex".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/




#include	<visp/vpMy.h>
#include	<visp/vpToken.h>

#include	<stdio.h>
#include	<stdlib.h>
#include	<string.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS


static	void	open_hash ();
static	void	close_hash ();
//static	int	hashplw ();
static	void	insert_keyword ();

#ifdef	debug
static	void	delete_keyword ();
static	char	*get_keyword ();
#endif	/* debug */


#define	PRIME		211
#define	NEXT(x)		(x) = (x)->next

typedef struct	bucket {
	struct	bucket	*next;	/* element suivant	*/
	char		*ident;	/* identifateur 	*/
	Byte		length;	/* longueur de "ident"	*/
	Index		token;	/* code du jeton 	*/
} Bucket;


static	Bucket	**hash_tbl;	/* table de "hash-coding"	*/


/*
 * La procedure "open_keyword" alloue et initialise les variables utilisees
 * par les procedures de gestion des mots cles.
 * Entree :
 * kwp		Tableau des mots cles termine par NULL.
 */
void open_keyword (Keyword *kwp)
{
	open_hash ();
	for (; kwp->ident != NULL; kwp++) /* recopie les mots cles	*/
		insert_keyword (kwp->ident, kwp->token);
}

/*
 * La procedure "close_keyword" libere les variables utilisees
 * par les procedures de gestion des mots cles.
 */
void close_keyword ()
{
	close_hash ();
}

/*
 * La procedure "open_hash" alloue et initialise la table de codage.
 */
static	void
open_hash ()
{
	static	 char	proc_name[] = "open_hash";

	Bucket	**head, **bend;

	if ((hash_tbl = (Bucket **) malloc (sizeof (Bucket *) * PRIME))==NULL){
		perror (proc_name);
		exit (1);
	}
	head = hash_tbl;
	bend = head + PRIME;
	for (; head < bend; *head++ = NULL);
}

/*
 * La procedure "close_hash" libere la table de codage et ses elements.
 */
static	void
close_hash ()
{
	Bucket	**head = hash_tbl;
	Bucket **bend = head + PRIME;
	Bucket	*bp;	/* element courant	*/
	Bucket	*next;	/* element suivant	*/

	for (; head < bend; head++) {	/* libere les listes	*/
		for (bp = *head; bp != NULL; bp = next) {
			next = bp->next;
			free ((char *) bp);
		}
	}
	free ((char *) hash_tbl);	/* libere la table	*/
}

/*
 * La procedure "hashpjw" calcule un indice code a partir de la chaine
 * de caracteres "str".
 * Pour plus de renseignements, voir :
 *	"Compilers. Principles, Techniques, and Tools",
 *	A.V. AHO, R. SETHI, J.D. ULLMAN,
 *	ADDISON-WESLEY PUBLISHING COMPANY, pp 436.
 * Entree :
 * str		Chaine de caracteres a coder.
 * Sortie :
 *		Le code de la chaine.
 */
static	int
hashpjw (char *str)
{
	unsigned	h = 0;	/* "hash value"	*/
	unsigned	g;

	for (; *str != '\0'; str++) {
		h = (h << 4) + *str;
		if ((g = h & ~0xfffffff)) {
			h ^= g >> 24;
			h ^= g;
		}
	}
	return (h % PRIME);
}


/*
 * La procedure "insert_keyword" insere en tete d'un point d'entree
 * de la table de "hachage" le mot cle ayant pour identificateur
 * la chaine de caracteres "str" et pour valeur "token".
 * Entree :
 * str		Chaine de caracteres du mot cle.
 * token	Valeur du jeton associe au mot cle.
 */
static	void
insert_keyword (char *str, int token)
{
	static	 char	proc_name[] = "insert_keyword";

	Bucket	**head = hash_tbl + hashpjw (str);
	Bucket	*bp;
	int	length;

	length = strlen (str);
	if ((bp = (Bucket *) malloc (sizeof (Bucket) + length  + 1)) == NULL) {
		perror (proc_name);
		exit (1);
	}
	bp->length = length;
	bp->token  = token;
	bp->ident  = (char *) (bp + 1);
	strcpy (bp->ident, str);

	bp->next = *head;	/* insere "b" en tete de "head"	*/
	*head = bp;
}

/*
 * La pocedure "get_symbol" verifie que la chaine pointee par "ident"
 * de longeur "length" est un mot cle.
 * Entree :
 * ident	Chaine de l'identificateur.
 * length	Nombre de caracteres de la chaine.
 * Note :
 * La chaine "ident" n'est pas terminee par '\0'.
 * Sortie :
 * 		Valeur du jeton associe si c'est un mot cle, 0 sinon.
 */
Index get_symbol (char *ident, int length)
{
	Bucket	*bp; 
	char	*kwd; 
	char	*idn = ident;
	int	len  = length;

	{	/* calcule le code de hachage (voir "hashpjw")	*/
		unsigned	h = 0;	/* "hash value"	*/
		unsigned	g;

		for (; len != 0; idn++, len--) {
			h = (h << 4) + *idn;
			if ((g = h & ~0xfffffff)) {
				h ^= g >> 24;
				h ^= g;
			}
		}
		bp = hash_tbl[h % PRIME];
	}

	/* recherche le mot cle	*/

	for (; bp != NULL; NEXT(bp)) {
		if (length == bp->length) {
			idn = ident;
			len = length;
			kwd = bp->ident;
			for (; *idn == *kwd; idn++, kwd++) 
				if (--len == 0) return (bp->token);
		}
	}
	return (0);	/*  identificateur	*/
}

#endif
