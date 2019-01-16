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

#include "vpMyio.h"
#include "vpLex.h"
#include "vpToken.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

extern char *mytext; /* chaine du symbole courant	*/

/*
 * La procedure "fscanf_float" lit en ascii un nombre flottant.
 * Entree :
 * fp		Nombre flottant a lire.
 */
void fscanf_float(float *fp)
{
  int t;

  if ((t = lex()) != T_FLOAT && t != T_INT)
    lexerr("start", "float expected", NULL);
  *fp = (t == T_INT) ? (float)myint : myfloat;
}

/*
 * La procedure "fscanf_Index" lit en ascii un indice.
 * Entree :
 * ip		Indice a lire.
 */
void fscanf_Index(Index *ip)
{
  if (lex() != T_INT)
    lexerr("start", "integer expected", NULL);
  *ip = (Index)myint;
}

/*
 * La procedure "fscanf_int" lit en ascii un nombre entier.
 * Entree :
 * ip		Nombre entier a lire.
 */
void fscanf_int(int *ip)
{
  if (lex() != T_INT)
    lexerr("start", "integer expected", NULL);
  *ip = myint;
}

/*
 * La procedure "fscanf_string" lit en ascii une chaine de caracteres.
 * Entree :
 * str		Chaine a lire.
 */
void fscanf_string(char **str)
{
  if (lex() != T_STRING)
    lexerr("start", "string expected", NULL);
  if (*str == NULL)
    *str = (char *)malloc((size_t)(mylength + 1) * sizeof(char));
  else
    *str = (char *)realloc(*str, (size_t)(mylength + 1) * sizeof(char));

  if (*str == NULL) {
    printf("Unable to read the string: bad memory allocation");
    return;
  }

  strncpy(*str, mytext, (size_t)mylength);
}

/*
 * La procedure "fscanf_Type" lit en ascii un octet.
 * Entree :
 * ip		Type a lire.
 */
void fscanf_Type(Type *ip)
{
  if (lex() != T_INT)
    lexerr("start", "integer expected", NULL);
  *ip = (Type)myint;
}

#endif
