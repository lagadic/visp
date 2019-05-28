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
 * Le module "skipio.c" contient les procedures d'analyse
 * syntaxique du fichier "source" qui permettent de traiter
 * les commandes inconnues.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include "vpLex.h"
#include "vpMy.h"
#include "vpSkipio.h"
#include "vpToken.h"
#include <stdio.h>

/*
 * La procedure "skip_cmd" saute les structures d'une commande
 * jusqu'a reconnaitre le debut d'une nouvelle commande.
 * Entree :
 * f		Fichier en sortie.
 */
void skip_cmd(void)
{
  int token;

  fprintf(stderr, "\n$ ");
  fwrite(mytext, (size_t)mylength, 1, stderr);
  while ((token = lexecho(stderr, '$')) != T_EOF && token != '$') {
  };
  unlex();
}

/*
 * La procedure "skip_keyword" saute les structures des articles
 * jusqu'a reconnaitre le mot cle de jeton "token".
 * Entree :
 * token	Jeton du mot cle a reconnaitre.
 * err		Message d'erreur si le mot cle n'est pas reconnu.
 */
void skip_keyword(int token, const char *err)
{
  int t;

  switch (t = lex()) {
  case T_IDENT: /* saute le mot cle inconnu	*/
    while ((t = lex()) != 0) {
      switch (t) {
      case '$':   /* nouvelle commande		*/
      case T_EOF: /* fin de fichier		*/
        lexerr("start", err, NULL);
        break;
      default:
        if (t == token)
          return;
        break;
      }
    }
    break;
  default:
    if (t == token)
      return;
    break;
  }
  lexerr("start", err, NULL);
}

#endif
