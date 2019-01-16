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
 * Le module "parser.c" contient les procedures de gestion
 * de l'analyse syntaxique d'un fichier source dont la grammaire
 * possede les symboles terminaux de "lex.c".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include "vpParser.h"
#include "vpBoundio.h"
#include "vpLex.h"
#include "vpSkipio.h"
#include "vpToken.h"

#include <stdio.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*
 * La procedure "parser" fait l'analyse syntaxique du fichier source.
 * Entree/Sortie :
 * bsp		Scene surfacique polygonale a lire.
 */
void parser(Bound_scene *bsp)
{
  int token;

  while ((token = lex()) != T_EOF)
    switch (token) {
    case '$':
      switch (lex()) {
      case T_IDENT: /* saute la commande inconnue	*/
        skip_cmd(/* stderr */);
        unlex();
        break;
      case T_EXIT:
        return;
        break;
      case T_BOUND:
        if (bsp->bound.nbr == BOUND_NBR) {
          fprintf(stderr, "mire: too much bound\n");
          return;
        }
        fscanf_Bound(&(bsp->bound.ptr[bsp->bound.nbr++]));
        break;
#ifdef used
      case T_REMOVE:
        fscanf_Remove(get_remove());
        break;
      case T_VIEW:
        fscanf_View_parameters(get_view_parameters());
        set_projection(void);
        break;
#endif /* used	*/
      default:
        lexerr("start", "keyword expected", NULL);
        break;
      }
      break;
    default:
      lexerr("start", "symbol '$' expected", NULL);
      break;
    }
}

#endif
