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
 * Le module "lex.c" contient les procedures de gestion
 * de l'analyse lexicale de l'analyseur lexicale "lex"
 * d'un fichier source dont la grammaire possede
 * les symboles terminaux suivants (ecrit en "LEX", UNIX) :
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#ifndef vpLex_h
#define vpLex_h

#include <stdio.h>
#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

void open_lex(void);
void close_lex(void);
int lex(void);
void unlex(void);
void open_source(FILE *fd, const char *str);
void close_source(void);
void lexerr(const char *path, ...);
void pusherr(const char *str);
void popuperr(const char *str);
void poperr(void);
int lexecho(FILE *f, int token);

#endif
#endif
