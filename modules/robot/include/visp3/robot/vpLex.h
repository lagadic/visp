/****************************************************************************
 *
 * $Id: vpLex.h 5297 2015-02-10 11:19:24Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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

#include <visp3/core/vpConfig.h>
#include <stdio.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

int lex(void);
void unlex (void);
void lexerr (const char* path, ...);
void pusherr (const char *str);
void popuperr (const char *str);
void poperr (void);
int lexecho (FILE *f, int token);

#endif
#endif
