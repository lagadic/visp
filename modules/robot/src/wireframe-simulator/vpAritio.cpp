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
 * Le module "aritio.c" contient les procedures d'entree/sortie
 *		  des types definis dans le module "arit.h".
 *		  Les entrees non specifiees sont effectuees
 *		  sur le fichier "source" du module "lex.c".
 *		  Pour les mots cles des "fprintf_..." voir "token.c".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include "vpArit.h"
#include "vpLex.h"
#include "vpMy.h"
#include "vpToken.h"

#include <stdio.h>
/*
 * La procedure "fprintf_Position" ecrit en ascii un positionnement.
 * Entree :
 * f		Fichier en sortie.
 * pp		Positionnement a ecrite.
 */
void fprintf_Position(FILE *f, AritPosition *pp)
{
  fprintf(f, "%.3f\t%.3f\t%.3f\n%.3f\t%.3f\t%.3f\n%.3f\t%.3f\t%.3f\n", pp->rotate.x, pp->rotate.y, pp->rotate.z,
          pp->scale.x, pp->scale.y, pp->scale.z, pp->translate.x, pp->translate.y, pp->translate.z);
}

/*
 * La procedure "fscanf_Point3f" lit en ascii un point flottant 3D.
 * Entree :
 * pp		Point flottant 3D a lire.
 */
void fscanf_Point3f(Point3f *pp)
{
  static const char *err_tbl[] = {"float expected (coordinate ", " of point)"};
  int t;

  /* Lecture de la premiere coordonnee du point.	*/

  if ((t = lex()) != T_FLOAT && t != T_INT)
    lexerr("start", err_tbl[0], "X", err_tbl[1], NULL);
  pp->x = (t == T_INT) ? (float)myint : myfloat;

  /* Lecture de la seconde coordonnee du point.	*/

  if ((t = lex()) != T_FLOAT && t != T_INT)
    lexerr("start", err_tbl[0], "Y", err_tbl[1], NULL);
  pp->y = (t == T_INT) ? (float)myint : myfloat;

  /* Lecture de la troisieme coordonnee du point.	*/

  if ((t = lex()) != T_FLOAT && t != T_INT)
    lexerr("start", err_tbl[0], "Z", err_tbl[1], NULL);
  pp->z = (t == T_INT) ? (float)myint : myfloat;
}

/*
 * La procedure "fscanf_Vector" lit en ascii un vecteur.
 * Entree :
 * vp		Vecteur a lire.
 */
void fscanf_Vector(Vector *vp)
{
  static const char *err_tbl[] = {"float expected (coordinate ", " of vector)"};

  int t;

  /* Lecture de la premiere coordonnee du vecteur.	*/

  if ((t = lex()) != T_FLOAT && t != T_INT)
    lexerr("start", err_tbl[0], "X", err_tbl[1], NULL);
  vp->x = (t == T_INT) ? (float)myint : myfloat;

  /* Lecture de la seconde coordonnee du vecteur.		*/

  if ((t = lex()) != T_FLOAT && t != T_INT)
    lexerr("start", err_tbl[0], "Y", err_tbl[1], NULL);
  vp->y = (t == T_INT) ? (float)myint : myfloat;

  /* Lecture de la troisieme coordonnee du vecteur.	*/

  if ((t = lex()) != T_FLOAT && t != T_INT)
    lexerr("start", err_tbl[0], "Z", err_tbl[1], NULL);
  vp->z = (t == T_INT) ? (float)myint : myfloat;
}

/*
 * La procedure "fscanf_Position" lit en ascii un positionnement.
 * Entree :
 * pp		Positionnement a lire.
 */
void fscanf_Position(AritPosition *pp)
{
  pusherr("rotate: ");
  fscanf_Vector(&pp->rotate);
  popuperr("scale: ");
  fscanf_Vector(&pp->scale);
  popuperr("translate: ");
  fscanf_Vector(&pp->translate);
  poperr();
}

#endif
