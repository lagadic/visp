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
 * Le module "tmstack.c" contient les procedures de gestion
 * de la pile de matrices de transformation (Transformation
 * Matrix STACK).
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include "vpTmstack.h"
#include "vpArit.h"
#include "vpMy.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#define STACKSIZE 32

static Matrix stack[STACKSIZE] /* = IDENTITY_MATRIX*/; /* pile		*/
static Matrix *sp = stack;                             /* sommet 	*/

/*
 * La procedure "get_tmstack" retourne la matrice au sommet
 * de la pile des matrices de transformation.
 * Sortie :
 *		Pointeur de la matrice au sommet de la pile.
 */
Matrix *get_tmstack(void) { return (sp); }

/*
 * La procedure "load_tmstack" charge une matrice au sommet
 * de la pile des matrices de transformation.
 * Entree :
 * m		Matrice a charger.
 */
void load_tmstack(Matrix m)
{
  // bcopy ((char *) m, (char *) *sp, sizeof (Matrix));
  memmove((char *)*sp, (char *)m, sizeof(Matrix));
}

/*
 * La procedure "pop_tmstack" depile la matrice au sommet
 * de la pile des matrices de transformation.
 */
void pop_tmstack(void)
{
  if (sp == stack) {
    static char proc_name[] = "pop_tmstack";
    fprintf(stderr, "%s: stack underflow\n", proc_name);
    return;
  } else
    sp--;
}

/*
 * La procedure "push_tmstack" empile et duplique le sommet
 * de la pile des matrices de transformation.
 */
void push_tmstack(void)
{
  if (sp == stack + STACKSIZE - 1) {
    static char proc_name[] = "push_tmstack";
    fprintf(stderr, "%s: stack overflow\n", proc_name);
    return;
  }
  sp++;
  // bcopy ((char *) (sp - 1), (char *) sp, sizeof (Matrix));
  memmove((char *)sp, (char *)(sp - 1), sizeof(Matrix));
}

/*
 * La procedure "swap_tmstack" echange les deux premieres matrices
 * de la pile des matrices de transformation.
 */
void swap_tmstack(void)
{
  Matrix *mp, tmp;

  mp = (sp == stack) ? sp + 1 : sp - 1;
  // 	bcopy ((char *) *sp, (char *) tmp, sizeof (Matrix));
  // 	bcopy ((char *) *mp, (char *) *sp, sizeof (Matrix));
  // 	bcopy ((char *) tmp, (char *) *mp, sizeof (Matrix));
  memmove((char *)tmp, (char *)*sp, sizeof(Matrix));
  memmove((char *)*sp, (char *)*mp, sizeof(Matrix));
  memmove((char *)*mp, (char *)tmp, sizeof(Matrix));
}

/*
 * La procedure "postmult_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation.
 * Entree :
 * m		Matrice multiplicative.
 */
void postmult_tmstack(Matrix m) { postmult_matrix(*sp, m); }

/*
 * La procedure "postrotate_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation par une rotation.
 * Entree :
 * vp		Vecteur de rotation.
 */
void postrotate_tmstack(Vector *vp)
{
  Matrix m;

  Rotate_to_Matrix(vp, m);
  postmult3_matrix(*sp, m);
}

/*
 * La procedure "postscale_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation par une homothetie.
 * Entree :
 * vp		Vecteur d'homothetie.
 */
void postscale_tmstack(Vector *vp) { postscale_matrix(*sp, vp); }

/*
 * La procedure "posttranslate_tmstack" postmultiplie la matrice au sommet
 * de la pile des matrices de transformation par une translation.
 * Entree :
 * vp		Vecteur de translation.
 */
void posttranslate_tmstack(Vector *vp) { posttrans_matrix(*sp, vp); }

/*
 * La procedure "premult_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation.
 * Entree :
 * m		Matrice multiplicative.
 */
void premult_tmstack(Matrix m) { premult_matrix(*sp, m); }

/*
 * La procedure "prerotate_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation par une rotation.
 * Entree :
 * vp		Vecteur de rotation.
 */
void prerotate_tmstack(Vector *vp)
{
  Matrix m;

  Rotate_to_Matrix(vp, m);
  premult3_matrix(*sp, m);
}

/*
 * La procedure "prescale_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation par une homothetie.
 * Entree :
 * vp		Vecteur d'homothetie.
 */
void prescale_tmstack(Vector *vp) { prescale_matrix(*sp, vp); }

/*
 * La procedure "pretranslate_tmstack" premultiplie la matrice au sommet
 * de la pile des matrices de transformation par une translation.
 * Entree :
 * vp		Vecteur de translation.
 */
void pretranslate_tmstack(Vector *vp) { pretrans_matrix(*sp, vp); }

#endif
