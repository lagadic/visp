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
 * Le module "vwstack.c" contient les procedures de gestion
 * de la pile des points de vue (VieW STACK).
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <cmath> // std::fabs()
#include <limits>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "vpArit.h"
#include "vpMy.h"
#include "vpView.h"
#include "vpVwstack.h"

#define STACKSIZE 4

static View_parameters stack[STACKSIZE] = {vpDEFAULT_VIEW};
static View_parameters *sp = stack;

/*
 * La procedure "fprintf_vwstack" affiche un parametre du sommet
 * de la pile des prises de vue.
 * Entree :
 * fp		Fichier de sortie.
 * argv		Argument a afficher.
 *		Si argv est nul, tous les parametres sont affiches.
 */
void fprintf_vwstack(FILE *fp, char *argv)
{
  if (argv == NULL || strcmp(argv, "type") == 0) {
    const char *typetoa;

    switch (sp->type) {
    case PARALLEL:
      typetoa = "parallel";
      break;
    case PERSPECTIVE:
      typetoa = "perspective";
      break;
    default:
      typetoa = "unknown";
      break;
    }
    fprintf(fp, "(type\t%s)\n", typetoa);
    if (argv != NULL)
      return;
  }
  if (argv == NULL || strcmp(argv, "cop") == 0) {
    fprintf(fp, "(cop\t%.3f\t%.3f\t%.3f)\n", sp->cop.x, sp->cop.y, sp->cop.z);
    if (argv != NULL)
      return;
  }
  if (argv == NULL || strcmp(argv, "vrp") == 0) {
    fprintf(fp, "(vrp\t%.3f\t%.3f\t%.3f)\n", sp->vrp.x, sp->vrp.y, sp->vrp.z);
    if (argv != NULL)
      return;
  }
  if (argv == NULL || strcmp(argv, "vpn") == 0) {
    fprintf(fp, "(vpn\t%.3f\t%.3f\t%.3f)\n", sp->vpn.x, sp->vpn.y, sp->vpn.z);
    if (argv != NULL)
      return;
  }
  if (argv == NULL || strcmp(argv, "vup") == 0) {
    fprintf(fp, "(vup\t%.3f\t%.3f\t%.3f)\n", sp->vup.x, sp->vup.y, sp->vup.z);
    if (argv != NULL)
      return;
  }
  if (argv == NULL || strcmp(argv, "window") == 0) {
    fprintf(fp, "(window\t%.3f\t%.3f\t%.3f\t%.3f)\n", sp->vwd.umin, sp->vwd.umax, sp->vwd.vmin, sp->vwd.vmax);
    if (argv != NULL)
      return;
  }
  if (argv == NULL || strcmp(argv, "depth") == 0) {
    fprintf(fp, "(depth\t%.3f\t%.3f)\n", sp->depth.front, sp->depth.back);
    if (argv != NULL)
      return;
  }
  if (argv != NULL) {
    static char proc_name[] = "fprintf_vwstack";
    fprintf(stderr, "%s: argument unknown\n", proc_name);
  }
}

/*
 * La procedure "get_vwstack" retourne le point de vue au sommet
 * de la pile des points de vue.
 * Sortie :
 * 		Pointeur sur le point de vue du sommet de la pile.
 */
View_parameters *get_vwstack(void) { return (sp); }

/*
 * La procedure "load_vwstack" charge un point de vue au sommet
 * de la pile des points de vue.
 * Entree :
 * vp		Point de vue a charger.
 */
void load_vwstack(View_parameters *vp) { *sp = *vp; }

/*
 * La procedure "pop_vwstack" depile le point de vue au sommet
 * de la pile des points de vue.
 */
void pop_vwstack(void)
{
  if (sp == stack) {
    static char proc_name[] = "pop_vwstack";
    fprintf(stderr, "%s: stack underflow\n", proc_name);
    return;
  } else
    sp--;
}

/*
 * La procedure "push_vwstack" empile et duplique le point de vue au sommet
 * de la pile des points de vue.
 */
void push_vwstack(void)
{
  if (sp == stack + STACKSIZE - 1) {
    static char proc_name[] = "push_vwstack";
    fprintf(stderr, "%s: stack overflow\n", proc_name);
    return;
  }
  sp++;
  *sp = *(sp - 1);
}

/*
 * La procedure "swap_vwstack" echange les deux premiers elements
 * de la pile des points de vue.
 */
void swap_vwstack(void)
{
  View_parameters *vp, tmp;

  vp = (sp == stack) ? sp + 1 : sp - 1;
  SWAP(*sp, *vp, tmp);
}

/*
 * La procedure "add_vwstack" modifie un agrument du point de vue au sommet
 * de la pile des points de vue.
 * Entree :
 * va_alist	Nom de l'argument a modifier suivi de ses parametres.
 */

void add_vwstack(const char *path, ...)
// add_vwstack (va_alist)
// va_dcl
{
  va_list ap;
  char *argv;

  va_start(ap, path);
  argv = va_arg(ap, char *);
  if (strcmp(argv, "cop") == 0) {
    /* initialise le centre de projection	*/
    SET_COORD3(sp->cop, (float)va_arg(ap, double), (float)va_arg(ap, double), (float)va_arg(ap, double));
  } else if (strcmp(argv, "depth") == 0) {
    /* initialise les distances des plans de decoupage	*/
    sp->depth.front = (float)va_arg(ap, double);
    sp->depth.back = (float)va_arg(ap, double);
  } else if (strcmp(argv, "type") == 0) {
    /* initialise le type de projection	*/
    sp->type = (Type)va_arg(ap, int);
  } else if (strcmp(argv, "vpn") == 0) {
    /* initialise le vecteur normal au plan	*/
    float x = (float)va_arg(ap, double);
    float y = (float)va_arg(ap, double);
    float z = (float)va_arg(ap, double);

    // if (x == 0 && y == 0 && z == 0)
    if (std::fabs(x) <= std::numeric_limits<double>::epsilon() &&
        std::fabs(y) <= std::numeric_limits<double>::epsilon() &&
        std::fabs(z) <= std::numeric_limits<double>::epsilon()) {
      static char proc_name[] = "add_vwstack";
      fprintf(stderr, "%s: bad vpn\n", proc_name);
    } else {
      SET_COORD3(sp->vpn, x, y, z);
    }
  } else if (strcmp(argv, "vrp") == 0) {
    /* initialise le vecteur de reference	*/
    SET_COORD3(sp->vrp, (float)va_arg(ap, double), (float)va_arg(ap, double), (float)va_arg(ap, double));
  } else if (strcmp(argv, "vup") == 0) {
    /* initialise le vecteur haut du plan	*/
    float x = (float)va_arg(ap, double);
    float y = (float)va_arg(ap, double);
    float z = (float)va_arg(ap, double);

    // if (x == 0 && y == 0 && z == 0)
    if (std::fabs(x) <= std::numeric_limits<double>::epsilon() &&
        std::fabs(y) <= std::numeric_limits<double>::epsilon() &&
        std::fabs(z) <= std::numeric_limits<double>::epsilon()) {
      static char proc_name[] = "add_vwstack";
      fprintf(stderr, "%s: bad vup\n", proc_name);
    } else {
      SET_COORD3(sp->vup, x, y, z);
    }
  } else if (strcmp(argv, "window") == 0) {
    /* initialise la fenetre de projection	*/
    sp->vwd.umin = (float)va_arg(ap, double);
    sp->vwd.umax = (float)va_arg(ap, double);
    sp->vwd.vmin = (float)va_arg(ap, double);
    sp->vwd.vmax = (float)va_arg(ap, double);
  } else {
    static char proc_name[] = "add_vwstack";
    fprintf(stderr, "%s: bad argument\n", proc_name);
  }
  va_end(ap);
}

#endif
