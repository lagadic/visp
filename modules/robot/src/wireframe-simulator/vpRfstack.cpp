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
 * Le module "rfstack.c" contient les procedures de gestion
 * de la pile d'elimination de faces (Remove Faces STACK).
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include "vpArit.h"
#include "vpMy.h"
#include "vpRfstack.h"
#include "vpView.h"
#include <stdio.h>
#include <string.h>
#define STACKSIZE 32

static int stack[STACKSIZE] = {vpDEFAULT_REMOVE}; /* pile		*/
static int *sp = stack;                           /* sommet 	*/

/*
 * La procedure "fprintf_rfstack" affiche le sommet
 * de la pile des drapeaux d'elimination de faces.
 * Entree :
 * fp		Fichier en sortie.
 */
void fprintf_rfstack(FILE *fp)
{
  int flg;
  flg = 0; /* nul si element unique	*/

  if (*sp == IS_INSIDE) {
    fprintf(fp, "(null)\n");
    return;
  }
  fprintf(fp, "(");
  if (*sp & IS_ABOVE) {
    // if (flg) fprintf (fp, " "); Removed since if (flg) cannot be true
    flg++;
    fprintf(fp, "above");
  }
  if (*sp & IS_BELOW) {
    if (flg)
      fprintf(fp, " ");
    flg++;
    fprintf(fp, "below");
  }
  if (*sp & IS_RIGHT) {
    if (flg)
      fprintf(fp, " ");
    flg++;
    fprintf(fp, "right");
  }
  if (*sp & IS_LEFT) {
    if (flg)
      fprintf(fp, " ");
    flg++;
    fprintf(fp, "left");
  }
  if (*sp & IS_BACK) {
    if (flg)
      fprintf(fp, " ");
    flg++;
    fprintf(fp, "back");
  }
  if (*sp & IS_FRONT) {
    /*if (flg)*/ fprintf(fp, " ");
    flg++;
    fprintf(fp, "front");
  }
  fprintf(fp, ")\n");
}

/*
 * La procedure "get_rfstack" retourne les drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 * Sortie :
 * 		Pointeur sur les drapeaux d'elimination du sommet de la pile.
 */
int *get_rfstack(void) { return (sp); }

/*
 * La procedure "load_rfstack" charge des drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 * Entree :
 * i		Niveau a charger.
 */
void load_rfstack(int i) { *sp = i; }

/*
 * La procedure "pop_rfstack" depile les drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void pop_rfstack(void)
{
  if (sp == stack) {
    static char proc_name[] = "pop_rfstack";
    fprintf(stderr, "%s: stack underflow\n", proc_name);
    return;
  } else
    sp--;
}

/*
 * La procedure "push_rfstack" empile et duplique les drapeaux du sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void push_rfstack(void)
{
  if (sp == stack + STACKSIZE - 1) {
    static char proc_name[] = "push_rfstack";
    fprintf(stderr, "%s: stack overflow\n", proc_name);
    return;
  }
  sp++;
  *sp = *(sp - 1);
}

/*
 * La procedure "swap_rfstack" echange les deux premiers elements
 * de la pile des drapeaux d'elimination de faces.
 */
void swap_rfstack(void)
{
  int *ip;

  ip = (sp == stack) ? sp + 1 : sp - 1;
  int tmp;
  // SWAP(*sp, *ip, tmp); // produce a cppcheck warning
  tmp = *sp;
  *sp = *ip;
  *ip = tmp;
}

/*
 * La procedure "add_rfstack" ajoute des drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void add_rfstack(int i) { *sp |= i; }

/*
 * La procedure "sub_rfstack" soustrait des drapeaux au sommet
 * de la pile des drapeaux d'elimination de faces.
 */
void sub_rfstack(int i) { *sp &= ~i; }

#endif
