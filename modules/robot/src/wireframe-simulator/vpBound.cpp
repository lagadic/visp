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
 * Le fichier "bound.c" contient les procedures de gestion des scenes de
 *modele geometrique surfacique.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include "vpArit.h"
#include "vpBound.h"
#include "vpMy.h"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * La procedure "free_Bound" libere la memoire d'une surface.
 * Les champs "bound.face.edge" ne sont pas utilises.
 * Entree :
 * bp		Surface a liberer.
 */
void free_Bound(Bound *bp)
{
  Face *fp = bp->face.ptr;
  Face *fend = fp + bp->face.nbr;

  for (; fp < fend; fp++) { /* libere les polygones	*/
    if (fp->vertex.ptr != fp->vertex.tbl)
      free((char *)fp->vertex.ptr);
  }
  if (bp->face.ptr != NULL) { /* libere les faces	*/
    free((char *)bp->face.ptr);
    bp->face.ptr = NULL;
  }
  if (bp->point.ptr != NULL) { /* libere les points	*/
    free((char *)bp->point.ptr);
    bp->point.ptr = NULL;
  }
#ifdef face_normal
  if (bp->normal.ptr != NULL) { /* libere les vecteurs	*/
    free((char *)bp->normal.ptr);
    bp->normal.ptr = NULL;
  }
#endif /* face_normal */
  bp->is_display = FALSE;
}

/*
 * La procedure "free_huge_Bound" libere  une surface de taille maximale.
 * La particularite de cette surface est le tableau unique des sommets.
 * Entree :
 * bp		Surface a liberer.
 */
void free_huge_Bound(Bound *bp)
{
  bp->face.nbr = 1; /* pour la liberation en une fois	*/
  free_Bound(bp);
}

/*
 * La procedure "free_Bound_scene" libere une scene de surfaces.
 * Entree :
 * bsp		Scene a liberer.
 */
void free_Bound_scene(Bound_scene *bsp)
{
  Bound *bp = bsp->bound.ptr;
  Bound *bend = bp + bsp->bound.nbr;

  for (; bp < bend; bp++) { /* libere les surfaces	*/
    free_Bound(bp);
  }
  if (bsp->name != NULL) { /* libere le nom	*/
    free((char *)bsp->name);
    bsp->name = NULL;
  }
  if (bsp->bound.ptr != NULL) { /* libere le tableau	*/
    free((char *)bsp->bound.ptr);
    bsp->bound.ptr = NULL;
  }
}

/*
 * La procedure "malloc_Bound" alloue une surface.
 * Les champs "bound.face.edge" ne sont pas utilises.
 * Entree :
 * bp		Surface a allouer.
 * type		Type de la surface.
 * polygonal	Booleen indiquant si la surface est polygonale.
 * fn		Nombre de faces  de la surface.
 * pn		Nombre de points de la surface.
 */
void malloc_Bound(Bound *bp, Type type, int polygonal, Index fn, Index pn)
{
  static char proc_name[] = "malloc_Bound";

  if ((bp->face.nbr = fn) == 0) /* faces	*/
    bp->face.ptr = NULL;
  else if ((bp->face.ptr = (Face *)malloc(fn * sizeof(Face))) == NULL) {
    perror(proc_name);
    exit(1);
  }

  if ((bp->point.nbr = pn) == 0) /* points	*/
    bp->point.ptr = NULL;
  else if ((bp->point.ptr = (Point3f *)malloc(pn * sizeof(Point3f))) == NULL) {
    perror(proc_name);
    exit(1);
  }

#ifdef face_normal
  /* normales aux sommets	*/
  if ((bp->normal.nbr = (bp->is_polygonal ? 0 : pn)) == 0)
    bp->normal.ptr = NULL;
  else if ((bp->normal.ptr = (Vector *)malloc(pn * sizeof(Vector))) == NULL) {
    perror(proc_name);
    exit(1);
  }
#endif /* face_normal */

  bp->type = type;
  bp->is_display = TRUE;
  bp->is_polygonal = (unsigned)polygonal;
}

/*
 * La procedure "malloc_huge_Bound" alloue une surface de taille maximale.
 * La surface est adaptee pour la reception de tout type de surface.
 * La surface allouee peut etre utilisee comme une surface de travail.
 * Sa taille est definie par les macros "..._NBR" de "world.h".
 * FACE_NBR	: son nombre de faces
 * POINT_NBR	: son nombre de points
 * VECTOR_NBR	: son monbre de vecteurs
 * VERTEX_NBR	: son nombre de sommets par face.
 * La particularite de la surface vient de l'allocation en une seule fois
 * d'un tableau de sommets. Les polygones des faces ne sont pas initialiser,
 * exepte celui de la premiere face qui est la base du tableau des sommets.
 * Les champs "bound.face.edge" ne sont pas utilises.
 * Entree :
 * bp		Surface maximale a allouer.
 */
void malloc_huge_Bound(Bound *bp)
{

#ifdef face_normal
  malloc_Bound(bp, (Type)BND_NULL, FALSE, FACE_NBR, POINT_NBR);
#else
  malloc_Bound(bp, (Type)BND_NULL, TRUE, FACE_NBR, POINT_NBR);
#endif /* face_normal */
  if ((bp->face.ptr->vertex.ptr = (Index *)malloc(FACE_NBR * VERTEX_NBR * sizeof(Index))) == NULL) {
    static char proc_name[] = "malloc_Huge_Bound";
    perror(proc_name);
    exit(1);
  }
}

/*
 * La procedure "malloc_Bound_scene" alloue une scene de surfaces.
 * Stocke le nom de la scene et alloue l'espace memoire necessaire.
 * Les champs "bound.face.edge" ne sont pas utilises.
 * Entree :
 * bsp		Scene a allouer.
 * name		Nom de la scene.
 * bn		Nombre de surfaces de la scene.
 */
void malloc_Bound_scene(Bound_scene *bsp, const char *name, Index bn)
{
  static char proc_name[] = "malloc_Bound_scene";

  if ((bsp->name = (char *)malloc((strlen(name) + 1) * sizeof(char))) == NULL) {
    perror(proc_name);
    exit(1);
  }
  if ((bsp->bound.nbr = bn) == 0)
    bsp->bound.ptr = NULL;
  else if ((bsp->bound.ptr = (Bound *)malloc(bn * sizeof(Bound))) == NULL) {
    perror(proc_name);
    exit(1);
  }
  strcpy(bsp->name, name);
  bsp->bound.nbr = 0;
}

#endif
