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
 * Le module "display.c" contient les procedures de d'affichage
 * des scenes de modele geometrique surfacique.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include <stdio.h>
#include <stdlib.h>

#include "vpCoreDisplay.h"
#include "vpImstack.h"
#include "vpMy.h"
#include "vpRfstack.h"
#include "vpView.h"
#include "vpVwstack.h"

/*
 * POINT2I	:
 * Tableau de points 2D dans l'espace ecran servant a l'affichage fil-de-fer.
 *
 * RENAME	:
 * Tableau de renommage des sommets ou tableau de compteurs associes aux
 * points.
 */
Point2i *point2i = (Point2i *)NULL;
Point2i *listpoint2i = (Point2i *)NULL;
static int *rename_jlc = (int *)NULL;

/*
 * La procedure "open_display" alloue et initialise les variables utilisees
 * par le mode "display".
 */
void open_display(void)
{
  if ((point2i = (Point2i *)malloc(POINT_NBR * sizeof(Point2i))) == NULL ||
      (listpoint2i = (Point2i *)malloc(50 * sizeof(Point2i))) == NULL ||
      (rename_jlc = (int *)malloc(POINT_NBR * sizeof(int))) == NULL) {
    static char proc_name[] = "open_display";
    perror(proc_name);
    exit(1);
  }
}

/*
 * La procedure "close_display" libere les variables utilisees par le mode
 * "display".
 */
void close_display(void)
{
  free((char *)point2i);
  free((char *)listpoint2i);
  free((char *)rename_jlc);
  point2i = (Point2i *)NULL;
  listpoint2i = (Point2i *)NULL;
  rename_jlc = (int *)NULL;
}

/*
 * La procedure "point_3D_2D" projette les points 3D du volume canonique
 * dans l'espace image 2D.
 *
 *	Volume canonique	Espace image
 *	________________	____________
 *
 *	- 1 < X < 1		0 < X < xsize
 *	- 1 < Y < 1		0 < Y < ysize
 *	  0 < Z < 1
 *
 *	      Z < 0		X = 0, Y = -1 non significatifs.
 *
 * Entree :
 * p3		Tableau de points 3D a projeter.
 * size		Taille du tableau de points "p3".
 * xsize, ysize	Tailles de l'espace image.
 * p2		Tableau de points 2D en sortie.
 */
// static
void point_3D_2D(Point3f *p3, Index size, int xsize, int ysize, Point2i *p2)
{
  Point3f *pend = p3 + size; /* borne de p3	*/
  float xdiv2 = ((float)xsize) / (float)2.0;
  float ydiv2 = ((float)ysize) / (float)2.0;

  for (; p3 < pend; p3++, p2++) {
    p2->x = (int)((1.0 + p3->x) * xdiv2);
    p2->y = (int)((1.0 - p3->y) * ydiv2);
  }
}

/*
 * La procedure "set_Bound_face_display" marque les faces affichables
 * de la surface "bp".
 * Soit la face comportant le contour oriente suivant : (...,P2,P0,P1...).
 * La normale a la face au point P0 est obtenue par le produit vectoriel :
 *
 *				| x1 - x0	x2 - x0	|   | Nx |
 * N = (P1 - P0) ^ (P2 - P0) =	| y1 - y0	y2 - y0 | = | Ny |
 * 				| z1 - z0	z2 - z0 |   | Nz |
 *
 * La face est dans le volume canonique de vision et dans un repere gauche.
 * L'observateur est situe a l'infini dans la direction [0, 0, -1].
 * IS_ABOVE	<=>	Ny < 0,		IS_BELOW	<=>	Ny > 0.
 * IS_RIGHT	<=>	Nx < 0,		IS_LEFT		<=>	Nx > 0.
 * IS_BACK	<=>	Nz < 0,		IS_FRONT	<=>	Nz > 0.
 * Entree :
 * bp		Surface a initialiser.
 * b		Drapeaux indiquant les faces non affichables.
 */
void set_Bound_face_display(Bound *bp, Byte b)
{
  Face *fp = bp->face.ptr;
  Face *fend = fp + bp->face.nbr;
  Point3f *pp = bp->point.ptr;

  for (; fp < fend; fp++) {
    Index *vp;
    Point3f *p0; /* premier sommet	*/
    Point3f *p1; /* second  sommet	*/
    Point3f *p2; /* dernier sommet	*/

    fp->is_visible = TRUE;
    if (b == IS_INSIDE)
      continue;
    vp = fp->vertex.ptr;
    p0 = pp + *vp;
    p1 = pp + *(vp + 1);
    p2 = pp + *(vp + fp->vertex.nbr - 1);
    if (b & IS_ABOVE) {
      fp->is_visible = ((p1->z - p0->z) * (p2->x - p0->x) >= (p1->x - p0->x) * (p2->z - p0->z));
    }
    if (!fp->is_visible)
      continue;
    if (b & IS_BELOW) {
      fp->is_visible = ((p1->z - p0->z) * (p2->x - p0->x) <= (p1->x - p0->x) * (p2->z - p0->z));
    }
    if (!fp->is_visible)
      continue;
    if (b & IS_RIGHT) {
      fp->is_visible = ((p1->y - p0->y) * (p2->z - p0->z) >= (p1->z - p0->z) * (p2->y - p0->y));
    }
    if (!fp->is_visible)
      continue;
    if (b & IS_LEFT) {
      fp->is_visible = ((p1->y - p0->y) * (p2->z - p0->z) <= (p1->z - p0->z) * (p2->y - p0->y));
    }
    if (!fp->is_visible)
      continue;
    if (b & IS_BACK) {
      fp->is_visible = ((p1->x - p0->x) * (p2->y - p0->y) >= (p1->y - p0->y) * (p2->x - p0->x));
    }
    if (!fp->is_visible)
      continue;
    if (b & IS_FRONT) {
      fp->is_visible = ((p1->x - p0->x) * (p2->y - p0->y) <= (p1->y - p0->y) * (p2->x - p0->x));
    }
  }
}

/*
 * La procedure "wireframe_Face" affiche une face "fp" en "fil de fer".
 * sur la fenetre graphique de "suncgi" sur "SUN".
 * Les points des sommets de la face sont contenu dans les points "pp"
 * de la surface contenant la face.
 * Entree :
 * fp		face a afficher.
 * pp		Points de la surface contenant la face.
 */
void wireframe_Face(Face *fp, Point2i *pp)
{
  //	extern Window id_window;

  Index *vp = fp->vertex.ptr;
  Index *vend = vp + fp->vertex.nbr;
  Point2i *cp = listpoint2i;

  if (fp->vertex.nbr < 2)
    return;
  if (fp->vertex.nbr > 50) {
    printf("pb malloc listpoint2i (display.c)\n");
    return;
  }
  for (; vp < vend; vp++, cp++) {
    SET_COORD2(*cp, pp[*vp].x, pp[*vp].y);
  }
}

#endif
