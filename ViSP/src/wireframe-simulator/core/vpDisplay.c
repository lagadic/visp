/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Le module "display.c" contient les procedures de d'affichage
 * des scenes de modele geometrique surfacique.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<stdio.h>
#include	<stdlib.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
/*
#include	<suntool/sunview.h>
*/
#ifdef	suncgi
#include	"cgidefs.h"
#endif	/* suncgi	*/

#include	<visp/vpMy.h>
#include	<visp/vpArit.h>
//#include	"graph.h"

#include	<visp/vpBound.h>
#include	<visp/vpView.h>
#include	<visp/vpImstack.h>
#include	<visp/vpRfstack.h>
#include	<visp/vpVwstack.h>


/*
 * POINT2I	: 
 * Tableau de points 2D dans l'espace ecran servant a l'affichage fil-de-fer.
 *
 * RENAME	:
 * Tableau de renommage des sommets ou tableau de compteurs associes aux points.
 */
Point2i	*point2i = (Point2i *) NULL;
Point2i	*listpoint2i = (Point2i *) NULL;
static	int	*rename_jlc  = (int *) NULL;


/*
 * La procedure "open_display" alloue et initialise les variables utilisees
 * par le mode "display".
 */
void open_display ()
{
  static	char	proc_name[] = "open_display";

  if ((point2i = (Point2i *) malloc (POINT_NBR*sizeof (Point2i))) == NULL
  || (listpoint2i = (Point2i *) malloc (10*sizeof (Point2i))) == NULL
  || (rename_jlc  = (int *) malloc (POINT_NBR * sizeof (int))) == NULL)
  {
    perror (proc_name);
    exit (1);
  }
}

/*
 * La procedure "close_display" libere les variables utilisees par le mode
 * "display".
 */
void close_display ()
{
  free ((char *) point2i);
  free ((char *) listpoint2i);
  free ((char *) rename_jlc);
  point2i = (Point2i *) NULL;
  listpoint2i = (Point2i *) NULL;
  rename_jlc  = (int *) NULL;
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
void point_3D_2D (Point3f *p3, Index size, int xsize, int ysize, Point2i *p2)
{
	Point3f	*pend = p3 + size;	/* borne de p3	*/
	float		xdiv2 = ((float) xsize) / (float)2.0;
	float		ydiv2 = ((float) ysize) / (float)2.0;

	for (; p3 < pend; p3++, p2++) {
		if (/*p3->z >= -50.0*/1) {	/* point visible	*/
			p2->x = (int) ((1.0 + p3->x) * xdiv2);
			p2->y = (int) ((1.0 - p3->y) * ydiv2);
		}
		else 			/* point invisible	*/
			p2->x = p2->y = -1;
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
void set_Bound_face_display (Bound *bp, Byte b)
{
	Face		*fp   = bp->face.ptr;
	Face		*fend = fp + bp->face.nbr;
	Point3f	*pp   = bp->point.ptr;

	for (; fp < fend; fp++) {
		Index	 *vp;
		Point3f *p0;	/* premier sommet	*/
		Point3f *p1;	/* second  sommet	*/
		Point3f *p2;	/* dernier sommet	*/

		fp->is_visible = TRUE;
		if (b == IS_INSIDE) continue;
		vp = fp->vertex.ptr;
		p0 = pp + *vp;	
		p1 = pp + *(vp + 1);
		p2 = pp + *(vp + fp->vertex.nbr - 1);
		if (b & IS_ABOVE) {
			fp->is_visible =  ((p1->z - p0->z) * (p2->x - p0->x) 
					>= (p1->x - p0->x) * (p2->z - p0->z));
		}
		if (! fp->is_visible) continue;
		if (b & IS_BELOW) {
			fp->is_visible =  ((p1->z - p0->z) * (p2->x - p0->x) 
					<= (p1->x - p0->x) * (p2->z - p0->z));
		}
		if (! fp->is_visible) continue;
		if (b & IS_RIGHT) {
			fp->is_visible =  ((p1->y - p0->y) * (p2->z - p0->z) 
					>= (p1->z - p0->z) * (p2->y - p0->y));
		}
		if (! fp->is_visible) continue;
		if (b & IS_LEFT) {
			fp->is_visible =  ((p1->y - p0->y) * (p2->z - p0->z) 
					<= (p1->z - p0->z) * (p2->y - p0->y));
		}
		if (! fp->is_visible) continue;
		if (b & IS_BACK) {
			fp->is_visible =  ((p1->x - p0->x) * (p2->y - p0->y) 
					>= (p1->y - p0->y) * (p2->x - p0->x));
		}
		if (! fp->is_visible) continue;
		if (b & IS_FRONT) {
			fp->is_visible =  ((p1->x - p0->x) * (p2->y - p0->y) 
					<= (p1->y - p0->y) * (p2->x - p0->x));
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
void wireframe_Face (Face *fp, Point2i *pp)
{
//	extern Window id_window;

	Index	*vp   = fp->vertex.ptr;
	Index	*vend = vp + fp->vertex.nbr;
	Point2i *cp   = listpoint2i;

	if (fp->vertex.nbr < 2) return;
	if (fp->vertex.nbr > 10)
	{
		printf("pb malloc listpoint2i (display.c)\n"); return;
	}
	for (; vp < vend; vp++, cp++) {	
		SET_COORD2(*cp,  pp[*vp].x,  pp[*vp].y);
	} 
}

#endif
