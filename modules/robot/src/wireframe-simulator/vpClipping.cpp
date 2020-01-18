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
 * Le module "clipping.c" contient les procedures de decoupage
 * d'une scene 3D par l'algorithme de Sutherland et Hodgman.
 * Pour plus de reseignements, voir :
 * I. Sutherland, E. Hodgman, W. Gary.
 * "Reentrant Polygon Clipping".
 * Communications of the ACM,
 * Junary 1974, Volume 17, Number 1, pp 32-44.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include "vpClipping.h"
#include "vpView.h"

#include <cmath>
#include <limits>
#include <stdio.h>
#include <stdlib.h>

static void inter(Byte mask, Index v0, Index v1);
static void point_4D_3D(Point4f *p4, int size, Byte *cp, Point3f *p3);

/*
 * Variables utilisees par le decoupage :
 *
 * CLIP	:
 * Surface resultat apres le decoupage.
 * La surface est adaptee pour la reception de tous les types de surfaces.
 * Sa taille est definie par les macros "..._NBR" de "bound.h".
 *
 * FACE_NBR	: son nombre de faces
 * POINT_NBR	: son nombre de points
 * VECTOR_NBR	: son monbre de vecteurs
 * VERTEX_NBR	: son nombre de sommets par face.
 *
 * La surface recoit une a une les faces decoupees.
 * La surface recoit en une fois tous les points 3D de la surface decoupee
 * par rapport aux 6 plans de la pyramide tronquee de vision.
 *
 * CODE	:
 * Tableau de booleens durant le decoupage.
 * Le tableau est initialise par les booleens indiquant le positionnement des
 * points 4D par rapport aux 6 plans de decoupage.
 * Le tableau recoit ensuite un a un les booleans des points 4D d'intersection
 * de la surface avec les 6 plans de decoupage.
 *
 * POINT4F :
 * Tableau de points durant le decoupage.
 * Le tableau est initialise par les points de la surface en entree apres
 * transforation en coordonnees homogenes 4D.
 * Le tableau recoit ensuite un a un les points 4D d'intersection de la
 * surface avec les 6 plans de la pyramide tronquee de vision.
 */
static Bound clip;        /* surface a  decouper	*/
static Byte *code;        /* tableau de bits	*/
static Point4f *point4f;  /* tableau de points 4D	*/
static Index point4f_nbr; /* nombre  de points 4D	*/

#if clip_opt
static Index *poly0, *poly1; /* polygones temporaires*/
#else
static Index *poly_tmp; /* polygone temporaire	*/
#endif /* clip_opt	*/

/*
 * La procedure "open_clipping" alloue et initialise les variables utilisees
 * par le mode "clipping".
 */
void open_clipping(void)
{
  /* alloue la surface de travail	*/
  malloc_huge_Bound(&clip);

  /* alloue les tableaux	*/
  if ((code = (Byte *)malloc(POINT_NBR * sizeof(Byte))) == NULL ||
      (point4f = (Point4f *)malloc(POINT_NBR * sizeof(Point4f))) == NULL
#ifdef clip_opt
      || (poly0 = (Index *)malloc(VERTEX_NBR * sizeof(Index))) == NULL ||
      (poly1 = (Index *)malloc(VERTEX_NBR * sizeof(Index))) == NULL) {
    static char proc_name[] = "open_clipping";
    perror(proc_name);
    exit(1);
  }
#else
      || (poly_tmp = (Index *)malloc(VERTEX_NBR * sizeof(Index))) == NULL) {
    static char proc_name[] = "open_clipping";
    perror(proc_name);
    exit(1);
  }
#endif /* clip_opt	*/
}

/*
 * La procedure "close_clipping" libere les variables utilisees par
 * le mode "clipping".
 */
void close_clipping(void)
{
  free_huge_Bound(&clip);
  free((char *)code);
  free((char *)point4f);
#ifdef clip_opt
  free((char *)poly0);
  free((char *)poly1);
#else
  free((char *)poly_tmp);
#endif /* clip_opt	*/
}

/*
 * La procedure "clipping" decoupe un polygone par rapport a un plan
 * suivant l'algorithme de Sutherland et Hodgman.
 * Entree :
 * mask		Masque du plan de decoupage pour le code.
 * vni		Nombre de sommets du polygone en entree.
 * pi		Polygone en entree.
 * po		Polygone resultat du decoupage.
 * Sortie :
 * 		Nombre de sommets du polygone resultat "po".
 */
static Index clipping(Byte mask, Index vni, Index *pi, Index *po)
{
  /*
   * vno	Nombre de sommets du polygone "po".
   * vs	Premier sommet de l'arete a decouper.
   * vp	Second  sommet de l'arete a decouper.
   * ins 	TRUE si le sommet "vs" est interieur, FALSE sinon.
   * inp 	TRUE si le sommet "vp" est interieur, FALSE sinon.
   */
  Index vno = vni;            /* nombre de sommets	*/
  Index vs = pi[vni - 1];     /* premier sommet	*/
  Byte ins = code[vs] & mask; /* code de "vs"		*/

  while (vni--) {               /* pour tous les sommets	*/
    Index vp = *pi++;           /* second sommet	*/
    Byte inp = code[vp] & mask; /* code du plan courant	*/

    if (ins == IS_INSIDE) {
      if (inp == IS_INSIDE) { /* arete interieure	*/
        *po++ = vp;
      } else { /* intersection		*/
        inter(mask, vs, vp);
        *po++ = point4f_nbr++;
      }
    } else {
      if (inp == IS_INSIDE) { /* intersection		*/
        inter(mask, vs, vp);
        *po++ = point4f_nbr++;
        *po++ = vp;
        vno++;
      } else { /* arete exterieure	*/
        vno--;
      }
    }
    vs = vp;
    ins = inp;
  }
  return (vno);
}

/*
 * La procedure "clipping_Face" decoupe une face par rapport aux 6 plans
 * de decoupage de la pyramide tronquee de vision.
 * Entree :
 * fi		Face a decouper.
 * fo		Face resultat du decoupage.
 * Sortie :
 *		Le nombre de sommets de la face resultat.
 */
static Index clipping_Face(Face *fi, Face *fo)
{
  Index *flip = poly_tmp;       /* polygone temporaire	*/
  Index *flop = fo->vertex.ptr; /* polygone resultat	*/
  Index vn = fi->vertex.nbr;    /* nombre de sommets	*/

  if ((vn = clipping(IS_ABOVE, vn, fi->vertex.ptr, flip)) != 0)
    if ((vn = clipping(IS_BELOW, vn, flip, flop)) != 0)
      if ((vn = clipping(IS_RIGHT, vn, flop, flip)) != 0)
        if ((vn = clipping(IS_LEFT, vn, flip, flop)) != 0)
          if ((vn = clipping(IS_BACK, vn, flop, flip)) != 0)
            if ((vn = clipping(IS_FRONT, vn, flip, flop)) != 0) {
              /* recopie de "fi" dans "fo"	*/
              /* fo->vertex.ptr == flop	*/
              fo->vertex.nbr = vn;
              fo->is_polygonal = fi->is_polygonal;
              fo->is_visible = fi->is_visible;
#ifdef face_normal
              fo->normal = fi->normal;
#endif /* face_normal	*/
              return (vn);
            }
  return (0);
}

/*
 * La procedure "clipping_Bound" decoupe une surface par rapport aux 6 plans
 * de decoupage de la pyramide tronquee de vision.
 * Les calculs geometriques sont effectues en coordonnees homogenes.
 * Note : Les points invisibles de la surface "clip" ont une profondeur
 *negative c'est a dire une coordonnee Z negative. Entree :
 * bp		Surface a decouper.
 * m		Matrice de projection dans le volume canonique.
 * Sortie :
 * 		Pointeur de la surface resultat "clip" si elle est visible,
 *		NULL sinon.
 */
Bound *clipping_Bound(Bound *bp, Matrix m)
{
  Face *fi = bp->face.ptr;        /* 1ere face	*/
  Face *fend = fi + bp->face.nbr; /* borne de "fi"*/
  Face *fo = clip.face.ptr;       /* face clippee	*/

  /* recopie de "bp" dans les tableaux intermediaires	*/

  point4f_nbr = bp->point.nbr;
  point_3D_4D(bp->point.ptr, (int)point4f_nbr, m, point4f);
  set_Point4f_code(point4f, (int)point4f_nbr, code);
#ifdef face_normal
  if (!(clip.is_polygonal = bp->is_polygonal))
    // bcopy (bp->normal.ptr, clip.normal.ptr,
    //	 bp->normal.nbr * sizeof (Vector));
    memmove(clip.normal.ptr, bp->normal.ptr, bp->normal.nbr * sizeof(Vector));
#endif                      /* face_normal	*/
  for (; fi < fend; fi++) { /* pour toutes les faces*/
    if (clipping_Face(fi, fo) != 0) {
      fo++; /* ajoute la face a "clip"	*/
      /*
       * Construction a la volee du future polygone.
       * dont l'espace memoire est deja alloue (voir
       * la procedure "malloc_huge_Bound").
       */
      fo->vertex.ptr = (fo - 1)->vertex.ptr + (fo - 1)->vertex.nbr;
    }
  }

  if (fo == clip.face.ptr)
    return (NULL); /* Rien a voir, circulez...	*/

  /* recopie des tableaux intermediaires dans "clip"	*/

  point_4D_3D(point4f, (int)point4f_nbr, code, clip.point.ptr);
  clip.type = bp->type;
  clip.face.nbr = (Index)(fo - clip.face.ptr);
  clip.point.nbr = point4f_nbr;
#ifdef face_normal
  if (!bp->is_polygonal)
    clip.normal.nbr = point4f_nbr;
#endif /* face_normal	*/
  return (&clip);
}

/*
 * La procedure "inter" calcule le point d'intersection "point4f[point4f_nbr]"
 * de l'arete (v0,v1) avec le plan "mask".
 * Entree :
 * mask		Mask du plan de decoupage.
 * v0		Permier sommet de l'arete.
 * v1		Second  sommet de l'arete.
 */
static void inter(Byte mask, Index v0, Index v1)
{
  Point4f *p = point4f + point4f_nbr;
  Point4f *p0 = point4f + v0;
  Point4f *p1 = point4f + v1;
  float t; /* parametre entre 0 et 1	*/

  /* calcule le point d'intersection	*/

  switch (mask) {

  case IS_ABOVE:
    /* t = (p0->w - p0->y) / ((p0->w - p0->y) - (p1->w - p1->y));	*/
    t = (p0->w - p0->y) - (p1->w - p1->y);
    // t = (t == 0) ? (float)1.0 : (p0->w - p0->y) / t;
    t = (std::fabs(t) <= std::numeric_limits<double>::epsilon()) ? (float)1.0 : (p0->w - p0->y) / t;
    PAR_COORD3(*p, t, *p0, *p1);
    p->w = p->y; /* propriete du point d'intersection	*/
    break;

  case IS_BELOW:
    /* t = (p0->w + p0->y) / ((p0->w + p0->y) - (p1->w + p1->y));	*/
    t = (p0->w + p0->y) - (p1->w + p1->y);
    // t = (t == 0) ? (float)1.0 : (p0->w + p0->y) / t;
    t = (std::fabs(t) <= std::numeric_limits<double>::epsilon()) ? (float)1.0 : (p0->w + p0->y) / t;
    PAR_COORD3(*p, t, *p0, *p1);
    p->w = -p->y; /* propriete du point d'intersection	*/
    break;

  case IS_RIGHT:
    /* t = (p0->w - p0->x) / ((p0->w - p0->x) - (p1->w - p1->x));	*/
    t = (p0->w - p0->x) - (p1->w - p1->x);
    // t = (t == 0) ? (float)1.0 : (p0->w - p0->x) / t;
    t = (std::fabs(t) <= std::numeric_limits<double>::epsilon()) ? (float)1.0 : (p0->w - p0->x) / t;
    PAR_COORD3(*p, t, *p0, *p1);
    p->w = p->x; /* propriete du point d'intersection	*/
    break;

  case IS_LEFT:
    /* t = (p0->w + p0->x) / ((p0->w + p0->x) - (p1->w + p1->x));	*/
    t = (p0->w + p0->x) - (p1->w + p1->x);
    // t = (t == 0) ? (float)1.0 : (p0->w + p0->x) / t;
    t = (std::fabs(t) <= std::numeric_limits<double>::epsilon()) ? (float)1.0 : (p0->w + p0->x) / t;
    PAR_COORD3(*p, t, *p0, *p1);
    p->w = -p->x; /* propriete du point d'intersection	*/
    break;

  case IS_BACK:
    /* t = (p0->w - p0->z) / ((p0->w - p0->z) - (p1->w - p1->z));	*/
    t = (p0->w - p0->z) - (p1->w - p1->z);
    // t = (t == 0) ? (float)1.0 : (p0->w - p0->z) / t;
    t = (std::fabs(t) <= std::numeric_limits<double>::epsilon()) ? (float)1.0 : (p0->w - p0->z) / t;
    PAR_COORD3(*p, t, *p0, *p1);
    p->w = p->z; /* propriete du point d'intersection	*/
    break;

  case IS_FRONT:
    /* t =  p0->z / (p0->z - p1->z);				*/
    t = (p0->z - p1->z);
    // t = (t == 0) ? (float)1.0 : p0->z / t;
    t = (std::fabs(t) <= std::numeric_limits<double>::epsilon()) ? (float)1.0 : p0->z / t;
    p->x = (p1->x - p0->x) * t + p0->x;
    p->y = (p1->y - p0->y) * t + p0->y;
    p->w = (p1->w - p0->w) * t + p0->w;
    p->z = (float)M_EPSILON; /* propriete du point d'intersection	*/
    break;
  }
  /* resout les problemes d'arrondis pour "where_is_Point4f"	*/
  /* p->w += (p->w < 0) ? (- M_EPSILON) : M_EPSILON;		*/
  p->w += (float)M_EPSILON;
  code[point4f_nbr] = where_is_Point4f(p); /* localise "p"	*/
#ifdef face_normal
  if (!clip.is_polygonal) {
    Vector *n0 = clip.normal.ptr + v0;
    Vector *n1 = clip.normal.ptr + v1;
    Vector *n = clip.normal.ptr + point4f_nbr;

    SET_COORD3(*n, (n1->x - n0->x) * t + n0->x, (n1->y - n0->y) * t + n0->y, (n1->z - n0->z) * t + n0->z);
  }
#endif /* face_normal	*/
}

/*
 * La procedure "point_4D_3D" transforme les points homogenes 4D  visibles
 * en points 3D par projection.
 * Note	: On marque un point 3D invisible par une profondeur negative.
 * Entree :
 * p4		Tableau de points 4D a transformer.
 * size		Taille  du tableau "p4".
 * cp		Tableau de code indiquant la visibilite des points 4D.
 * p3		Tableau de points 3D issus de la transformation.
 */
static void point_4D_3D(Point4f *p4, int size, Byte *cp, Point3f *p3)
{
  Point4f *pend = p4 + size; /* borne de p4	*/
  float w;

  for (; p4 < pend; p4++, p3++) {
    if (*cp++ == IS_INSIDE) { /* point visible	*/
      w = p4->w;

      p3->x = p4->x / w; /* projection 4D en 3D	*/
      p3->y = p4->y / w;
      p3->z = p4->z / w;
    } else { /* marque invisible	*/
      p3->z = -1.0;
    }
  }
}

/*
 * La procedure "set_Point4f_code" initialise la position des points 4D
 * par rapport a 6 plans de la pyramide tronquee de vision.
 * A chaque point est associe un code indiquant la position respective du
 * point. Entree : p4		Tableau de points 4D a localiser. size
 * Taille  du tableau "p4". cp		Tableau de codes de localisation
 * resultat.
 */
void set_Point4f_code(Point4f *p4, int size, Byte *cp)
{
  Point4f *pend = p4 + size; /* borne de p4	*/
  Byte b;                    /* code  de p4	*/

  for (; p4 < pend; p4++, *cp++ = b) {
    b = IS_INSIDE;
    if (p4->w < p4->y)
      b |= IS_ABOVE;
    else if (-p4->w > p4->y)
      b |= IS_BELOW;
    if (p4->w < p4->x)
      b |= IS_RIGHT;
    else if (-p4->w > p4->x)
      b |= IS_LEFT;
    if (p4->w < p4->z)
      b |= IS_BACK;
    else if (-0.9 > p4->z)
      b |= IS_FRONT;
  }
}

/*
 * La procedure "where_is_Point4f" localise un point 4D  par rapport aux 6
 * plans de decoupage de la pyramide tronquee de vision. Entree : p4
 * Point homogene 4D a localiser. Sortie : Code indiquant le position de "p4"
 * par rapport aux 6 plans.
 */
Byte where_is_Point4f(Point4f *p4)
{
  Byte b = IS_INSIDE; /* code de "p4"	*/

  if (p4->w < p4->y)
    b |= IS_ABOVE;
  else if (-p4->w > p4->y)
    b |= IS_BELOW;
  if (p4->w < p4->x)
    b |= IS_RIGHT;
  else if (-p4->w > p4->x)
    b |= IS_LEFT;
  if (p4->w < p4->z)
    b |= IS_BACK;
  else if (-0.9 > p4->z)
    b |= IS_FRONT;
  return (b);
}

#endif
