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
 * Le module "boundio.c" contient les procedures d'entree/sortie
 * des types definis dans le module "bound.h".
 * Les entrees non specifiees sont effectuees
 * sur le fichier "source" de "lex.c".
 * Pour les mots cles des "fprintf_..." voir "token.c".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#include "vpArit.h"
#include "vpBoundio.h"
#include "vpLex.h"
#include "vpMy.h"
#include "vpSkipio.h"
#include "vpToken.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

/*
 * La procedure "fscanf_Bound" lit en ascii une surface.
 * Entree :
 * bp		Surface a lire.
 */
void fscanf_Bound(Bound *bp)
{
  /* Lecture du type polygonale de la surface.	*/

  skip_keyword(T_TYPE, "bound: keyword \"type\" expected");
  if (lex() != T_INT)
    lexerr("start", "bound_type: boolean expected (0=FALSE|~0=TRUE)", NULL);
  bp->is_polygonal = (myint ? 1 : 0);

  /* Lecture de la liste de points de la surface.	*/

  skip_keyword(T_POINT_LIST, "bound: keyword \"point_list\" expected");
  pusherr("bound_point_list: ");
  fscanf_Point3f_list(&bp->point);
  poperr();

  /* Lecture de la liste de faces de la surface.	*/

  skip_keyword(T_FACE_LIST, "bound: keyword \"face_list\" expected");
  pusherr("bound_face_list: ");
  fscanf_Face_list(&bp->face);
  poperr();
}

/*
 * La procedure "fscanf_Face_list" lit en ascii une liste de faces.
 * Entree :
 * lp		Liste de faces a lire.
 */
void fscanf_Face_list(Face_list *lp)
{
  Face *fp;   /* face courante	*/
  Face *fend; /* borne de "fp"	*/

  /* Lecture du nombre de faces de la liste	*/

  if (lex() != T_INT)
    lexerr("start", "integer expected (number of faces)", NULL);
  lp->nbr = (Index)myint;

  /* Allocation dynamique de la liste de faces.	*/

  if (lp->nbr == 0)
    lp->ptr = NULL;
  else if ((lp->ptr = (Face *)malloc(lp->nbr * sizeof(Face))) == NULL) {
    static char proc_name[] = "fscanf_Face_list";
    perror(proc_name);
    exit(1);
  }

  /* Lecture des faces de la liste une a une.	*/

  fp = lp->ptr;
  fend = fp + lp->nbr;
  for (; fp < fend; fp++) {
    Vertex_list *vlp = &fp->vertex;
    Index *vp;   /* sommet courant	*/
    Index *vend; /* borne de "vp"	*/

    /* Lecture du type polygonale de la face.	*/

    if (lex() != T_INT)
      lexerr("start", "boolean expected (0=FALSE|~0=TRUE)", NULL);
    fp->is_polygonal = (myint ? 1 : 0);

    /* Lecture du nombre de sommets de la face.	*/

    if (lex() != T_INT)
      lexerr("start", "integer expected (number of vertices)", NULL);
    vlp->nbr = (Index)myint;

    /* Allocation dynamique du polygone de la face.	*/

    if (vlp->nbr <= DEFAULT_VSIZE)
      vlp->ptr = vlp->tbl;
    else if ((vlp->ptr = (Index *)malloc(vlp->nbr * sizeof(Index))) == NULL) {
      static char proc_name[] = "fscanf_Face_list";
      perror(proc_name);
      exit(1);
    }

    /* Lecture des sommets de la face un a un.	*/

    vp = vlp->ptr;
    vend = vp + vlp->nbr;
    for (; vp < vend; *vp++ = (Index)myint)
      if (lex() != T_INT)
        lexerr("start", "integer expected (index of points 3D)", NULL);
  }
}

/*
 * La procedure "fscanf_Point_list" lit en ascii une liste de points 3D.
 * Entree :
 * lp		Liste de points a lire.
 */
void fscanf_Point3f_list(Point3f_list *lp)
{
  static const char *err_tbl[] = {"float expected (coordinate ", " of point)"};
  Point3f *pp;   /* point courant	*/
  Point3f *pend; /* borne de "pp"	*/

  /* Lecture du nombre de points de la liste.	*/

  if (lex() != T_INT)
    lexerr("start", "integer expected (number of points 3D)", NULL);
  lp->nbr = (Index)myint;
  /* FC printf("nbr %d\n",lp->nbr); */
  /* Allocation dynamique la liste de points.	*/

  if (lp->nbr == 0)
    lp->ptr = NULL;
  else if ((lp->ptr = (Point3f *)malloc(lp->nbr * sizeof(Point3f))) == NULL) {
    static const char proc_name[] = "fscanf_Point3f_list";
    perror(proc_name);
    exit(1);
  }

  /* Lecture des points de la liste un a un.	*/

  pp = lp->ptr;
  pend = pp + lp->nbr;
  for (; pp < pend; pp++) {
    int t;

    if ((t = lex()) != T_FLOAT && t != T_INT)
      lexerr("start", err_tbl[0], "X", err_tbl[1], NULL);
    /* FC printf("X %d %f\n",myint, myfloat); */
    pp->x = (t == T_INT) ? (float)myint : myfloat;

    if ((t = lex()) != T_FLOAT && t != T_INT)
      lexerr("start", err_tbl[0], "Y", err_tbl[1], NULL);
    /* FC printf("Y %d %f\n",myint, myfloat); */
    pp->y = (t == T_INT) ? (float)myint : myfloat;

    if ((t = lex()) != T_FLOAT && t != T_INT)
      lexerr("start", err_tbl[0], "Z", err_tbl[1], NULL);
    /* FC printf("Z %d %f\n",myint, myfloat); */
    pp->z = (t == T_INT) ? (float)myint : myfloat;
  }
}

#endif
