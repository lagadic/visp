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
 * Le fichier "bound.h" contient les macros et le types
 * utilises par le modele geometrique surfacique polygonale 3D.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#ifndef vpBound_H
#define vpBound_H

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include "vpArit.h"
#include "vpMy.h"
#include <visp3/robot/vpWireFrameSimulatorTypes.h>

#define START_ARG 0
#define NEXT_ARG 1

#define BND_NULL (-1)

#define BND_BLOCK 0
#define BND_CARDIOIDE 1
#define BND_CONE 2
#define BND_CYLINDER 3
#define BND_SPHERE 4
#define BND_TORUS 5
#define BND_WEDGE 6

#define BND_F3 7
#define BND_F4 8

#define BND_GRID 9
#define BND_PIPE 10
#define BND_SECTION 11

#define BND_NBR 12

#define BOUND_NBR 1024
#define FACE_NBR 6144 /* Tailles de tableaux	*/
#define VERTEX_NBR 16
#define POINT_NBR 6144
#ifdef face_normal
#define VECTOR_NBR 6144
#endif // face_normal

#ifdef face_edge
typedef struct {
  Index v0, v1; /* extremites		*/
  Index f0, f1; /* faces		*/
} Edge;
#endif // face_edge

#ifdef face_edge
typedef struct {
  Index nbr; /* nombre d'aretes	*/
  Edge *ptr; /* liste  dynamique	*/
} Edge_list;
#endif // face_edge

#ifdef face_normal
typedef struct {
  Index nbr;   /* nombre de vecteurs	*/
  Vector *ptr; /* liste  dynamique	*/
} Vector_list;
#endif // face_normal

typedef struct {
  float xmin, xmax; /* bornes sur l'axe x	*/
  float ymin, ymax; /* bornes sur l'axe y	*/
  float zmin, zmax; /* bornes sur l'axe z	*/
} Bounding_box;

typedef struct {
  Index nbr;        /* nombre de scenes	*/
  Bound_scene *ptr; /* liste  dynamique	*/
} Bound_scene_list;

void free_Bound(Bound *bp);
void free_huge_Bound(Bound *bp);
void free_Bound_scene(Bound_scene *bsp);
void malloc_Bound(Bound *bp, Type type, int polygonal, Index fn, Index pn);
void malloc_huge_Bound(Bound *bp);
void malloc_Bound_scene(Bound_scene *bsp, const char *name, Index bn);

#endif
#endif
