/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Wire frame simulator
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/

#ifndef vpWireFrameSimulatorTypes_h
#define vpWireFrameSimulatorTypes_h

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

typedef unsigned short Index;
typedef char Type;
typedef float Matrix[4][4];

/*
 *				MATRIX
 *				______
 *
 * Matrice homogene ou non.
 *				|   Rotation  | 0 |
 * Matrice non homogene = 	|     3x3     | 0 |
 *				|-------------| 0 |
 *				| Translation | 1 |
 */
typedef float Matrix[4][4];

#define DEFAULT_VSIZE 4

/*
 * Vertex_list :
 * Pour optimiser l'allocation et la liberation memoire d'une liste de
 * sommets: si (nbr > DEFAULT_VSIZE) |	alors ptr est alloue et libere
 * dynamiquement |	sinon ptr = tbl fsi;
 */
typedef struct {
  Index nbr;  /* nombre de sommets	*/
  Index *ptr; /* liste  dynamique	*/
  Index tbl[DEFAULT_VSIZE];
} Vertex_list;

typedef struct {
  unsigned is_polygonal : 1; /* face polygonale	*/
  unsigned is_visible : 1;   /* face affichable	*/
#ifdef face_edge
  Edge_list edge;     /* liste d'aretes	*/
#endif                // face_edge
  Vertex_list vertex; /* liste de sommets	*/
#ifdef face_normal
  Vector normal; /* vecteur normal	*/
#endif           // face_normal
} Face;

typedef struct {
  Index nbr; /* nombre de faces	*/
  Face *ptr; /* liste  dynamique	*/
} Face_list;

typedef struct {
  float x, y, z;
} Point3f;

typedef struct {
  Index nbr;    /* nombre de points	*/
  Point3f *ptr; /* liste  dynamique	*/
} Point3f_list;

typedef struct {
  unsigned is_display : 1;   /* surface affichable	*/
  unsigned is_polygonal : 1; /* surface polyedrique	*/
  Type type;                 /* type de la primitive	*/
#ifdef face_edge
  Edge_list edge;     /* liste d'aretes	*/
#endif                // face_edge
  Face_list face;     /* liste de faces	*/
  Point3f_list point; /* points aux sommets	*/
#ifdef face_normal
  Vector_list normal; /* normales aux sommets	*/
#endif                // face_normal
} Bound;

typedef struct {
  Index nbr;  /* nombre de surfaces	*/
  Bound *ptr; /* liste  dynamique	*/
} Bound_list;

typedef struct {
  char *name;       /* nom de la scene	*/
  Bound_list bound; /* liste de surfaces	*/
} Bound_scene;

#endif
#endif
