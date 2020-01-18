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
 * Le module "token.h" contient les Macros et les Types
 * des jetons de l'analyseur lexicale .
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/
#ifndef vpToken_H
#define vpToken_H

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/robot/vpWireFrameSimulatorTypes.h>

typedef struct {
  const char *ident; /* identifateur 	*/
  Index token;       /* code du jeton 	*/
} Keyword;

#define T_EOF 256
#define T_FLOAT 257
#define T_IDENT 258
#define T_INT 259
#define T_STRING 260

extern float myfloat;
extern int myint;
extern int mylength;
extern int mylineno;
extern char *mytext;
extern Keyword keyword_tbl[];

/*
 * Jetons superieurs a 270 (voir "../mylex/token.h").
 */
#define T_ABOVE 270
#define T_BACK 271
#define T_BELOW 272
#define T_BOUND 273
#define T_COP 274
#define T_DEPTH 275
#define T_EXIT 276
#define T_FACE_LIST 277
#define T_FILE 278
#define T_FRONT 279
#define T_IMAGE 280
#define T_LEFT 281
#define T_NONE 282
#define T_ORIGIN 283
#define T_PARALLEL 284
#define T_PERSPECTIVE 285
#define T_POINT_LIST 286
#define T_REMOVE 287
#define T_RIGHT 288
#define T_SIZE 289
#define T_TYPE 290
#define T_VIEW 291
#define T_VPN 292
#define T_VRP 293
#define T_VUP 294
#define T_WINDOW 295
#define T_NULL 296

#endif
#endif
