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
 * Le module "token.h" contient les Macros et les Types
 * des jetons de l'analyseur lexicale .
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS


typedef	struct	{
	char	*ident;	/* identifateur 	*/
	Index	token;	/* code du jeton 	*/
} Keyword;

#define	T_EOF 		256
#define	T_FLOAT 	257
#define	T_IDENT 	258
#define	T_INT 		259
#define	T_STRING 	260

extern	float	myfloat;
extern	int	myint;
extern	int	mylength;
extern	int	mylineno;
extern	char	*mytext;
extern	Keyword	keyword_tbl[];


/*
 * Jetons superieurs a 270 (voir "../mylex/token.h").
 */
#define	T_ABOVE		270
#define	T_BACK		271
#define	T_BELOW		272
#define	T_BOUND		273
#define	T_COP		274
#define	T_DEPTH		275
#define	T_EXIT		276
#define	T_FACE_LIST	277
#define	T_FILE		278
#define	T_FRONT		279
#define	T_IMAGE		280
#define	T_LEFT		281
#define	T_NONE		282
#define	T_ORIGIN	283
#define	T_PARALLEL	284
#define	T_PERSPECTIVE	285
#define	T_POINT_LIST	286
#define	T_REMOVE	287
#define	T_RIGHT		288
#define	T_SIZE		289
#define	T_TYPE		290
#define	T_VIEW		291
#define	T_VPN		292
#define	T_VRP		293
#define	T_VUP		294
#define	T_WINDOW	295

#endif
