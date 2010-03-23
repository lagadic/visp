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
 * Le module "view.h" contient les Macros et les types
 * des parametres de visualisation et de transformation 3D.
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/
 
#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*
 * Macros de numerotation des 6 plans de decoupage :
 * - Les 6 plans de clipping definissent le volume canonique 
 *   de la pyramide de vision dans lequel la scene est visible.
 * - les 6 plans ont pour equations :
 * Plan dessus	:  W = Y
 * Plan dessous	: -W = Y
 * Plan droit	:  W = X
 * Plan gauche	: -W = X
 * Plan arriere	:  W = Z
 * Plan avant	:  W = 0
 */
#define	PLANE_ABOVE	0
#define	PLANE_BELOW	1
#define	PLANE_RIGHT	2
#define	PLANE_LEFT	3
#define	PLANE_BACK	4
#define	PLANE_FRONT	5
#define	PLANE_NBR	6

/*
 * Macros de positionnement des points 4D :
 * Le positionnement d'un point 4D dans l'espace de l'observateur virtuel
 * se fait par rapport aux 6 plans de decoupage.
 * A chaque point 4D on associe 6 bits, un par plan de decoupage.
 */
#define	IS_INSIDE	0x00
#define	IS_ABOVE	0x01
#define	IS_BELOW	0x02
#define	IS_RIGHT	0x04
#define	IS_LEFT		0x08
#define	IS_BACK		0x10
#define	IS_FRONT	0x20

#define	DEFAULT_REMOVE	IS_INSIDE

#define	PARALLEL	0
#define	PERSPECTIVE	1

#define	DEFAULT_EYE	{ 0.0, 0.0, 1.0	}
#define	DEFAULT_TARGET	{ 0.0, 0.0, 0.0	}
#define	DEFAULT_FOCAL	1.0
#define	DEFAULT_ANGLE	45.0
#define	DEFAULT_TWIST	0.0
#define	DEFAULT_SPEED	0.0
#define	DEFAULT_CAMERA	{ DEFAULT_EYE,	DEFAULT_TARGET,\
			  DEFAULT_FOCAL,DEFAULT_ANGLE, DEFAULT_TWIST,\
			  DEFAULT_SPEED }

#define	DEFAULT_COP	{ 0.0, 0.0, 1.0	}
#define	DEFAULT_VRP	{ 0.0, 0.0, 0.0	}
#define	DEFAULT_VPN	{ 0.0, 0.0,-1.0	}
#define	DEFAULT_VUP	{ 0.0, 1.0, 0.0	}
#define	DEFAULT_VWD	{-1.0, 1.0,-1.0, 1.0 }
#define	DEFAULT_DEPTH	{ 0.0, 1.0	}
#define	DEFAULT_TYPE	PERSPECTIVE

#define	DEFAULT_VIEW	{ DEFAULT_TYPE,\
			  DEFAULT_COP, DEFAULT_VRP,\
			  DEFAULT_VPN, DEFAULT_VUP,\
			  DEFAULT_VWD, DEFAULT_DEPTH }

#define	DEFAULT_WC	{ 1.0, 0.0, 0.0, 0.0,\
			  0.0, 1.0, 0.0, 0.0,\
			  0.0, 0.0, 1.0, 0.0,\
			  0.0, 0.0, 0.0, 1.0 }


/*
 *			CAMERA PARAMETERS
 *			_________________
 *
 * La structure "Camera_parameters" definit les parametres de la camera.
 * eye		Position de l'oeil ou de la camera.
 * target	Position de la cible ou du point visee dans la scene.
 * focal	Distance eye-target
 * angle	Angle d'ouverture en degres.
 * twist	Angle de rotation sur l'axe de visee (eye,target) en degres.
 * speed	Vitesse sur l'axe de visee (eye,target).
 */
typedef	struct	{
	Point3f	eye;		/* position de l'observateur	*/
	Point3f	target;		/* point vise			*/
	float	focal;		/* focale de la camera		*/
	float	angle;		/* angle d'ouverture		*/
	float	twist;		/* rotation sur l'axe de visee	*/
	float	speed;		/* vitesse  sur l'axe de visee	*/
} Camera_parameters;

typedef	struct	{
	float	umin,umax;	/* bords gauche et droit	*/
	float	vmin,vmax;	/* bords inferieur et superieur */
} View_window;

typedef	struct	{
	float	front;		/* plan avant ("hither")	*/
	float	back;		/* plan arriere ("yon")		*/
} View_depth;

typedef	struct	{
	Type		type;	/* type de la  projection	*/
	Point3f		cop;	/* centre de projection		*/
	Point3f		vrp;	/* point de reference de visee	*/
	Vector		vpn;	/* vecteur nomal au plan	*/
	Vector		vup;	/* vecteur indiquant le "haut"	*/
	View_window	vwd;	/* fenetre de projection	*/
	View_depth 	depth;	/* profondeurs de decoupages	*/
} View_parameters;

#endif


