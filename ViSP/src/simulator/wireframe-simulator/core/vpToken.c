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
 * Le module "token.c" contient la declaration des mots cles.
 * de l'analyseur lexical "lex".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/


#include	<visp/vpMy.h>
#include	<visp/vpToken.h>

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS


Keyword	keyword_tbl[] = {	/* tableau des mots cles termine par NULL*/
	{ "above",	T_ABOVE		},
	{ "back",	T_BACK		},
	{ "below",	T_BELOW		},
	{ "bound",	T_BOUND		},
	{ "cop",	T_COP		},
	{ "depth",	T_DEPTH		},
	{ "exit",	T_EXIT		},
	{ "face_list",	T_FACE_LIST	},
	{ "front",	T_FRONT		},
	{ "left",	T_LEFT		},
	{ "none",	T_NONE		},
	{ "parallel",	T_PARALLEL	},
	{ "perspective",T_PERSPECTIVE	},
	{ "point_list",	T_POINT_LIST	},
	{ "remove",	T_REMOVE	},
	{ "right",	T_RIGHT		},
	{ "type",	T_TYPE		},
	{ "view",	T_VIEW		},
	{ "vpn",	T_VPN		},
	{ "vrp",	T_VRP		},
	{ "vup",	T_VUP		},
	{ "window",	T_WINDOW	},
	{ NULL,		NULL		}
};

#endif
