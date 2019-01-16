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
 * Le module "viewio.c" contient les procedures d'entree/sortie
 * des types definis dans le module "view.h".
 * Les entrees non specifiees sont effectuees
 * sur le fichier source de "lex.c".
 * Pour les mots cles des "fprintf_..." voir "token.c".
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <stdio.h>

#include "vpArit.h"
#include "vpLex.h"
#include "vpMyio.h"
#include "vpSkipio.h"
#include "vpToken.h"
#include "vpViewio.h"

/*
 * La procedure "fscanf_Remove" lit en ascii les parametres d'elimination
 * des faces.
 * Entree :
 * bp		Parametres a lire.
 */
void fscanf_Remove(Byte *bp)
{
  switch (lex()) {
  case T_NONE:
    *bp = IS_INSIDE;
    break;
  case T_ABOVE:
    *bp |= IS_ABOVE;
    break;
  case T_BACK:
    *bp |= IS_BACK;
    break;
  case T_BELOW:
    *bp |= IS_BELOW;
    break;
  case T_FRONT:
    *bp |= IS_FRONT;
    break;
  case T_LEFT:
    *bp |= IS_LEFT;
    break;
  case T_RIGHT:
    *bp |= IS_RIGHT;
    break;
  default:
    lexerr("start", "remove: keyword "
                    "\"none|above|back|below|front|left|right\" expected");
    break;
  }
}

/*
 * La procedure "fscanf_View_parameters" lit en ascii les parametres
 * de visualisation.
 * Entree :
 * vp		Parametres de visualisation a lire.
 */
void fscanf_View_parameters(View_parameters *vp)
{
  /* Lecture du type de projection lors de la prise de vue.	*/

  skip_keyword(T_TYPE, "view: keyword \"type\" expected");
  switch (lex()) {
  case T_PARALLEL:
    vp->type = PARALLEL;
    break;
  case T_PERSPECTIVE:
    vp->type = PERSPECTIVE;
    break;
  default:
    lexerr("start", "view_type: keyword \"parallel|perspective\" expected");
    break;
  }

  /* Lecture du centre de projection (oeil) de la prise de vue.	*/

  skip_keyword(T_COP, "view: keyword \"cop\" expected");
  pusherr("view_cop: ");
  fscanf_Point3f(&vp->cop);
  poperr();

  /* Lecture du point de reference (cible) a la prise de vue.	*/

  skip_keyword(T_VRP, "view: keyword \"vrp\" expected");
  pusherr("view_vrp: ");
  fscanf_Point3f(&vp->vrp);
  poperr();

  /* Lecture de la direction normale au plan de projection.	*/

  skip_keyword(T_VPN, "view: keyword \"vpn\" expected");
  pusherr("view_vpn: ");
  fscanf_Vector(&vp->vpn);
  poperr();

  /* Lecture de la direction indiquant le haut de la projection.	*/

  skip_keyword(T_VUP, "view: keyword \"vup\" expected");
  pusherr("view_vup: ");
  fscanf_Vector(&vp->vup);
  poperr();

  /* Lecture de la fenetre de projection de la prise de vue.	*/

  skip_keyword(T_WINDOW, "view: keyword \"window\" expected");
  pusherr("view_window_umin: ");
  fscanf_float(&vp->vwd.umin);
  popuperr("view_window_umax: ");
  fscanf_float(&vp->vwd.umax);
  popuperr("view_window_vmin: ");
  fscanf_float(&vp->vwd.vmin);
  popuperr("view_window_vmax: ");
  fscanf_float(&vp->vwd.vmax);
  poperr();

  /* Lecture des profondeurs de decoupage avant et arriere.	*/

  skip_keyword(T_DEPTH, "view: keyword \"depth\" expected");
  pusherr("view_depth_front: ");
  fscanf_float(&vp->depth.front);
  popuperr("view_depth_back: ");
  fscanf_float(&vp->depth.back);
  poperr();
}

#endif
