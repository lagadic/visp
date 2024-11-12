/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Le module "myio.c" contient les procedures d'entree/sortie
 * des types definis dans le module "my.h".
 * Les entrees non specifiees sont effectuees
 * sur le fichier "source" du module "lex.c".
 * Pour les mots cles des "fprintf_..." voir "token.c".
 */

#ifndef VP_MY_IO_H
#define VP_MY_IO_H

#include <visp3/core/vpConfig.h>                    // for BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/robot/vpWireFrameSimulatorTypes.h>  // for Index, Type

BEGIN_VISP_NAMESPACE
void fscanf_float(float *fp);
void fscanf_Index(Index *ip);
void fscanf_int(int *ip);
void fscanf_string(char **str);
void fscanf_Type(Type *ip);
END_VISP_NAMESPACE
#endif
#endif
