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

#ifndef vpClipping_h
#define vpClipping_h

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include "vpArit.h"
#include "vpBound.h"
#include "vpMy.h"

void open_clipping(void);
void open_clipping(void);
void close_clipping(void);
Bound *clipping_Bound(Bound *bp, Matrix m);
void set_Point4f_code(Point4f *p4, int size, Byte *cp);
Byte where_is_Point4f(Point4f *p4);

#endif
#endif
