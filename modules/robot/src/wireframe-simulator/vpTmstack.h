/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Le module "tmstack.h" contient les macros, les types et
 * les specifications des procedures de gestion de la pile
 * de matrices de transformation (Transformation Matrix STACK).
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/

#ifndef vpTmstack_h
#define vpTmstack_h
#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

Matrix	*get_tmstack (void);
void	load_tmstack (Matrix m);
void	pop_tmstack (void);
void	push_tmstack (void);
void	swap_tmstack (void);

void	postmult_tmstack (Matrix m);
void	postrotate_tmstack (Vector *vp);
void	postscale_tmstack (Vector *vp);
void	posttranslate_tmstack (Vector *vp);
void	premult_tmstack (Matrix m);
void	prerotate_tmstack (Vector *vp);
void	prescale_tmstack (Vector *vp);
void	pretranslate_tmstack (Vector *vp);

#endif
#endif
