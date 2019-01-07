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
 * Le module "rfstack.h" contient les macros, les types et
 * les specifications des procedures de gestion de la pile
 * des drapeaux d'elimination de faces (Remove Faces STACK).
 *
 * Authors:
 * Jean-Luc CORRE
 *
 *****************************************************************************/
#ifndef vpRfstack_h
#define vpRfstack_h

#include <stdio.h>
#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

void fprintf_rfstack(FILE *fp);
int *get_rfstack(void);
void load_rfstack(int i);
void pop_rfstack(void);
void push_rfstack(void);
void swap_rfstack(void);
void add_rfstack(int i);
void sub_rfstack(int i);

#endif
#endif
