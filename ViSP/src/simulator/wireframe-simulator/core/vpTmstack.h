/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

extern	Matrix	*get_tmstack ();
extern	void	load_tmstack ();
extern	void	pop_tmstack ();
extern	void	push_tmstack ();
extern	void	swap_tmstack ();

extern	void	postmult_tmstack ();
extern	void	postrotate_tmstack ();
extern	void	postscale_tmstack ();
extern	void	posttranslate_tmstack ();
extern	void	premult_tmstack ();
extern	void	prerotate_tmstack ();
extern	void	prescale_tmstack ();
extern	void	pretranslate_tmstack ();

#endif
