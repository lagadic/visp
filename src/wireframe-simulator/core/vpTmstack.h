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
