/****************************************************************************
 *
 * $Id: vpColor.cpp,v 1.5 2007-12-18 15:14:21 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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
 * Color definition.
 *
 * Authors:
 * Nicolas Melchior
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpColor.h>

/*!< Predefined black color with R=G=B=0 and identifier vpColor::id_black. */
vpColor const vpColor::black  = vpColor(0, 0, 0, id_black);
/*!< Predefined white color with R=G=B=255 and identifier vpColor::id_white. */
vpColor const vpColor::white  = vpColor(255, 255, 255, id_white);
/*!< Predefined red color with R=255 and G=B=0 and identifier
   vpColor::id_red. */
vpColor const vpColor::red    = vpColor(255, 0, 0, id_red);
/*!< Predefined green color with G=255 and R=B=0 and identifier
   vpColor::id_green. */
vpColor const vpColor::green  = vpColor(0, 255, 0, id_green);
/*!< Predefined blue color with R=G=0 and B=255 and identifier
   vpColor::id_blue. */
vpColor const vpColor::blue   = vpColor(0, 0, 255, id_blue);
/*!< Predefined yellow color with R=G=255 and B=0 and identifier
   vpColor::id_yellow. */
vpColor const vpColor::yellow = vpColor(255, 255, 0, id_yellow);
/*!< Predefined cyan color with R=0 and G=B=255 and identifier
   vpColor::id_cyan. */
vpColor const vpColor::cyan   = vpColor(0, 255, 255, id_cyan);
/*!< Predefined orange color with R=255, G=165 and B=0 and identifier
   vpColor::id_orange. */
vpColor const vpColor::orange = vpColor(255, 165, 0, id_orange);

/*!< Predefined none color with R=G=B=0 and identifier vpColor::id_unknown. */
vpColor const vpColor::none = vpColor(0, 0, 0, id_unknown);

