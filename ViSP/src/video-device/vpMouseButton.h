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
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/


#ifndef vpMouseButton_h
#define vpMouseButton_h


#include <visp/vpConfig.h>

/*!
  \class vpMouseButton
  \ingroup ImageGUI
  \brief Class that defines mouse button identiers.
*/
class VISP_EXPORT vpMouseButton
{
public:
  typedef enum {
    button1 = 1, /*!< Mouse left button. */
    button2 = 2, /*!< Mouse middle button, or roll. */
    button3 = 3  /*!< Mouse right button. */
  } vpMouseButtonType ;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
