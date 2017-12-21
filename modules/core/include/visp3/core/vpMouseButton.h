/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Color definition.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpMouseButton_h
#define vpMouseButton_h

#include <visp3/core/vpConfig.h>

/*!
  \class vpMouseButton
  \ingroup group_gui_display
  \brief Class that defines mouse button identifiers.
*/
class VISP_EXPORT vpMouseButton
{
public:
  typedef enum {
    button1 = 1, /*!< Mouse left button. */
    button2 = 2, /*!< Mouse middle button, or roll. */
    button3 = 3, /*!< Mouse right button. */
    none = 0     /*!< No button. */
  } vpMouseButtonType;
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
