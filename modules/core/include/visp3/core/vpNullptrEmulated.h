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
 */

#ifndef VP_NULLPTR_EMULATED_H
#define VP_NULLPTR_EMULATED_H

#include <visp3/core/vpConfig.h>

// Note that on ubuntu 12.04 __cplusplus is equal to 1 that's why in the next line we consider __cplusplus <= 199711L
// and not __cplusplus == 199711L
#if (!defined(VISP_HAVE_NULLPTR)) && (__cplusplus <= 199711L)

// Inspired from this thread https://stackoverflow.com/questions/24433436/compile-error-nullptr-undeclared-identifier
// Does the emulation of nullptr when not available with cxx98
const
class nullptr_t
{
public:
  template<class T>
  inline operator T *() const // convertible to any type of null non-member pointer...
  {
    return 0;
  }

  template<class C, class T>
  inline operator T C:: *() const   // or any type of null member pointer...
  {
    return 0;
  }

private:
  void operator&() const;  // Can't take address of nullptr

} nullptr = {};

#endif
#endif
