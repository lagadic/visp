/****************************************************************************
 *
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
 * Display Factory
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

namespace vpDisplayFactory
{
/**
* \brief Return a newly allocated vpDisplay specialisation
* if a GUI library is available or nullptr otherwise.
*
* \return vpDisplay* A newly allocated vpDisplay specialisation
* if a GUI library is available or nullptr otherwise.
*/
vpDisplay *displayFactory()
{
#if defined(VISP_HAVE_DISPLAY)
#ifdef VISP_HAVE_X11
  return new vpDisplayX();
#elif defined(VISP_HAVE_D3D9)
  return new vpDisplayD3D();
#elif defined(VISP_HAVE_GDI)
  return new vpDisplayGDI();
#elif defined(VISP_HAVE_GTK)
  return new vpDisplayGTK();
#elif defined(HAVE_OPENCV_HIGHGUI)
  return new vpDisplayOpenCV();
#endif
#else
  return nullptr;
#endif
}

/**
* \brief Return a newly allocated vpDisplay specialisation initialized with \b I
* if a GUI library is available or nullptr otherwise.
*
* \tparam T Any type that an image can handle and that can be displayed.
* \param[in] I The image the display must be initialized with.
*
* \return vpDisplay* A newly allocated vpDisplay specialisation initialized with \b I
* if a GUI library is available or nullptr otherwise.
*/
template<typename T>
vpDisplay *displayFactory(vpImage<T> &I)
{
#if defined(VISP_HAVE_DISPLAY)
#ifdef VISP_HAVE_X11
  return new vpDisplayX(I);
#elif defined(VISP_HAVE_D3D9)
  return new vpDisplayD3D(I);
#elif defined(VISP_HAVE_GDI)
  return new vpDisplayGDI(I);
#elif defined(VISP_HAVE_GTK)
  return new vpDisplayGTK(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
  return new vpDisplayOpenCV(I);
#endif
#else
  return nullptr;
#endif
}
}
