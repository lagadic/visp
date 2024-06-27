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
 */

#ifndef VP_DISPLAY_FACTORY_H
#define VP_DISPLAY_FACTORY_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

BEGIN_VISP_NAMESPACE
/**
 * \ingroup group_gui_display
*/
namespace vpDisplayFactory
{
/**
 * \brief Return a newly allocated vpDisplay specialization
 * if a GUI library is available or nullptr otherwise.
 *
 * \return A newly allocated vpDisplay specialization
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
 * \brief Return a newly allocated vpDisplay specialization initialized with \b I
 * if a GUI library is available or nullptr otherwise.
 *
 * \tparam T : Any type that an image can handle and that can be displayed.
 * \param[in] I : The image the display must be initialized with.
 *
 * \return A newly allocated vpDisplay specialization initialized with \b I
 * if a GUI library is available or nullptr otherwise.
 */
template<typename T>
vpDisplay *displayFactory(vpImage<T> &I)
{
#if defined(VISP_HAVE_DISPLAY)
#ifdef VISP_HAVE_X11
  return new vpDisplayX(I);
#elif defined(VISP_HAVE_GDI)
  return new vpDisplayGDI(I);
#elif defined(HAVE_OPENCV_HIGHGUI)
  return new vpDisplayOpenCV(I);
#elif defined(VISP_HAVE_GTK)
  return new vpDisplayGTK(I);
#elif defined(VISP_HAVE_D3D9)
  return new vpDisplayD3D(I);
#endif
#else
  (void)I;
  return nullptr;
#endif
}

/**
 * \brief Return a newly allocated vpDisplay specialization initialized with \b I
 * if a GUI library is available or nullptr otherwise.
 *
 * \tparam T : Any type that an image can handle and that can be displayed.
 * \param[in] I : The image the display must be initialized with.
 * \param[in] scale_type : If this parameter is set to:
 * - vpDisplay::SCALE_AUTO, the display size is adapted to ensure the image is fully displayed in the screen;
 * - vpDisplay::SCALE_DEFAULT or vpDisplay::SCALE_1, the display size is the same than the image size.
 * - vpDisplay::SCALE_2, the display size is down scaled by 2 along the lines and the columns.
 * - vpDisplay::SCALE_3, the display size is down scaled by 3 along the lines and the columns.
 * - vpDisplay::SCALE_4, the display size is down scaled by 4 along the lines and the columns.
 * - vpDisplay::SCALE_5, the display size is down scaled by 5 along the lines and the columns.
 *
 * \return A newly allocated vpDisplay specialization initialized with \b I
 * if a GUI library is available or nullptr otherwise.
 */
template<typename T>
vpDisplay *displayFactory(vpImage<T> &I, vpDisplay::vpScaleType scale_type)
{
#if defined(VISP_HAVE_DISPLAY)
#ifdef VISP_HAVE_X11
  return new vpDisplayX(I, scale_type);
#elif defined(VISP_HAVE_GDI)
  return new vpDisplayGDI(I, scale_type);
#elif defined(HAVE_OPENCV_HIGHGUI)
  return new vpDisplayOpenCV(I, scale_type);
#elif defined(VISP_HAVE_GTK)
  return new vpDisplayGTK(I, scale_type);
#elif defined(VISP_HAVE_D3D9)
  return new vpDisplayD3D(I, scale_type);
#endif
#else
  (void)I;
  (void)scale_type;
  return nullptr;
#endif
}
}
END_VISP_NAMESPACE
#endif
