/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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

#include "drawingHelpers.h"

#include <visp3/core/vpImageConvert.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

#if defined(VISP_HAVE_X11)
vpDisplayX drawingHelpers::d_Iinput;
vpDisplayX drawingHelpers::d_dIx;
vpDisplayX drawingHelpers::d_dIy;
vpDisplayX drawingHelpers::d_IcannyVisp;
vpDisplayX drawingHelpers::d_IcannyImgFilter;
#elif defined(HAVE_OPENCV_HIGHGUI)
vpDisplayOpenCV drawingHelpers::d_Iinput;
vpDisplayOpenCV drawingHelpers::d_dIx;
vpDisplayOpenCV drawingHelpers::d_dIy;
vpDisplayOpenCV drawingHelpers::d_IcannyVisp;
vpDisplayOpenCV drawingHelpers::d_IcannyImgFilter;
#elif defined(VISP_HAVE_GTK)
vpDisplayGTK drawingHelpers::d_Iinput;
vpDisplayGTK drawingHelpers::d_dIx;
vpDisplayGTK drawingHelpers::d_dIy;
vpDisplayGTK drawingHelpers::d_IcannyVisp;
vpDisplayGTK drawingHelpers::d_IcannyImgFilter;
#elif defined(VISP_HAVE_GDI)
vpDisplayGDI drawingHelpers::d_Iinput;
vpDisplayGDI drawingHelpers::d_dIx;
vpDisplayGDI drawingHelpers::d_dIy;
vpDisplayGDI drawingHelpers::d_IcannyVisp;
vpDisplayGDI drawingHelpers::d_IcannyImgFilter;
#elif defined(VISP_HAVE_D3D9)
vpDisplayD3D drawingHelpers::d_Iinput;
vpDisplayD3D drawingHelpers::d_dIx;
vpDisplayD3D drawingHelpers::d_dIy;
vpDisplayD3D drawingHelpers::d_IcannyVisp;
vpDisplayD3D drawingHelpers::d_IcannyImgFilter;
#endif

void drawingHelpers::init(vpImage<unsigned char> &Iinput, vpImage<unsigned char> &IcannyVisp, vpImage<unsigned char> *p_dIx,
                           vpImage<unsigned char> *p_dIy, vpImage<unsigned char> *p_IcannyimgFilter)
{
#if defined(VISP_HAVE_DISPLAY)
  d_Iinput.init(Iinput, 10, 10);
  d_IcannyVisp.init(IcannyVisp, 10, Iinput.getHeight() + 10 * 2);
  if (p_dIx != nullptr) {
    d_dIx.init(*p_dIx, Iinput.getWidth() + 2 * 10, 10);
  }
  if (p_dIy != nullptr) {
    d_dIy.init(*p_dIy, 2 * Iinput.getWidth() + 3 * 10, 10);
  }
  if (p_IcannyimgFilter != nullptr) {
    d_IcannyImgFilter.init(*p_IcannyimgFilter, Iinput.getWidth() + 2 * 10, Iinput.getHeight() + 10 * 2);
  }
#else
  (void)Iinput;
  (void)IcannyVisp;
  (void)p_dIx;
  (void)p_dIy;
  (void)p_IcannyimgFilter;
#endif
}

void drawingHelpers::display(vpImage<unsigned char> &I, const std::string &title)
{
  vpDisplay::display(I);
  vpDisplay::setTitle(I, title);
  vpDisplay::flush(I);
}

bool drawingHelpers::waitForClick(const vpImage<unsigned char> &I, const bool &blockingMode)
{
  vpDisplay::displayText(I, 15, 15, "Left click to continue...", vpColor::red);
  vpDisplay::displayText(I, 35, 15, "Right click to stop...", vpColor::red);
  vpDisplay::flush(I);
  vpMouseButton::vpMouseButtonType button;
  vpDisplay::getClick(I, button, blockingMode);
  bool hasToContinue = true;
  if (button == vpMouseButton::button3) {
    // Right click => stop the program
    hasToContinue = false;
  }

  return hasToContinue;
}

#endif
