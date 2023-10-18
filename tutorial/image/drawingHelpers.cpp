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

#if defined(VISP_HAVE_X11)
  vpDisplayX drawingHelpers::d;
#elif defined(HAVE_OPENCV_HIGHGUI)
  vpDisplayOpenCV drawingHelpers::d;
#elif defined(VISP_HAVE_GTK)
  vpDisplayGTK drawingHelpers::d;
#elif defined(VISP_HAVE_GDI)
  vpDisplayGDI drawingHelpers::d;
#elif defined(VISP_HAVE_D3D9)
  vpDisplayD3D drawingHelpers::d;
#endif

vpImage<vpRGBa> drawingHelpers::I_disp;

bool drawingHelpers::display(vpImage<vpRGBa> &I, const std::string &title, const bool &blockingMode)
{ 
  I_disp = I;
  d.init(I_disp);
  vpDisplay::setTitle(I_disp, title.c_str());
  
  vpDisplay::display(I_disp);
  vpDisplay::displayText(I_disp, 15, 15, "Left click to continue...", vpColor::red);
  vpDisplay::displayText(I_disp, 35, 15, "Right click to stop...", vpColor::red);
  vpDisplay::flush(I_disp);
  vpMouseButton::vpMouseButtonType button;
  vpDisplay::getClick(I_disp, button, blockingMode);
  bool hasToContinue = true;
  if (button == vpMouseButton::button3)
  {
    // Right click => stop the program
    hasToContinue = false;
  }

  return hasToContinue;
}
 
bool drawingHelpers::display(vpImage<unsigned char> &D, const std::string &title, const bool &blockingMode)
{
  vpImage<vpRGBa> I; // Image to display
  vpImageConvert::convert(D, I);
  return display(I, title, blockingMode);
} 

bool drawingHelpers::display(vpImage<double> &D, const std::string &title, const bool &blockingMode)
{
  vpImage<unsigned char> I; // Image to display
  vpImageConvert::convert(D, I);
  return display(I, title, blockingMode);
}

bool drawingHelpers::display(vpImage<float> &F, const std::string &title, const bool &blockingMode)
{
  vpImage<unsigned char> I; // Image to display
  vpImageConvert::convert(F, I);
  return display(I, title, blockingMode);
}
