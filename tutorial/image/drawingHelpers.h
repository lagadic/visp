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
#ifndef _drawingHelpers_h_
#define _drawingHelpers_h_

#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

namespace drawingHelpers
{
  #if defined(VISP_HAVE_X11)
    extern vpDisplayX d;
  #elif defined(HAVE_OPENCV_HIGHGUI)
    extern vpDisplayOpenCV d;
  #elif defined(VISP_HAVE_GTK)
    extern vpDisplayGTK d;
  #elif defined(VISP_HAVE_GDI)
    extern vpDisplayGDI d;
  #elif defined(VISP_HAVE_D3D9)
    extern vpDisplayD3D d;
  #endif

  extern vpImage<vpRGBa> I_disp; /*!< Displayed image.*/

  /**
   * \brief Display a RGB image and catch the user clicks to know if
   * the user wants to stop the program.
   * 
   * \param[out] I The RGB image to display.
   * \param[in] title The title of the window.
   * \param[in] blockingMode If true, wait for a click to switch to the next image.
   * \return true The user wants to continue the application.
   * \return false The user wants to stop the application.
   */
  bool display(vpImage<vpRGBa> &I, const std::string &title, const bool &blockingMode);

  /**
   * \brief Display a gray-scale image and catch the user clicks to know if
   * the user wants to stop the program.
   * 
   * \param[out] I The gray-scale image to display.
   * \param[in] title The title of the window.
   * \param[in] blockingMode If true, wait for a click to switch to the next image.
   * \return true The user wants to continue the application.
   * \return false The user wants to stop the application.
   */
  bool display(vpImage<unsigned char> &I, const std::string &title, const bool &blockingMode);

  /**
   * \brief Display a double precision image and catch the user clicks to know if
   * the user wants to stop the program.
   * 
   * \param[out] I The double precision image to display.
   * \param[in] title The title of the window.
   * \param[in] blockingMode If true, wait for a click to switch to the next image.
   * \return true The user wants to continue the application.
   * \return false The user wants to stop the application.
   */
  bool display(vpImage<double> &D, const std::string &title, const bool &blockingMode);

  /**
   * \brief Display a floating-point precision image and catch the user clicks to know if
   * the user wants to stop the program.
   * 
   * \param[out] I The floating-point precision image to display.
   * \param[in] title The title of the window.
   * \param[in] blockingMode If true, wait for a click to switch to the next image.
   * \return true The user wants to continue the application.
   * \return false The user wants to stop the application.
   */
  bool display(vpImage<float> &F, const std::string &title, const bool &blockingMode);
}

#endif
