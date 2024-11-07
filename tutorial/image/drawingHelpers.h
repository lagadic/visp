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
#ifndef DRAWING_HELPERS_H
#define DRAWING_HELPERS_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

namespace drawingHelpers
{
#if defined(VISP_HAVE_X11)
extern VISP_NAMESPACE_ADDRESSING vpDisplayX d_Iinput;
extern VISP_NAMESPACE_ADDRESSING vpDisplayX d_dIx;
extern VISP_NAMESPACE_ADDRESSING vpDisplayX d_dIy;
extern VISP_NAMESPACE_ADDRESSING vpDisplayX d_IcannyVisp;
extern VISP_NAMESPACE_ADDRESSING vpDisplayX d_IcannyImgFilter;
#elif defined(HAVE_OPENCV_HIGHGUI)
extern VISP_NAMESPACE_ADDRESSING vpDisplayOpenCV d_Iinput;
extern VISP_NAMESPACE_ADDRESSING vpDisplayOpenCV d_dIx;
extern VISP_NAMESPACE_ADDRESSING vpDisplayOpenCV d_dIy;
extern VISP_NAMESPACE_ADDRESSING vpDisplayOpenCV d_IcannyVisp;
extern VISP_NAMESPACE_ADDRESSING vpDisplayOpenCV d_IcannyImgFilter;
#elif defined(VISP_HAVE_GTK)
extern VISP_NAMESPACE_ADDRESSING vpDisplayGTK d_Iinput;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGTK d_dIx;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGTK d_dIy;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGTK d_IcannyVisp;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGTK d_IcannyImgFilter;
#elif defined(VISP_HAVE_GDI)
extern VISP_NAMESPACE_ADDRESSING vpDisplayGDI d_Iinput;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGDI d_dIx;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGDI d_dIy;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGDI d_IcannyVisp;
extern VISP_NAMESPACE_ADDRESSING vpDisplayGDI d_IcannyImgFilter;
#elif defined(VISP_HAVE_D3D9)
extern VISP_NAMESPACE_ADDRESSING vpDisplayD3D d_Iinput;
extern VISP_NAMESPACE_ADDRESSING vpDisplayD3D d_dIx;
extern VISP_NAMESPACE_ADDRESSING vpDisplayD3D d_dIy;
extern VISP_NAMESPACE_ADDRESSING vpDisplayD3D d_IcannyVisp;
extern VISP_NAMESPACE_ADDRESSING vpDisplayD3D d_IcannyImgFilter;
#endif

/**
 * \brief Initialize the different displays.
 *
 * \param[out] Iinput Input image of the program.
 * \param[out] IcannyVisp Image resulting from the vpCannyEdgeDetection method.
 * \param[out] p_dIx If different from nullptr, pointer towards the gradient along the horizontal axis.
 * \param[out] p_dIy If different from nullptr, pointer towards the gradient along the vertical axis.
 * \param[out] p_IcannyimgFilter If different from nullptr, pointer towards the result of the vpImageFilter::canny
 * method.
 */
void init(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &Iinput, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &IcannyVisp, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> *p_dIx,
          VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> *p_dIy, VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> *p_IcannyimgFilter);

/**
 * \brief Display a gray-scale image.
 *
 * \param[out] I The gray-scale image to display.
 * \param[in] title The title of the window.
 */
void display(VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const std::string &title);

/**
 * \brief Catch the user clicks to know if the user wants to stop the program.
 *
 * \param[in] I The gray-scale image to display.
 * \param[in] blockingMode If true, wait for a click to switch to the next image.
 * \return true The user wants to continue the application.
 * \return false The user wants to stop the application.
 */
bool waitForClick(const VISP_NAMESPACE_ADDRESSING vpImage<unsigned char> &I, const bool &blockingMode);
}

#endif
#endif
