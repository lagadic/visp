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
 *
 * Description:
 * Draw text in an image.
 */

/*!
  \file vpFont.h

  \brief Draw text in an image.
*/

#ifndef _vpFont_h_
#define _vpFont_h_

#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpFont

  \ingroup group_core_image

  \brief Font drawing functions for image.
*/
class VISP_EXPORT vpFont
{
public:
  enum vpFontFamily { GENERIC_MONOSPACE, TRUETYPE_FILE };

  vpFont(unsigned int height = 16, const vpFontFamily &fontFamily = TRUETYPE_FILE,
         const std::string &ttfFilename = std::string(VISP_RUBIK_REGULAR_FONT_RESOURCES));
  ~vpFont();

  bool drawText(vpImage<unsigned char> &I, const std::string &text, const vpImagePoint &position,
                unsigned char color) const;
  bool drawText(vpImage<unsigned char> &I, const std::string &text, const vpImagePoint &position, unsigned char color,
                unsigned char background) const;

  bool drawText(vpImage<vpRGBa> &I, const std::string &text, const vpImagePoint &position, const vpColor &color) const;
  bool drawText(vpImage<vpRGBa> &I, const std::string &text, const vpImagePoint &position, const vpColor &color,
                const vpColor &background) const;

  unsigned int getHeight() const;
  vpImagePoint getMeasure(const std::string &text) const;
  bool setHeight(unsigned int height);

private:
  vpFont(const vpFont &);            // noncopyable
  vpFont &operator=(const vpFont &); //

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};
END_VISP_NAMESPACE
#endif
