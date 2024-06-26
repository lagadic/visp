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
 *
 * Description:
 * XML parser to load and save oriented rectangles in a XML file.
 */

/*!
  \file vpXmlParserRectOriented.h
  \brief Declaration of the vpXmlParserRectOriented class.
  Class vpXmlParserRectOriented allows to load and save oriented rectangles in a file XML
*/

#ifndef VP_XML_PARSER_RECT_ORIENTED_H
#define VP_XML_PARSER_RECT_ORIENTED_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PUGIXML)
#include <visp3/core/vpRectOriented.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpXmlParserRectOriented

  \ingroup group_core_geometry

  \brief XML parser to load and save an oriented rectangle in a file.

  The following example shows how to save an oriented rectangle in an xml file:
  \code
  #include <visp3/core/vpRectOriented.h>
  #include <visp3/core/vpXmlParserRectOriented.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRectOriented rect(vpImagePoint(10, 15), 20, 12, 0.25);
    vpXmlParserRectOriented parser;
    parser.setRectangle(rect);
    std::string filename = "myRectangle.xml";
    parser.save(filename);
    return 0;
  }
  \endcode

  The following example shows how to read an oriented rectangle from an xml file:
  \code
  #include <visp3/core/vpRectOriented.h>
  #include <visp3/core/vpXmlParserRectOriented.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpXmlParserRectOriented parser;
    std::string filename = "myRectangle.xml";
    parser.parse(filename);
    vpRectOriented rect = parser.getRectangle();
    return 0;
  }
  \endcode

  \warning This class is only available if pugixml third-party is successfully
built.
*/

class VISP_EXPORT vpXmlParserRectOriented
{
public:
  vpXmlParserRectOriented();
  ~vpXmlParserRectOriented();

  enum vpXmlCodeSequenceType { SEQUENCE_OK, SEQUENCE_ERROR };

  vpRectOriented getRectangle() const;

  void parse(const std::string &filename);
  void save(const std::string &filename, bool append = false);

  void setRectangle(const vpRectOriented &rectangle);

private:
  vpXmlParserRectOriented(const vpXmlParserRectOriented &ro);            // noncopyable
  vpXmlParserRectOriented &operator=(const vpXmlParserRectOriented &); //

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif // vpXmlParserRectOriented_h
