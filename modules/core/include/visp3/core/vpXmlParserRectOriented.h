/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * XML parser to load and save oriented rectangles in a XML file
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/*!
  \file vpXmlParserRectOriented.h
  \brief Declaration of the vpXmlParserRectOriented class.
  Class vpXmlParserRectOriented allows to load and save oriented rectangles in a file XML
*/

#ifndef vpXmlParserRectOriented_h
#define vpXmlParserRectOriented_h

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2
#include <string>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpRectOriented.h>
#include <visp3/core/vpXmlParser.h>

/*!
  \class vpXmlParserRectOriented

  \ingroup group_core_geometry

  \brief XML parser to load and save an oriented rectangle in a file.

  The following example shows how to save an oriented rectangle in an xml file:
  \code
#include <visp3/core/vpRectOriented.h>
#include <visp3/core/vpXmlParserRectOriented.h>
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
int main()
{
  vpXmlParserRectOriented parser;
  std::string filename = "myRectangle.xml";
  parser.parse(filename);
  vpRectOriented rect = parser.getRectangle();
  return 0;
}
  \endcode

  \warning This class is only available if libxml2 is installed and detected by ViSP. Installation instructions are
  provided here https://visp.inria.fr/3rd_xml2.
*/

class VISP_EXPORT vpXmlParserRectOriented : public vpXmlParser
{

public:
  vpXmlParserRectOriented();
  virtual ~vpXmlParserRectOriented();

  typedef enum {
    CODE_XML_BAD = -1,
    CODE_XML_OTHER,
    CODE_XML_CENTER_I,
    CODE_XML_CENTER_J,
    CODE_XML_HEIGHT,
    CODE_XML_WIDTH,
    CODE_XML_THETA
  } vpXmlCodeType;

  typedef enum { SEQUENCE_OK, SEQUENCE_ERROR } vpXmlCodeSequenceType;

  vpRectOriented const getRectangle() { return m_rectangle; }
  void setRectangle(const vpRectOriented rectangle) { m_rectangle = rectangle; }

private:
  vpRectOriented m_rectangle;
  vpImagePoint m_center;
  double m_height;
  double m_width;
  double m_theta;

protected:
  void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void writeMainClass(xmlNodePtr node);
};
#endif // VISP_HAVE_XML2
#endif // vpXmlParserRectOriented_h
