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
 * XML parser to load and save oriented rectangle in a XML file
 */

/*!
  \file vpXmlParserRectOriented.cpp
  \brief Definition of the vpXmlParserRectOriented class member functions.
*/

#include <visp3/core/vpXmlParserRectOriented.h>

#include <map>

#if defined(VISP_HAVE_PUGIXML)
#include <pugixml.hpp>

#include <visp3/core/vpIoTools.h>

BEGIN_VISP_NAMESPACE
#ifndef DOXYGEN_SHOULD_SKIP_THIS
class vpXmlParserRectOriented::Impl
{
private:
  enum vpXmlCodeType
  {
    CODE_XML_BAD = -1,
    CODE_XML_OTHER,
    CODE_XML_CENTER_I,
    CODE_XML_CENTER_J,
    CODE_XML_HEIGHT,
    CODE_XML_WIDTH,
    CODE_XML_THETA
  };

public:
  Impl() : m_rectangle(), m_center(), m_height(), m_width(), m_theta(), m_nodeMap()
  {
    m_nodeMap["center_i"] = CODE_XML_CENTER_I;
    m_nodeMap["center_j"] = CODE_XML_CENTER_J;
    m_nodeMap["height"] = CODE_XML_HEIGHT;
    m_nodeMap["width"] = CODE_XML_WIDTH;
    m_nodeMap["theta"] = CODE_XML_THETA;
  }

  void parse(const std::string &filename)
  {
    pugi::xml_document doc;
    if (!doc.load_file(filename.c_str())) {
      throw vpException(vpException::ioError, "cannot open file: %s", filename.c_str());
    }

    pugi::xml_node root_node = doc.document_element();
    if (!root_node) {
      throw vpException(vpException::ioError, "cannot get root element");
    }

    readMainClass(root_node);
  }

  void readMainClass(const pugi::xml_node &node_)
  {
    for (pugi::xml_node dataNode = node_.first_child(); dataNode; dataNode = dataNode.next_sibling()) {
      if (dataNode.type() == pugi::node_element) {
        std::map<std::string, int>::iterator iter_data = m_nodeMap.find(dataNode.name());
        if (iter_data != m_nodeMap.end()) {
          switch (iter_data->second) {
          case CODE_XML_CENTER_I:
            this->m_center.set_i(dataNode.text().as_double());
            break;
          case CODE_XML_CENTER_J:
            this->m_center.set_j(dataNode.text().as_double());
            break;
          case CODE_XML_HEIGHT:
            this->m_height = dataNode.text().as_double();
            break;
          case CODE_XML_WIDTH:
            this->m_width = dataNode.text().as_double();
            break;
          case CODE_XML_THETA:
            this->m_theta = dataNode.text().as_double();
            break;
          default:
            break;
          }
        }
      }
    }

    m_rectangle = vpRectOriented(m_center, m_width, m_height, m_theta);
  }

  void save(const std::string &filename, bool append)
  {
    pugi::xml_document doc;
    pugi::xml_node root_node;

    if (!doc.load_file(filename.c_str(), pugi::parse_default | pugi::parse_comments)) {
      root_node = doc.append_child(pugi::node_declaration);
      root_node.append_attribute("version") = "1.0";
      root_node = doc.append_child("config");
    }
    else if (!append) {
      if (!vpIoTools::remove(filename)) {
        throw vpException(vpException::ioError, "Cannot remove existing xml file");
      }

      root_node = doc.append_child(pugi::node_declaration);
      root_node.append_attribute("version") = "1.0";
      root_node = doc.append_child("config");
    }

    root_node = doc.document_element();
    if (!root_node) {
      throw vpException(vpException::ioError, "problem to get the root node");
    }

    writeMainClass(root_node);

    doc.save_file(filename.c_str(), PUGIXML_TEXT("  "));
  }

  void writeMainClass(pugi::xml_node &node)
  {
    node.append_child("center_i").append_child(pugi::node_pcdata).text() = m_rectangle.getCenter().get_i();
    node.append_child("center_j").append_child(pugi::node_pcdata).text() = m_rectangle.getCenter().get_j();
    node.append_child("height").append_child(pugi::node_pcdata).text() = m_rectangle.getHeight();
    node.append_child("width").append_child(pugi::node_pcdata).text() = m_rectangle.getWidth();
    node.append_child("theta").append_child(pugi::node_pcdata).text() = m_rectangle.getOrientation();
  }

  vpRectOriented getRectangle() const { return m_rectangle; }
  void setRectangle(const vpRectOriented &rectangle) { m_rectangle = rectangle; }

private:
  vpRectOriented m_rectangle;
  vpImagePoint m_center;
  double m_height;
  double m_width;
  double m_theta;
  std::map<std::string, int> m_nodeMap;
};
#endif // DOXYGEN_SHOULD_SKIP_THIS

vpXmlParserRectOriented::vpXmlParserRectOriented() : m_impl(new Impl()) { }

vpXmlParserRectOriented::~vpXmlParserRectOriented() { delete m_impl; }

/*!
  Parse the document.
  The data in the file are stored in the attributes of the child class. This
  method calls the readMainClass method which has to be implemented for every
  child class depending on the content to parse.

  \param filename : name of the file to parse
*/
void vpXmlParserRectOriented::parse(const std::string &filename) { m_impl->parse(filename); }

/*!
  Save the content of the class in the file given in parameters.
  The data of the class are in the child class.
  This method calls the write_main_class method which has to be implemented
  for every class depending on the data to save.

  \param filename : the name of the file used to record the data
  \param append : if true and if the file exists, the data will be added to
  the data already in the file
*/
void vpXmlParserRectOriented::save(const std::string &filename, bool append) { m_impl->save(filename, append); }

vpRectOriented vpXmlParserRectOriented::getRectangle() const { return m_impl->getRectangle(); }

void vpXmlParserRectOriented::setRectangle(const vpRectOriented &rectangle) { m_impl->setRectangle(rectangle); }
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_core.a(vpXmlParserRectOriented.cpp.o) has no symbols
void dummy_vpXmlParserRectOriented() { };

#endif
