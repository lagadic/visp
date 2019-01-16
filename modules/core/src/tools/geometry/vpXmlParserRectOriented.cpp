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
 * XML parser to load and save oriented rectangle in a XML file
 *
 * Authors:
 * Marc Pouliquen
 *
 *****************************************************************************/

/*!
  \file vpXmlParserRectOriented.cpp
  \brief Definition of the vpXmlParserRectOriented class member functions.

*/
#include <visp3/core/vpXmlParserRectOriented.h>
#ifdef VISP_HAVE_XML2

/**
 * Default constructor.
 */
vpXmlParserRectOriented::vpXmlParserRectOriented() : m_rectangle(), m_center(), m_height(), m_width(), m_theta()
{
  nodeMap["center_i"] = CODE_XML_CENTER_I;
  nodeMap["center_j"] = CODE_XML_CENTER_J;
  nodeMap["height"] = CODE_XML_HEIGHT;
  nodeMap["width"] = CODE_XML_WIDTH;
  nodeMap["theta"] = CODE_XML_THETA;
}

/**
* Destructor.
*/
vpXmlParserRectOriented::~vpXmlParserRectOriented() {}

/**
* Reading method, called by vpXmlParser::parse().
* @param doc : a pointer representing the document.
* @param node : the root node of the document.
*/
void vpXmlParserRectOriented::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case CODE_XML_CENTER_I:
          this->m_center.set_i(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_CENTER_J:
          this->m_center.set_j(xmlReadDoubleChild(doc, dataNode));
          break;
        case CODE_XML_HEIGHT:
          this->m_height = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_WIDTH:
          this->m_width = xmlReadDoubleChild(doc, dataNode);
          break;
        case CODE_XML_THETA:
          this->m_theta = xmlReadDoubleChild(doc, dataNode);
          break;
        default:
          vpTRACE("unknown tag in readConfigNode : %d, %s", iter_data->second, (iter_data->first).c_str());
          break;
        }
      }
    }
  }
  m_rectangle = vpRectOriented(m_center, m_width, m_height, m_theta);
}

/**
* Writing method, called by vpXmlParser::save().
* @param node : the root node of the document.
*/
void vpXmlParserRectOriented::writeMainClass(xmlNodePtr node)
{
  xmlWriteDoubleChild(node, "center_i", this->m_rectangle.getCenter().get_i());
  xmlWriteDoubleChild(node, "center_j", this->m_rectangle.getCenter().get_j());
  xmlWriteDoubleChild(node, "height", this->m_rectangle.getHeight());
  xmlWriteDoubleChild(node, "width", this->m_rectangle.getWidth());
  xmlWriteDoubleChild(node, "theta", this->m_rectangle.getOrientation());
}

#endif // VISP_HAVE_XML2
