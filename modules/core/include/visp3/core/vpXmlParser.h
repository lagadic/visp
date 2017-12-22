/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Tools to automatise the creation of xml parser based on the libXML2
 *
 * Authors:
 * Romain Tallonneau
 *
 *****************************************************************************/

#ifndef vpXmlParser_HH
#define vpXmlParser_HH

/*!
  \file vpXmlParser.h
  \brief Tools to simplify the creation of xml parser based on the libXML2
*/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <visp3/core/vpException.h>

#include <libxml/parser.h>

#include <iomanip>
#include <map>
#include <sstream>
#include <string.h>
#include <string>
#include <typeinfo>

/*!
  \class vpXmlParser

  \brief This class intends to simplify the creation of xml parser based on
  the libxml2 third party library.

  This class can be useful to manage external data parameters (for example for
  configuration of an experiment, ...).

  \warning This class is only available if libxml2 is installed and detected
  by ViSP. Installation instructions are provided here
  https://visp.inria.fr/3rd_xml2.

  In order to use this class, you have to create a new class inheriting from
  this one. In the child class, you have to implement the methods:
  - writeMainClass()
  - readMainClass()

  These two methods depends on the data to parse, and must not be directly
  called (they are called from the parse() and the save() methods).

  Following is an example of implementation for the document:

  \code
  <config>
      <range>5</range>
      <step>7</step>
      <size_filter>3</size_filter>
  </config>
  \endcode

  A class to parse this document is declared as follows:

  \code
  class vpDataParser: public vpXmlParser
  {
  private:
    int m_range;
    int m_step;
    int m_size_filter
  public:
    typedef enum{
      config,
      range,
      step,
      size_filter
    }dataToParse

    vpDataParser(){
      nodeMap["config"] = config;
      nodeMap["range"] = range;
      nodeMap["step"] = step;
      nodeMap["size_filter"] = size_filter;
    }

    virtual void writeMainClass(xmlNodePtr node);
    virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);

    // additionals methods specific to the data to parse
    // such as: accessors
  }
  \endcode

  The readMainClass function implementation is:

  \code
  void
  vpDataParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
  {
    for (xmlNodePtr tmpNode = node->xmlChildrenNode; tmpNode != NULL; tmpNode = tmpNode->next) {
      if(tmpNode->type == XML_ELEMENT_NODE) {

        std::map<std::string, int>::iterator iter = this->nodeMap.find((char*)tmpNode->name);
        if(iter == nodeMap.end()) {
          continue;
        }

        switch (iter->second){
        case range:
          this->m_range = xmlReadIntChild(doc, tmpNode);
          break;
        case step:
          this->m_step = xmlReadIntChild(doc, tmpNode);
          break;
        case size_filter:
          this->m_size_filter = xmlReadIntChild(doc, tmpNode);
          break;
        default:
          std::cout << "problem in the readMainClass (" << iter->second
                    << " , " << iter->first << " )" << std::endl; break;
        }
      }
    }
  }
  \endcode

  Data can now be accessed through the internal variables of the class
  vpDataParser.

  To store the data in a xml file, the function save has to be called. This
  function needs the implementation of the writeMainClass function.

  For example,

  \code
  void
  vpDataParser::writeMainClass(xmlNodePtr node)
  {
    xmlWriteIntChild(node, "range", m_range);
    xmlWriteIntChild(node, "step", m_step);
    xmlWriteIntChild(node, "size_filter", m_size_filter);
  }
  \endcode

*/
class VISP_EXPORT vpXmlParser
{
protected:
  /** @name Protected Member Functions Inherited from vpXmlParser */
  //@{
  /*!
    pure virtual method used to read the document.

    As the content of the function depends on the structure of the file to
    read, data name, data types and data values, it has to be reimplemented
    for every type of filenam

    \param doc : a pointer representing the document
    \param node : the root node of the document
  */
  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node) = 0;

  /*!
    pure virtual method used to write the document.

    As the content of the function depends on the structure of the file to
    read, data name and data types, it has to be reimplemented for every type
    of file to parse.

    \param node : the root node of the document
  */
  virtual void writeMainClass(xmlNodePtr node) = 0;

  bool xmlReadBoolChild(xmlDocPtr doc, xmlNodePtr node);
  char *xmlReadCharChild(xmlDocPtr doc, xmlNodePtr node);
  double xmlReadDoubleChild(xmlDocPtr doc, xmlNodePtr node);
  float xmlReadFloatChild(xmlDocPtr doc, xmlNodePtr node);
  int xmlReadIntChild(xmlDocPtr doc, xmlNodePtr node);
  std::string xmlReadStringChild(xmlDocPtr doc, xmlNodePtr node);
  unsigned int xmlReadUnsignedIntChild(xmlDocPtr doc, xmlNodePtr node);

  void xmlWriteBoolChild(xmlNodePtr node, const char *label, const bool value);
  void xmlWriteCharChild(xmlNodePtr node, const char *label, const char *value);
  void xmlWriteDoubleChild(xmlNodePtr node, const char *label, const double value);
  void xmlWriteFloatChild(xmlNodePtr node, const char *label, const float value);
  void xmlWriteIntChild(xmlNodePtr node, const char *label, const int value);
  void xmlWriteStringChild(xmlNodePtr node, const char *label, const std::string &value);
  void xmlWriteUnsignedIntChild(xmlNodePtr node, const char *label, const unsigned int value);
  //@}

protected:
  /*!
    The map describing the data to parse
  */
  std::map<std::string, int> nodeMap;

  /*!
    The name of the main tag for the file to parse
  */
  std::string main_tag;

public:
  /** @name Public Member Functions Inherited from vpXmlParser */
  //@{
  vpXmlParser();
  vpXmlParser(const vpXmlParser &_twin);
  virtual ~vpXmlParser();

  /* virtual */ void parse(const std::string &filename);
  /* virtual */ void save(const std::string &filename, const bool append = false);

  /*!
    Set the map describing the data to parse. This map stores the name of each
    node and an associated key used to simplify the parsing of the file.

    If the following file want to be parsed:

    \code
    <config>
      <range>5</range>
      <step>7</step>
      <size_filter>3</size_filter>
    </config>
    \endcode

    The following map has to be declared:

    \code
    std::map dataToParse;
    dataToParse["config"] = 0;
    dataToParse["range"] = 1;
    dataToParse["step"] = 2;
    dataToParse["size_filter"] = 3;
    \endcode

    Or, you can use keyzord instead of number as key but it implies to declare
    in the child class an enumeration type of the name. For example:

    \code
    typedef enum{
      config,
      range,
      step,
      size_filter} data_enum;

    std::map dataToParse;
    dataToParse["config"] = config;
    dataToParse["range"] = range;
    dataToParse["step"] = step;
    dataToParse["size_filter"] = size_filter;
    \endcode

    \param _map : the map describing the data to parse
  */
  void setMap(const std::map<std::string, int> &_map) { nodeMap = _map; }

  /*!
    set the name of the main tag

    The main tag corresponds to the name of the root node

    \param tag : name of the root node of the document
  */
  inline void setMainTag(const std::string &tag) { main_tag = tag; }
  //@}

  /** @name Static Public Member Functions Inherited from vpXmlParser */
  //@{
  /*!
  As stated in http://xmlsoft.org/html/libxml-parser.html#xmlCleanupParser
  to clean up memory allocated by the xml2 library itself, the user should
  call xmlCleanupParser() only when the process has finished using the xml2
  library. In case of doubt abstain from calling this function or do it just
  before calling exit() to avoid leak reports from valgrind ! That's why in
  ViSP the destructor doesn't call xmlCleanupParser(). Rather we provide the
  static function vpXmlParser::cleanup() that calls xmlCleanupParser() that
  could be called just before exit().
    */
  static void cleanup() { xmlCleanupParser(); }
  //@}
};

#endif /* VISP_HAVE_XML2 */

#endif
