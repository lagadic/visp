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
 * Load XML parameters of the Model based tracker (using edges and point
 *features).
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 * \file vpMbtEdgeKltXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig
 * object.
 */

#ifndef vpMbtEdgeKltXmlParser_HH
#define vpMbtEdgeKltXmlParser_HH

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h> /* Fonctions de la lib XML.                */

#include <visp3/mbt/vpMbtKltXmlParser.h>
#include <visp3/mbt/vpMbtXmlParser.h>

/*!
  \class vpMbtEdgeKltXmlParser
  \brief Parse an Xml file to extract configuration parameters of a mbtConfig
  object. \ingroup group_mbt_xml_parser

  Data parser for the model based tracker.

 */
class VISP_EXPORT vpMbtEdgeKltXmlParser : public vpMbtXmlParser, public vpMbtKltXmlParser
{
protected:
  typedef enum { camera, face, klt, ecm, lod } dataToParseMbtEdgeKlt;

public:
  vpMbtEdgeKltXmlParser();
  virtual ~vpMbtEdgeKltXmlParser();

  void parse(const char *filename);

  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);

  void writeMainClass(xmlNodePtr node);

protected:
  void init();
};

#endif

#endif /* NMBTEDGEKLTXMLPARSER_H_ */
