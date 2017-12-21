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
 * Load XML parameters of the Model based tracker (using edges).
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 * \file vpMbtXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig
 * object.
 */

#ifndef vpMbtXmlParser_HH
#define vpMbtXmlParser_HH

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h> /* Fonctions de la lib XML.                */

#include <visp3/mbt/vpMbXmlParser.h>
#include <visp3/me/vpMe.h>

/*!
  \class vpMbtXmlParser
  \brief Parse an Xml file to extract configuration parameters of a mbtConfig
  object. \ingroup group_mbt_xml_parser

  Data parser for the model based tracker.

 */
class VISP_EXPORT vpMbtXmlParser : virtual public vpMbXmlParser
{
protected:
  typedef enum {
    ecm = vpMbXmlParser::last,
    mask,
    size,
    nb_mask,
    range,
    tracking,
    contrast,
    edge_threshold,
    mu1,
    mu2,
    sample,
    step,
    last
  } dataToParseMb;

  //! Moving edges parameters.
  vpMe m_ecm;

public:
  /** @name Public Member Functions Inherited from vpMbtXmlParser */
  //@{
  vpMbtXmlParser();
  virtual ~vpMbtXmlParser();

  void getMe(vpMe &_ecm) const { _ecm = this->m_ecm; }

  void parse(const char *filename);

  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);
  void read_ecm(xmlDocPtr doc, xmlNodePtr node);
  void read_sample(xmlDocPtr doc, xmlNodePtr node);
  void read_sample_deprecated(xmlDocPtr doc, xmlNodePtr node);
  void read_mask(xmlDocPtr doc, xmlNodePtr node);
  void read_range(xmlDocPtr doc, xmlNodePtr node);
  void read_contrast(xmlDocPtr doc, xmlNodePtr node);

  void setMovingEdge(const vpMe &_ecm) { m_ecm = _ecm; }

  void writeMainClass(xmlNodePtr node);
  //@}

protected:
  /** @name Protected Member Functions Inherited from vpMbtXmlParser */
  //@{
  void init();
  //@}
};

#endif

#endif /* NMBTXMLPARSER_H_ */
