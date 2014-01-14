/****************************************************************************
 *
 * $Id: vpMbtEdgeKltXmlParser.h 4574 2014-01-09 08:48:51Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Load XML parameters of the Model based tracker (using edges and point features).
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
 * \file vpMbtEdgeKltXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig object.
*/

#ifndef vpMbtEdgeKltXmlParser_HH
#define vpMbtEdgeKltXmlParser_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */

#include <visp/vpMbtXmlParser.h>
#include <visp/vpMbtKltXmlParser.h>

/*!
  \class vpMbtEdgeKltXmlParser
  \brief Parse an Xml file to extract configuration parameters of a mbtConfig object.
  \ingroup ModelBasedTracking

  Data parser for the model based tracker.

 */
class VISP_EXPORT vpMbtEdgeKltXmlParser: public vpMbtXmlParser, public vpMbtKltXmlParser
{
protected:
  typedef enum{
    camera,
    face,
    klt,
    ecm,
    sample
  } dataToParseMbtEdgeKlt;
public:

    vpMbtEdgeKltXmlParser();
    virtual ~vpMbtEdgeKltXmlParser();

  void parse(const char * filename);

  virtual void readMainClass(xmlDocPtr doc, xmlNodePtr node);

  void writeMainClass(xmlNodePtr node);
	
protected:
  void init();

};

#endif

#endif /* NMBTEDGEKLTXMLPARSER_H_ */



