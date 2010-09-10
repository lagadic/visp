/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.GPL at the root directory of this source
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
 * Make the complete tracking of an object by using its CAD model
 *
 * Authors:
 * Nicolas Melchior
 * Romain Tallonneau
 * Eric Marchand
 *
 *****************************************************************************/

/*!
 * \file vpMbtXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig object.
*/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#ifndef vpMbtXmlParser_HH
#define vpMbtXmlParser_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2
#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */
#endif
#include <visp/vpMe.h>
#include <visp/vpCameraParameters.h>



/*!
  \class vpMbtXmlParser

  \ingroup ModelBasedTracking

 */
class VISP_EXPORT vpMbtXmlParser
{
private:
    vpMe ecm;
    vpCameraParameters cam;


public:

/* --- CODE XML ------------------------------------------------------------ */
enum CodeXml
{
 CODE_XML_AUTRE,
 CODE_XML_ECM,
 CODE_XML_MASK,
 CODE_XML_SIZE,
 CODE_XML_NB_MASK,
 CODE_XML_RANGE,
 CODE_XML_INIT,
 CODE_XML_TRACKING	,
 CODE_XML_CONTRAST	,
 CODE_XML_EDGE_THRESHOLD,
 CODE_XML_MU1				,
 CODE_XML_MU2			,
 CODE_XML_SAMPLE	,
 CODE_XML_STEP		,
 CODE_XML_NB_SAMPLE,
 CODE_XML_CAMERA		,
 CODE_XML_HEIGHT		,
 CODE_XML_WIDTH			,
 CODE_XML_U0		,
 CODE_XML_V0		,
 CODE_XML_PX		,
 CODE_XML_PY		
};


enum CodeSequence
{
	SEQUENCE_OK    ,
	SEQUENCE_ERROR
};

public:



#ifdef VISP_HAVE_XML2
void
myXmlReadIntChild (xmlDocPtr doc,
		   xmlNodePtr node,
		   int &res,
		   int &code_erreur);

void
myXmlReadDoubleChild (xmlDocPtr doc,
		     xmlNodePtr node,
		     double &res,
		     int &code_erreur);

void
myXmlReadCharChild (xmlDocPtr doc,
		   xmlNodePtr node,
		   char **res);
#endif

private:


int
code_str_to_int (char * str, int & res);

public:

	vpMbtXmlParser(){}

	~vpMbtXmlParser();

#ifdef VISP_HAVE_XML2
	int Parse(const char * filename);
	int lecture (xmlDocPtr doc, xmlNodePtr node);

	int lecture_ecm (xmlDocPtr doc, xmlNodePtr node);
	int lecture_sample (xmlDocPtr doc, xmlNodePtr node);
	int lecture_camera (xmlDocPtr doc, xmlNodePtr node);
	int lecture_mask (xmlDocPtr doc, xmlNodePtr node);
	int lecture_range (xmlDocPtr doc, xmlNodePtr node);
	int lecture_contrast (xmlDocPtr doc, xmlNodePtr node);
#endif
	
	void getCameraParameters(vpCameraParameters& _cam) const { _cam = this->cam;}
	void getMe(vpMe& _ecm) const { _ecm = this->ecm;}

};

#endif

#endif /* NMBTXMLPARSER_H_ */



