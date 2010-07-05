/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
	
	void getCameraParameters(vpCameraParameters& _cam){ _cam = this->cam;}
	void getMe(vpMe& _ecm){ _ecm = this->ecm;}

};

#endif

#endif /* NMBTXMLPARSER_H_ */



