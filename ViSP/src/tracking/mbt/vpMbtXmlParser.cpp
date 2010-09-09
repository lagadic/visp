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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <visp/vpMbtXmlParser.h>

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */
#include <iostream>

/* -------------------------------------------------------------------------- */
/* --- LABEL XML ------------------------------------------------------------ */
/* -------------------------------------------------------------------------- */

// Remarque : CODE_XML_XXX sont definis dans la classe

#define LABEL_XML_ECM												"ecm"
#define LABEL_XML_MASK											"mask"
#define LABEL_XML_SIZE											"size"
#define LABEL_XML_NB_MASK										"nb_mask"
#define LABEL_XML_RANGE											"range"
#define LABEL_XML_INIT											"init"
#define LABEL_XML_TRACKING									"tracking"
#define LABEL_XML_CONTRAST									"contrast"
#define LABEL_XML_EDGE_THRESHOLD						"edge_threshold"
#define LABEL_XML_MU1												"mu1"
#define LABEL_XML_MU2												"mu2"
#define LABEL_XML_SAMPLE										"sample"
#define LABEL_XML_STEP											"step"
#define LABEL_XML_NB_SAMPLE									"nb_sample"
#define LABEL_XML_CAMERA										"camera"
#define LABEL_XML_HEIGHT										"height"
#define LABEL_XML_WIDTH											"width"
#define LABEL_XML_U0												"u0"
#define LABEL_XML_V0								 				"v0"
#define LABEL_XML_PX												"px"
#define LABEL_XML_PY												"py"

vpMbtXmlParser::
~vpMbtXmlParser()
{
}

int vpMbtXmlParser::
Parse(const char * filename)
{
	xmlDocPtr doc;
	xmlNodePtr node;


	doc = xmlParseFile(filename);
	if (doc == NULL)
	{
		return SEQUENCE_ERROR;
	}

	node = xmlDocGetRootElement(doc);
	if (node == NULL)
	{
		xmlFreeDoc(doc);
		return SEQUENCE_ERROR;
	}

	this ->lecture (doc, node);

	xmlFreeDoc(doc);

	return SEQUENCE_OK;
}


void vpMbtXmlParser::
myXmlReadCharChild (xmlDocPtr doc,
		xmlNodePtr node,
		char **res)
{
	xmlNodePtr cur;

	cur = node ->xmlChildrenNode;

	*res = (char *)	xmlNodeListGetString(doc, cur, 1);

	return ;
} /* myXmlReadCharChild () */


/* Lit un champs du fichier XML en integer
 * INPUT:
 *   - doc, node: l'arbre XML.
 *   - res: variable ou placer le resultat.
 *   - code_erreur: place a SEQUENCE_ERROR si erreur.
 */
void vpMbtXmlParser::
myXmlReadIntChild (xmlDocPtr doc,
		xmlNodePtr node,
		int &res,
		int &code_erreur)
{
	char * val_char;
	char * control_convert;
	int val_int;
	xmlNodePtr cur;

	cur = node ->xmlChildrenNode;

	val_char = (char *)	xmlNodeListGetString(doc, cur, 1);
	val_int = strtol ((char *)val_char, &control_convert, 10);

	if (val_char == control_convert)
	{
		val_int = 0;
		code_erreur = SEQUENCE_ERROR;
	}

	xmlFree(val_char);
	res = val_int;
	return ;
} /* myXmlReadIntChild () */

/* Lit un champs du fichier XML en integer
 * INPUT:
 *   - doc, node: l'arbre XML.
 *   - res: variable ou placer le resultat.
 *   - code_erreur: place a SEQUENCE_ERROR si erreur.
 */
void vpMbtXmlParser::
myXmlReadDoubleChild (xmlDocPtr doc,
		xmlNodePtr node,
		double &res,
		int &code_erreur)
{
	char * val_char;
	char * control_convert;
	double val_double;
	xmlNodePtr cur;

	cur = node ->xmlChildrenNode;

	val_char = (char *)	xmlNodeListGetString(doc, cur, 1);
	val_double = strtod ((char *)val_char, &control_convert);

	if (val_char == control_convert)
	{
		val_double = 0;
		code_erreur = SEQUENCE_ERROR;
	}

	xmlFree(val_char);
	res = val_double;
	return ;
} /* myXmlReaddoubleChild () */


/* Lecture des parametres a partir d'un fichier XML.
 * INPUT:
 *   - doc: document XML.
 *   - node: arbre XML, pointant sur un marqueur equipement.
 * OUTPUT:
 *   - code d'erreur.
 */
int vpMbtXmlParser::
lecture (xmlDocPtr doc, xmlNodePtr node)
{
	int prop;
	int retour = SEQUENCE_OK;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE) continue;
		if (SEQUENCE_OK != code_str_to_int ((char*)(node ->name), prop))
		{
			prop = CODE_XML_AUTRE;
			retour = SEQUENCE_ERROR;
		}

		switch (prop)
		{
		case CODE_XML_ECM:

			this ->lecture_ecm (doc, node);
			break;

		case CODE_XML_SAMPLE:

			this ->lecture_sample (doc, node);
			break;

		case CODE_XML_CAMERA:

			this ->lecture_camera (doc, node);
			break;

		default:
			retour = SEQUENCE_ERROR;
			break;
		}
	}

	return retour;
}

int vpMbtXmlParser::
lecture_ecm (xmlDocPtr doc, xmlNodePtr node)
{
	int prop;
	int retour = SEQUENCE_OK;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE) continue;
		if (SEQUENCE_OK != code_str_to_int ((char*)(node ->name), prop))
		{
			prop = CODE_XML_AUTRE;
			retour = SEQUENCE_ERROR;
		}

		switch (prop)
		{
		case CODE_XML_MASK:

			this ->lecture_mask (doc, node);
			break;
		case CODE_XML_RANGE:

			this ->lecture_range (doc, node);
			break;
		case CODE_XML_CONTRAST:

			this ->lecture_contrast (doc, node);
			break;

		default:
			retour = SEQUENCE_ERROR;
			break;
		}
	}

	return retour;
}

int vpMbtXmlParser::
lecture_sample (xmlDocPtr doc, xmlNodePtr node)
{
	int prop;
	/* Compteur du nombre de parametres lus. */
	int nb = 0;
	/* Valeur lue dans le XML. */
	int val;

	int stp = this->ecm.sample_step;
	int nb_sample = this->ecm.ntotal_sample;

	int retour = SEQUENCE_OK;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE) continue;
		if (SEQUENCE_OK != code_str_to_int ((char*)(node ->name), prop))
		{
			prop = CODE_XML_AUTRE;
			retour = SEQUENCE_ERROR;
		}

		switch (prop)
		{
		case CODE_XML_STEP:

			myXmlReadIntChild (doc, node, val, retour);
			stp=val;nb++;
			break;

		case CODE_XML_NB_SAMPLE:

			myXmlReadIntChild (doc, node, val, retour);
			nb_sample = val;nb++;
			break;

		default:
			retour = SEQUENCE_ERROR;
			break;
		}

	}

  
	this->ecm.sample_step = stp;
	this->ecm.ntotal_sample = nb_sample;

	std::cout <<"**** sample:\n";
	std::cout <<"sample_step "<< this->ecm.sample_step<<std::endl;
	std::cout <<"n_total_sample "<< this->ecm.ntotal_sample<<std::endl;

	if (nb != 2)
	{
		std::cout <<"ERROR in 'sample' field:\n";
		std::cout << "it must contain 2 parameters\n";

		return SEQUENCE_ERROR;
	}

	return retour;
}

int vpMbtXmlParser::
lecture_camera (xmlDocPtr doc, xmlNodePtr node)
{
	int prop;
	/* Compteur du nombre de parametres lus. */
	int nb = 0;
	/* Valeur lue dans le XML. */
	int val;
	double vald;

	int height=0 ;
	int width= 0 ;
	double u0 = this->cam.get_u0();
	double v0 = this->cam.get_v0();
	double px = this->cam.get_px();
	double py = this->cam.get_py();

	int retour = SEQUENCE_OK;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE) continue;
		if (SEQUENCE_OK != code_str_to_int ((char*)(node ->name), prop))
		{
			prop = CODE_XML_AUTRE;
			retour = SEQUENCE_ERROR;
		}

		switch (prop)
		{
		case CODE_XML_WIDTH:
			myXmlReadIntChild (doc, node, val, retour);
			width=val;nb++;
			break;
		case CODE_XML_HEIGHT:
			myXmlReadIntChild (doc, node, val, retour);
			height = val;nb++;
			break;

		case CODE_XML_U0:
			myXmlReadDoubleChild (doc, node, vald, retour);
			u0=vald;nb++;
			break;
		case CODE_XML_V0:
			myXmlReadDoubleChild (doc, node, vald, retour);
			v0 = vald;nb++;
			break;
			
		case CODE_XML_PX:
			myXmlReadDoubleChild (doc, node, vald, retour);
			px = vald;nb++;
			break;
		case CODE_XML_PY:
			myXmlReadDoubleChild (doc, node, vald, retour);
			py = vald;nb++;
			break;
			
		default:
			retour = SEQUENCE_ERROR;
			break;
		}

	}



	this->cam.initPersProjWithoutDistortion(px, py, u0, v0) ;

	std::cout <<"**** camera: \n"<<nb <<std::endl;
	std::cout << "u0 "<< this->cam.get_u0() <<std::endl;
	std::cout << "v0 "<< this->cam.get_v0() <<std::endl;
	std::cout << "px "<< this->cam.get_px() <<std::endl;
	std::cout << "py "<< this->cam.get_py() <<std::endl;

	if (nb != 6){
		std::cout <<"ERROR in 'camera' field:\n";
		std::cout << "it must contain 6 parameters\n";

		return SEQUENCE_ERROR;
	}

	return retour;
}


int vpMbtXmlParser::
lecture_mask (xmlDocPtr doc, xmlNodePtr node)
{
	int prop;
	/* Compteur du nombre de parametres lus. */
	int nb = 0;
	/* Valeur lue dans le XML. */
	int val;

	int size = this->ecm.mask_size;
	int nb_mask = this->ecm.n_mask;

	int retour = SEQUENCE_OK;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE) continue;
		if (SEQUENCE_OK != code_str_to_int ((char*)(node ->name), prop))
		{

			prop = CODE_XML_AUTRE;
			retour = SEQUENCE_ERROR;
		}

		switch (prop)
		{
		case CODE_XML_SIZE:

			myXmlReadIntChild (doc, node, val, retour);
			size=val;nb++;
			break;

		case CODE_XML_NB_MASK:

			myXmlReadIntChild (doc, node, val, retour);
			nb_mask = val;nb++;
			break;

		default:
			retour = SEQUENCE_ERROR;
			break;
		}

	}


	this->ecm.mask_size=size;
	this->ecm.n_mask=nb_mask;

	this->ecm.setMaskSize(size) ;
	this->ecm.setMaskNumber(nb_mask);

	std::cout << "**** mask:\n";
	std::cout << "size "<< this->ecm.mask_size<<std::endl;
	std::cout << "nb_mask "<< this->ecm.n_mask<<std::endl;

	if (nb != 2){
		std::cout <<"ERROR in 'mask' field:\n";
		std::cout << "it must contain  2 parameters\n";

		return SEQUENCE_ERROR;
	}
	return retour;
}

int vpMbtXmlParser::
lecture_range (xmlDocPtr doc, xmlNodePtr node)
{
	int prop;
	/* Compteur du nombre de parametres lus. */
	int nb = 0;
	/* Valeur lue dans le XML. */
	int val;

	int tracking = this->ecm.range;

	int retour = SEQUENCE_OK;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE) continue;
		if (SEQUENCE_OK != code_str_to_int ((char*)(node ->name), prop))
		{

			prop = CODE_XML_AUTRE;
			retour = SEQUENCE_ERROR;
		}

		switch (prop)
		{
		case CODE_XML_TRACKING:

			myXmlReadIntChild (doc, node, val, retour);
			tracking = val;nb++;
			break;

		default:
			retour = SEQUENCE_ERROR;
			break;
		}

	}
	
	this->ecm.range = tracking;

	std::cout <<"**** range:\n";
	std::cout <<"tracking "<< this->ecm.range<<std::endl;
	if (nb != 1){
		std::cout <<"ERROR in 'range' field:\n";
		std::cout << "it must contain  1 parameters\n";

		return SEQUENCE_ERROR;
	}

	return retour;
}

int vpMbtXmlParser::
lecture_contrast (xmlDocPtr doc, xmlNodePtr node)
{
	int prop;
	/* Compteur du nombre de parametres lus. */
	int nb = 0;
	/* Valeur lue dans le XML. */
	double val;

	double edge_threshold = this->ecm.threshold;
	double mu1 = this->ecm.mu1;
	double mu2 = this->ecm.mu2;

	int retour = SEQUENCE_OK;

	for (node = node->xmlChildrenNode; node != NULL;  node = node->next)
	{
		if (node->type != XML_ELEMENT_NODE) continue;
		if (SEQUENCE_OK != code_str_to_int ((char*)(node ->name), prop))
		{

			prop = CODE_XML_AUTRE;
			retour = SEQUENCE_ERROR;
		}

		switch (prop)
		{
		case CODE_XML_EDGE_THRESHOLD:

			myXmlReadDoubleChild (doc, node, val, retour);
			edge_threshold=val;nb++;
			break;

		case CODE_XML_MU1:

			myXmlReadDoubleChild (doc, node, val, retour);
			mu1=val;nb++;
			break;

		case CODE_XML_MU2:

			myXmlReadDoubleChild (doc, node, val, retour);
			mu2= val;nb++;
			break;

		default:
			retour = SEQUENCE_ERROR;
			break;
		}

	}

	this->ecm.mu1 = mu1;
	this->ecm.mu2 = mu2;
	this->ecm.threshold = edge_threshold;

	std::cout <<"**** contrast:\n";
	std::cout <<"mu1 " << this->ecm.mu1<<std::endl;
	std::cout <<"mu2 " << this->ecm.mu2<<std::endl;
	std::cout <<"threshold " << this->ecm.threshold<<std::endl;

	if (nb != 3){
		std::cout <<"ERROR in 'contrast' field:\n";
		std::cout << "it must contain  3 parameters\n";

		return SEQUENCE_ERROR;
	}


	return retour;
}



int
vpMbtXmlParser::code_str_to_int (char * str, int & res)
{
	int val_int = -1;
	int retour = vpMbtXmlParser::SEQUENCE_OK;


	if (! strcmp (str,  LABEL_XML_ECM))
	{
		val_int = vpMbtXmlParser::CODE_XML_ECM;
	}
	else if (! strcmp (str,  LABEL_XML_MASK))
	{
		val_int = vpMbtXmlParser::CODE_XML_MASK;
	}
	else if (! strcmp (str,  LABEL_XML_SIZE))
	{
		val_int = vpMbtXmlParser::CODE_XML_SIZE;
	}
	else if (! strcmp (str,  LABEL_XML_NB_MASK))
	{
		val_int = vpMbtXmlParser::CODE_XML_NB_MASK;
	}
	else if (! strcmp (str,  LABEL_XML_RANGE))
	{
		val_int = vpMbtXmlParser::CODE_XML_RANGE;
	}
	else if (! strcmp (str,  LABEL_XML_INIT))
	{
		val_int = vpMbtXmlParser::CODE_XML_INIT;
	}
	else if (! strcmp (str,  LABEL_XML_TRACKING))
	{
		val_int = vpMbtXmlParser::CODE_XML_TRACKING;
	}
	else if (! strcmp (str,  LABEL_XML_CONTRAST))
	{
		val_int = vpMbtXmlParser::CODE_XML_CONTRAST;
	}
	else if (! strcmp (str,  LABEL_XML_EDGE_THRESHOLD))
	{
		val_int = vpMbtXmlParser::CODE_XML_EDGE_THRESHOLD;
	}
	else if (! strcmp (str,  LABEL_XML_MU1))
	{
		val_int = vpMbtXmlParser::CODE_XML_MU1;
	}
	else if (! strcmp (str,  LABEL_XML_MU2))
	{
		val_int = vpMbtXmlParser::CODE_XML_MU2;
	}
	else if (! strcmp (str,  LABEL_XML_SAMPLE))
	{
		val_int = vpMbtXmlParser::CODE_XML_SAMPLE;
	}
	else if (! strcmp (str,  LABEL_XML_STEP))
	{
		val_int = vpMbtXmlParser::CODE_XML_STEP;
	}
	else if (! strcmp (str,  LABEL_XML_NB_SAMPLE))
	{
		val_int = vpMbtXmlParser::CODE_XML_NB_SAMPLE;
	}
	else if (! strcmp (str,  LABEL_XML_CAMERA))
	{
		val_int = vpMbtXmlParser::CODE_XML_CAMERA;
	}
	else if (! strcmp (str,  LABEL_XML_HEIGHT))
	{
		val_int = vpMbtXmlParser::CODE_XML_HEIGHT;
	}
	else if (! strcmp (str,  LABEL_XML_WIDTH))
	{
		val_int = vpMbtXmlParser::CODE_XML_WIDTH;
	}
	else if (! strcmp (str,  LABEL_XML_U0))
	{
		val_int = vpMbtXmlParser::CODE_XML_U0;
	}
	else if (! strcmp (str,  LABEL_XML_V0))
	{
		val_int = vpMbtXmlParser::CODE_XML_V0;
	}
	else if (! strcmp (str,  LABEL_XML_PX))
	{
		val_int = vpMbtXmlParser::CODE_XML_PX;
	}
	else if (! strcmp (str,  LABEL_XML_PY))
	{
		val_int = vpMbtXmlParser::CODE_XML_PY;
	}
	else
	{
		val_int = vpMbtXmlParser::CODE_XML_AUTRE;
	}
	res = val_int;

	return retour;
}
#endif

#endif

