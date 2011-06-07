/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
#include <map>

/*!
  Default constructor. 
  
*/
vpMbtXmlParser::vpMbtXmlParser()
{
  init();
}

/*!
  Default destructor.
*/
vpMbtXmlParser::~vpMbtXmlParser()
{
}

/*!
  Initialise internal variables (including the map).
*/
void 
vpMbtXmlParser::init()
{
  setMainTag("conf");

  nodeMap["conf"] = conf;
  nodeMap["ecm"] = ecm;
  nodeMap["mask"] = mask;
  nodeMap["size"] = size;
  nodeMap["nb_mask"] = nb_mask;
  nodeMap["range"] = range;
  nodeMap["tracking"] = tracking;
  nodeMap["contrast"] = contrast;
  nodeMap["edge_threshold"] = edge_threshold;
  nodeMap["mu1"] = mu1;
  nodeMap["mu2"] = mu2;
  nodeMap["sample"] = sample;
  nodeMap["step"] = step;
  nodeMap["nb_sample"] = nb_sample;
  nodeMap["camera"] = camera;
  nodeMap["height"] = height;
  nodeMap["width"] = width;
  nodeMap["u0"] = u0;
  nodeMap["v0"] = v0;
  nodeMap["px"] = px;
  nodeMap["py"] = py;
  
}

/*!
  Parse the file in parameters.
  This method is deprecated, use parse() instead.
  
  \paran filename : File to parse.
*/
void
vpMbtXmlParser::parse(const char * filename)
{
  std::string file = filename;
  vpXmlParser::parse(file);
}

/*!
  Write info to file.
  
  \waning Useless, so not yet implemented => Throw exception.
*/
void 
vpMbtXmlParser::writeMainClass(xmlNodePtr /*node*/)
{
  throw vpException(vpException::notImplementedError, "Not yet implemented." );
}

/*!
  Read the parameters of the class from the file given by its document pointer 
  and by its root node. 
  
  \param doc : Document to parse.
  \param node : Root node. 
*/
void
vpMbtXmlParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case ecm:{
          this->lecture_ecm (doc, dataNode);
          nb++;
          }break;
        case sample:{
          this->lecture_sample (doc, dataNode);
          nb++;
          }break;
        case camera:{
          this->lecture_camera (doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_sample : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb != 3){
		std::cout <<"ERROR in 'ECM' field:\n";
		std::cout << "it must contain 3 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract ECM informations.");
	}
}


/*!
  Read ecm informations.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the ecm informations.
*/
void 
vpMbtXmlParser::lecture_ecm (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case mask:{
          this->lecture_mask (doc, dataNode);
          nb++;
          }break;
        case range:{
          this->lecture_range (doc, dataNode);
          nb++;
          }break;
        case contrast:{
          this->lecture_contrast (doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_ecm : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb != 3){
		std::cout <<"ERROR in 'ECM' field:\n";
		std::cout << "it must contain 3 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract ECM informations.");
	}
}

/*!
  Read sample informations.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the sample informations.
*/
void 
vpMbtXmlParser::lecture_sample (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
	int d_stp = this->m_ecm.sample_step;
	int d_nb_sample = this->m_ecm.ntotal_sample;
	
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case step:{
          d_stp = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case nb_sample:{
          d_nb_sample = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_sample : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb == 2){
	  this->m_ecm.sample_step = d_stp;
	  this->m_ecm.ntotal_sample = d_nb_sample;

	  std::cout <<"**** sample:\n";
	  std::cout <<"sample_step "<< this->m_ecm.sample_step<<std::endl;
	  std::cout <<"n_total_sample "<< this->m_ecm.ntotal_sample<<std::endl;
  }
	else{
		std::cout <<"ERROR in 'sample' field:\n";
		std::cout << "it must contain 2 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract sample informations.");
	}
}

/*!
  Read camera informations.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera informations.
*/
void 
vpMbtXmlParser::lecture_camera (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
// 	int d_height=0 ;
// 	int d_width= 0 ;
	double d_u0 = this->cam.get_u0();
	double d_v0 = this->cam.get_v0();
	double d_px = this->cam.get_px();
	double d_py = this->cam.get_py();
	
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case height:{
          /* d_height = */ xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case width:{
          /* d_width = */ xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case u0:{
          d_u0 = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case v0:{
          d_v0 = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case px:{
          d_px = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case py:{
          d_py = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb == 6){
	  this->cam.initPersProjWithoutDistortion(d_px, d_py, d_u0, d_v0) ;

	  std::cout <<"**** camera: \n"<<nb <<std::endl;
	  std::cout << "u0 "<< this->cam.get_u0() <<std::endl;
	  std::cout << "v0 "<< this->cam.get_v0() <<std::endl;
	  std::cout << "px "<< this->cam.get_px() <<std::endl;
	  std::cout << "py "<< this->cam.get_py() <<std::endl;
  }
	else{
		std::cout <<"ERROR in 'camera' field:\n";
		std::cout << "it must contain  6 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract camera informations.");
	}
}

/*!
  Read mask informations for the vpMeSite.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the mask informations.
*/
void 
vpMbtXmlParser::lecture_mask (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
  unsigned int d_size = this->m_ecm.getMaskSize();
  unsigned int d_nb_mask = this->m_ecm.getMaskNumber();
	
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case size:{
          d_size = xmlReadUnsignedIntChild(doc, dataNode);
          nb++;
          }break;
        case nb_mask:{
          d_nb_mask = xmlReadUnsignedIntChild(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_mask : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb == 2){
	  this->m_ecm.setMaskSize(d_size) ;
	  this->m_ecm.setMaskNumber(d_nb_mask);
	
	  std::cout << "**** mask:\n";
    std::cout << "size "<< this->m_ecm.getMaskSize() <<std::endl;
    std::cout << "nb_mask "<< this->m_ecm.getMaskNumber() <<std::endl;
  }
	else{
		std::cout <<"ERROR in 'mask' field:\n";
		std::cout << "it must contain  2 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract mask informations.");
	}
}

/*!
  Read range informations for the vpMeSite.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the range informations.
*/
void 
vpMbtXmlParser::lecture_range (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
	unsigned int m_range_tracking = this->m_ecm.range;
	
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case tracking:{
          m_range_tracking = xmlReadUnsignedIntChild(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_range : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb == 1){
	  this->m_ecm.range = m_range_tracking;
	  std::cout <<"**** range:\n";
	  std::cout <<"tracking "<< this->m_ecm.range<<std::endl;
  }
	else{
		std::cout <<"ERROR in 'range' field:\n";
		std::cout << "it must contain  1 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract range informations.");
	}
}


/*!
  Read the contrast informations from the xml file.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the contrast informations.
*/
void
vpMbtXmlParser::lecture_contrast (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
	double d_edge_threshold = this->m_ecm.threshold;
	double d_mu1 = this->m_ecm.mu1;
	double d_mu2 = this->m_ecm.mu2;
	
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case edge_threshold:{
          d_edge_threshold = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case mu1:{
          d_mu1 = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case mu2:{
          d_mu2 = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in lecture_contrast : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb == 3){
	  this->m_ecm.mu1 = d_mu1;
	  this->m_ecm.mu2 = d_mu2;
	  this->m_ecm.threshold = d_edge_threshold;

	  std::cout <<"**** contrast:\n";
	  std::cout <<"mu1 " << this->m_ecm.mu1<<std::endl;
	  std::cout <<"mu2 " << this->m_ecm.mu2<<std::endl;
	  std::cout <<"threshold " << this->m_ecm.threshold<<std::endl;
  }
	else{
		std::cout <<"ERROR in 'contrast' field:\n";
		std::cout << "it must contain  3 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract contrast informations.");
	}
}


#endif

#endif

