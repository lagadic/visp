/****************************************************************************
 *
 * $Id: vpMbtKltXmlParser.cpp 3672 2012-04-04 15:49:57Z ayol $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Read MBT KLT Tracker information in an XML file
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/
#include <visp/vpConfig.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS


#ifdef VISP_HAVE_XML2

#include <iostream>
#include <map>

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */

#include <visp/vpMbtKltXmlParser.h>


/*!
  Default constructor. 
  
*/
vpMbtKltXmlParser::vpMbtKltXmlParser()
{
  init();
}

/*!
  Default destructor.
*/
vpMbtKltXmlParser::~vpMbtKltXmlParser()
{
}

/*!
  Initialise internal variables (including the map).
*/
void 
vpMbtKltXmlParser::init()
{
  setMainTag("conf");

  nodeMap["conf"] = conf;
  nodeMap["klt"] = klt;
  nodeMap["mask_border"] = mask_border;
  nodeMap["threshold_outlier"] = threshold_outlier;
  nodeMap["max_features"] = max_features;
  nodeMap["window_size"] = window_size;
  nodeMap["quality"] = quality;
  nodeMap["min_distance"] = min_distance;
  nodeMap["harris"] = harris;
  nodeMap["size_block"] = size_block;
  nodeMap["pyramid_lvl"] = pyramid_lvl;
  nodeMap["angle_appear"] = angle_appear;
  nodeMap["angle_desappear"] = angle_desappear;
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
vpMbtKltXmlParser::parse(const char * filename)
{
  std::string file = filename;
  vpXmlParser::parse(file);
}

/*!
  Write info to file.
  
  \waning Useless, so not yet implemented => Throw exception.
*/
void 
vpMbtKltXmlParser::writeMainClass(xmlNodePtr /*node*/)
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
vpMbtKltXmlParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case klt:{
          this->read_klt(doc, dataNode);
          nb++;
          }break;
        case camera:{
          this->read_camera(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in read_sample : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb != 2){
		std::cout <<"ERROR in 'CONF' field:\n";
		std::cout << "it must contain 2 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract CONF informations.");
	}
}

/*!
  Read klt informations.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera informations.
*/
void 
vpMbtKltXmlParser::read_klt(xmlDocPtr doc, xmlNodePtr node)
{
	unsigned int nb=0;
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case mask_border:{
          maskBorder = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case threshold_outlier:{
          threshold = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case max_features:{
          maxFeatures = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case window_size:{
          winSize = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case quality:{
          qualityValue = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case min_distance:{
          minDist = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case harris:{
          harrisParam = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case size_block:{
          blockSize = xmlReadIntChild(doc, dataNode);
          nb++;
          }break;
        case pyramid_lvl:{
          pyramidLevels = xmlReadIntChild(doc, dataNode);
          nb++;
          }break; 
        case angle_appear:{
          angleAppear = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        case angle_desappear:{
          angleDesappear = xmlReadDoubleChild(doc, dataNode);
          nb++;
          }break;
        default:{
//          vpTRACE("unknown tag in read_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(nb == 11){
	  std::cout <<"**** KLT: \n"<< nb <<std::endl;
	  std::cout << "Mask Border "<< maskBorder <<std::endl;
    std::cout << "Threshold Outlier "<< threshold <<std::endl;
	  std::cout << "Max Features "<< maxFeatures <<std::endl;
	  std::cout << "Windows Size "<< winSize <<std::endl;
	  std::cout << "Quality "<< qualityValue <<std::endl;
    std::cout << "Min Distance "<< minDist <<std::endl;
    std::cout << "Harris Parameter "<< harrisParam <<std::endl;
    std::cout << "Block Size "<< blockSize <<std::endl;
    std::cout << "Pyramid Levels "<< pyramidLevels <<std::endl;
    std::cout << "Angle Appear "<< angleAppear <<std::endl;
    std::cout << "Angle Desappear "<< angleDesappear <<std::endl;
  }
	else{
		std::cout <<"ERROR in 'KLT' field:\n";
		std::cout << "it must contain  11 parameters\n";
    throw vpException(vpException::fatalError, "Bad number of data to extract camera informations.");
	}
}

/*!
  Read camera informations.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera informations.
*/
void 
vpMbtKltXmlParser::read_camera (xmlDocPtr doc, xmlNodePtr node)
{
    // current data values.
//  int d_height=0 ;
//  int d_width= 0 ;
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
//          vpTRACE("unknown tag in read_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
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


#endif

#endif

