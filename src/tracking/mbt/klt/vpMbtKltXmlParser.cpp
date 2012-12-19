/****************************************************************************
 *
 * $Id$
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
  nodeMap["max_features"] = max_features;
  nodeMap["window_size"] = window_size;
  nodeMap["quality"] = quality;
  nodeMap["min_distance"] = min_distance;
  nodeMap["harris"] = harris;
  nodeMap["size_block"] = size_block;
  nodeMap["pyramid_lvl"] = pyramid_lvl;
  nodeMap["face"] = face;
  nodeMap["angle_appear"] = angle_appear;
  nodeMap["angle_disappear"] = angle_disappear;
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
  bool klt_node = false;
  bool camera_node = false;
  bool face_node = false;
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case klt:{
          this->read_klt(doc, dataNode);
          klt_node = true;
          }break;
        case camera:{
          this->read_camera(doc, dataNode);
          camera_node = true;
          }break;
        case face:{
          this->read_face(doc, dataNode);
          face_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_sample : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  if(!klt_node)
    std::cout << "WARNING: KLT Node not specified, default values used" << std::endl;
  
  if(!camera_node)
    std::cout << "WARNING: CAMERA Node not specified, default values used" << std::endl;
  
  if(!face_node)
    std::cout << "WARNING: FACE Node not specified, default values used" << std::endl;
}

/*!
  Read face informations.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera informations.
*/
void 
vpMbtKltXmlParser::read_face(xmlDocPtr doc, xmlNodePtr node)
{
  bool angle_appear_node = false;
  bool angle_disappear_node = false;
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case angle_appear:{
          angleAppear = xmlReadDoubleChild(doc, dataNode);
          angle_appear_node = true;
          }break;
        case angle_disappear:{
          angleDisappear = xmlReadDoubleChild(doc, dataNode);
          angle_disappear_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }
  
  if(!angle_appear_node)
    std::cout << "WARNING: In FACE Node, ANGLE_APPEAR Node not specified, default value used : " << angleAppear << std::endl;
  else
    std::cout << "face : Angle Appear "<< angleAppear <<std::endl;
  
  if(!angle_disappear_node)
    std::cout << "WARNING: In FACE Node, ANGLE_DESAPPEAR Node not specified, default value used : " << angleDisappear << std::endl;
  else
    std::cout << "face : Angle Disappear : "<< angleDisappear <<std::endl;
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
	bool mask_border_node = false;
  bool max_features_node = false;
  bool window_size_node = false;
  bool quality_node = false;
  bool min_distance_node = false;
  bool harris_node = false;
  bool size_block_node = false;
  bool pyramid_lvl_node = false;
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case mask_border:{
          maskBorder = xmlReadUnsignedIntChild(doc, dataNode);
          mask_border_node = true;
          }break;
        case max_features:{
          maxFeatures = xmlReadUnsignedIntChild(doc, dataNode);
          max_features_node = true;
          }break;
        case window_size:{
          winSize = xmlReadUnsignedIntChild(doc, dataNode);
          window_size_node = true;
          }break;
        case quality:{
          qualityValue = xmlReadDoubleChild(doc, dataNode);
          quality_node = true;
          }break;
        case min_distance:{
          minDist = xmlReadDoubleChild(doc, dataNode);
          min_distance_node = true;
          }break;
        case harris:{
          harrisParam = xmlReadDoubleChild(doc, dataNode);
          harris_node = true;
          }break;
        case size_block:{
          blockSize = xmlReadUnsignedIntChild(doc, dataNode);
          size_block_node = true;
          }break;
        case pyramid_lvl:{
          pyramidLevels = xmlReadUnsignedIntChild(doc, dataNode);
          pyramid_lvl_node = true;
          }break; 
        default:{
//          vpTRACE("unknown tag in read_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }
  
  if(!mask_border_node)
    std::cout << "WARNING: In KLT Node, MASK_BORDER Node not specified, default value used : " << maskBorder << std::endl;
  else
    std::cout << "klt : Mask Border : "<< maskBorder <<std::endl;
  
  if(!max_features_node)
    std::cout << "WARNING: In KLT Node, MAX_FEATURES Node not specified, default value used : " << maxFeatures << std::endl;
  else
    std::cout << "klt : Max Features : "<< maxFeatures <<std::endl;
  
  if(!window_size_node)
    std::cout << "WARNING: In KLT Node, WINDOW_SIZE Node not specified, default value used : " << winSize << std::endl;
  else
    std::cout << "klt : Windows Size : "<< winSize <<std::endl;
  
  if(!quality_node)
    std::cout << "WARNING: In KLT Node, QUALITY Node not specified, default value used : " << qualityValue << std::endl;
  else
    std::cout << "klt : Quality : "<< qualityValue <<std::endl;
  
  if(!min_distance_node)
    std::cout << "WARNING: In KLT Node, MIN_DISTANCE Node not specified, default value used : " << minDist << std::endl;
  else
    std::cout << "klt : Min Distance : "<< minDist <<std::endl;
  
  if(!harris_node)
    std::cout << "WARNING: In KLT Node, HARRIS Node not specified, default value used : " << harrisParam << std::endl;
  else
    std::cout << "klt : Harris Parameter : "<< harrisParam <<std::endl;
  
  if(!size_block_node)
    std::cout << "WARNING: In KLT Node, SIZE_BLOCK Node not specified, default value used : " << blockSize << std::endl;
  else
    std::cout << "klt : Block Size : "<< blockSize <<std::endl;
  
  if(!pyramid_lvl_node)
    std::cout << "WARNING: In KLT Node, PYRAMID_LVL Node not specified, default value used : " << pyramidLevels << std::endl;
  else
    std::cout << "klt : Pyramid Levels : "<< pyramidLevels <<std::endl;
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
  bool height_node = false;
  bool width_node = false;
  bool u0_node = false;
  bool v0_node = false;
  bool px_node = false;
  bool py_node = false;
  
    // current data values.
//  int d_height=0 ;
//  int d_width= 0 ;
  double d_u0 = this->cam.get_u0();
  double d_v0 = this->cam.get_v0();
  double d_px = this->cam.get_px();
  double d_py = this->cam.get_py();
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case height:{
          /* d_height = */ xmlReadIntChild(doc, dataNode);
          height_node = true;
          }break;
        case width:{
          /* d_width = */ xmlReadIntChild(doc, dataNode);
          width_node = true;
          }break;
        case u0:{
          d_u0 = xmlReadDoubleChild(doc, dataNode);
          u0_node = true;
          }break;
        case v0:{
          d_v0 = xmlReadDoubleChild(doc, dataNode);
          v0_node = true;
          }break;
        case px:{
          d_px = xmlReadDoubleChild(doc, dataNode);
          px_node = true;
          }break;
        case py:{
          d_py = xmlReadDoubleChild(doc, dataNode);
          py_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }
  
  this->cam.initPersProjWithoutDistortion(d_px, d_py, d_u0, d_v0) ;

  if(!height_node)
    std::cout << "WARNING: In CAMERA Node, HEIGHT Node not specified, default value used" << std::endl;
  
  if(!width_node)
    std::cout << "WARNING: In CAMERA Node, WIDTH Node not specified, default value used" << std::endl;
  
  if(!u0_node)
    std::cout << "WARNING: In CAMERA Node, u0 Node not specified, default value used : " << this->cam.get_u0() << std::endl;
  else
    std::cout << "camera : u0 "<< this->cam.get_u0() <<std::endl;
  
  if(!v0_node)
    std::cout << "WARNING: In CAMERA Node, v0 Node not specified, default value used : " << this->cam.get_v0() << std::endl;
  else
    std::cout << "camera : v0 "<< this->cam.get_v0() <<std::endl;
  
  if(!px_node)
    std::cout << "WARNING: In CAMERA Node, px Node not specified, default value used : " << this->cam.get_px() << std::endl;
  else
    std::cout << "camera : px "<< this->cam.get_px() <<std::endl;
  
  if(!py_node)
    std::cout << "WARNING: In CAMERA Node, py Node not specified, default value used : " << this->cam.get_py() << std::endl;
  else
    std::cout << "camera : py "<< this->cam.get_py() <<std::endl;
}


#endif

#endif

