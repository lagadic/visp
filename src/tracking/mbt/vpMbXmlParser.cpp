/****************************************************************************
 *
 * $Id: vpMbXmlParser.cpp 4577 2014-01-10 16:19:41Z fspindle $
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
 * Load XML Parameter for Model Based Tracker.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/
#include <visp/vpConfig.h>


#ifdef VISP_HAVE_XML2

#include <iostream>
#include <map>

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */

#include <visp/vpMbXmlParser.h>


/*!
  Default constructor. 
  
*/
vpMbXmlParser::vpMbXmlParser()
  : cam(), angleAppear(70), angleDisappear(80),
    hasNearClipping(false), nearClipping(false),
    hasFarClipping(false), farClipping(false), fovClipping(false)

{
  init();
}

/*!
  Default destructor.
*/
vpMbXmlParser::~vpMbXmlParser()
{
}

/*!
  Initialise internal variables (including the map).
*/
void 
vpMbXmlParser::init()
{
  setMainTag("conf");

  nodeMap["conf"] = conf;
  nodeMap["face"] = face;
  nodeMap["angle_appear"] = angle_appear;
  nodeMap["angle_disappear"] = angle_disappear;
  nodeMap["near_clipping"] = near_clipping;
  nodeMap["far_clipping"] = far_clipping;
  nodeMap["fov_clipping"] = fov_clipping;
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
  
  \param filename : File to parse.
*/
void
vpMbXmlParser::parse(const char * filename)
{
  std::string file = filename;
  vpXmlParser::parse(file);
}

/*!
  Write info to file.
  
  \warning Useless, so not yet implemented => Throw exception.
*/
void 
vpMbXmlParser::writeMainClass(xmlNodePtr /*node*/)
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
vpMbXmlParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
  bool camera_node = false;
  bool face_node = false;
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case camera:{
          this->read_camera (doc, dataNode);
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
  
  if(!camera_node) {
    std::cout << "camera : u0 : "<< this->cam.get_u0() << " (default)" <<std::endl;
    std::cout << "camera : v0 : "<< this->cam.get_v0() << " (default)" <<std::endl;
    std::cout << "camera : px : "<< this->cam.get_px() << " (default)" <<std::endl;
    std::cout << "camera : py : "<< this->cam.get_py() << " (default)" <<std::endl;
  }
  
  if(!face_node) {
    std::cout << "face : Angle Appear : "<< angleAppear <<" (default)" <<std::endl;
    std::cout << "face : Angle Disappear : "<< angleDisappear <<" (default)" <<std::endl;
  }
}

/*!
  Read camera information.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera information.
*/
void 
vpMbXmlParser::read_camera (xmlDocPtr doc, xmlNodePtr node)
{
  bool u0_node = false;
  bool v0_node = false;
  bool px_node = false;
  bool py_node = false;
  
    // current data values.
	double d_u0 = this->cam.get_u0();
	double d_v0 = this->cam.get_v0();
	double d_px = this->cam.get_px();
	double d_py = this->cam.get_py();
	
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
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
  
  if(!u0_node)
    std::cout << "camera : u0 : "<< this->cam.get_u0() << " (default)" <<std::endl;
  else
    std::cout << "camera : u0 : "<< this->cam.get_u0() <<std::endl;
  
  if(!v0_node)
    std::cout << "camera : v0 : "<< this->cam.get_v0() << " (default)" <<std::endl;
  else
    std::cout << "camera : v0 : "<< this->cam.get_v0() <<std::endl;
  
  if(!px_node)
    std::cout << "camera : px : "<< this->cam.get_px() << " (default)" <<std::endl;
  else
    std::cout << "camera : px : "<< this->cam.get_px() <<std::endl;

  if(!py_node)
    std::cout << "camera : py : "<< this->cam.get_py() << " (default)" <<std::endl;
  else
    std::cout << "camera : py : "<< this->cam.get_py() <<std::endl;
}

/*!
  Read face information.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera information.
*/
void 
vpMbXmlParser::read_face(xmlDocPtr doc, xmlNodePtr node)
{
  bool angle_appear_node = false;
  bool angle_disappear_node = false;
  bool near_clipping_node = false;
  bool far_clipping_node = false;
  bool fov_clipping_node = false;
  
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
        case near_clipping:{
          nearClipping = xmlReadDoubleChild(doc, dataNode);
          near_clipping_node = true;
          hasNearClipping = true;
          }break;
        case far_clipping:{
          farClipping = xmlReadDoubleChild(doc, dataNode);
          far_clipping_node = true;
          hasFarClipping = true;
          }break;
        case fov_clipping:{
          if (xmlReadIntChild(doc, dataNode))
            fovClipping = true;
          else
            fovClipping = false;
          fov_clipping_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_camera : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }
  
  if(!angle_appear_node)
    std::cout << "face : Angle Appear : "<< angleAppear << " (default)" <<std::endl;
  else
    std::cout << "face : Angle Appear : "<< angleAppear <<std::endl;
  
  if(!angle_disappear_node)
    std::cout << "face : Angle Disappear : "<< angleDisappear << " (default)" <<std::endl;
  else
    std::cout << "face : Angle Disappear : "<< angleDisappear <<std::endl;
  
  if(near_clipping_node)
    std::cout << "face : Near Clipping : "<< nearClipping <<std::endl;
  
  if(far_clipping_node)
    std::cout << "face : Far Clipping : "<< farClipping <<std::endl;
  
  if(fov_clipping_node) {
    if(fovClipping)
      std::cout << "face : Fov Clipping : True" <<std::endl;
    else
      std::cout << "face : Fov Clipping : False" <<std::endl;
  }
}

#endif

