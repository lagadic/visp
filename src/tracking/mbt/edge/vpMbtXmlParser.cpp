/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
#include <visp/vpConfig.h>


#ifdef VISP_HAVE_XML2

#include <iostream>
#include <map>

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */

#include <visp/vpMbtXmlParser.h>


/*!
  Default constructor. 
  
*/
vpMbtXmlParser::vpMbtXmlParser()
{
  hasNearClipping = false;
  hasFarClipping = false;
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
  nodeMap["face"] = face;
  nodeMap["angle_appear"] = angle_appear;
  nodeMap["angle_disappear"] = angle_disappear;
  nodeMap["near_clipping"] = near_clipping;
  nodeMap["far_clipping"] = far_clipping;
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
vpMbtXmlParser::parse(const char * filename)
{
  std::string file = filename;
  vpXmlParser::parse(file);
}

/*!
  Write info to file.
  
  \warning Useless, so not yet implemented => Throw exception.
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
	bool ecm_node = false;
  bool sample_node = false;
  bool camera_node = false;
  bool face_node = false;
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case ecm:{
          this->read_ecm (doc, dataNode);
          ecm_node = true;
          }break;
        case sample:{
          this->read_sample (doc, dataNode);
          sample_node = true;
          }break;
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
  
  if(!ecm_node)
    std::cout << "WARNING: ECM Node not specified, default values used" << std::endl;
  
  if(!sample_node)
    std::cout << "WARNING: SAMPLE Node not specified, default values used" << std::endl;
  
  if(!camera_node)
    std::cout << "WARNING: CAMERA Node not specified, default values used" << std::endl;
  
  if(!face_node)
    std::cout << "WARNING: FACE Node not specified, default values used" << std::endl;
}


/*!
  Read ecm information.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the ecm information.
*/
void 
vpMbtXmlParser::read_ecm (xmlDocPtr doc, xmlNodePtr node)
{
  bool mask_node = false;
  bool range_node = false;
  bool contrast_node = false;
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case mask:{
          this->read_mask (doc, dataNode);
          mask_node = true;
          }break;
        case range:{
          this->read_range (doc, dataNode);
          range_node = true;
          }break;
        case contrast:{
          this->read_contrast (doc, dataNode);
          contrast_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_ecm : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }
  
  if(!mask_node)
    std::cout << "WARNING: In ECM Node, MASK Node not specified, default values used" << std::endl;
  
  if(!range_node)
    std::cout << "WARNING: In ECM Node, RANGE Node not specified, default values used" << std::endl;
  
  if(!contrast_node)
    std::cout << "WARNING: In ECM Node, CONTRAST Node not specified, default values used" << std::endl;
}

/*!
  Read sample information.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the sample information.
*/
void 
vpMbtXmlParser::read_sample (xmlDocPtr doc, xmlNodePtr node)
{
  bool step_node = false;
  bool nb_sample_node = false;
  
    // current data values.
	double d_stp = this->m_ecm.getSampleStep();
	int d_nb_sample = this->m_ecm.getNbTotalSample();
	
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case step:{
          d_stp = xmlReadIntChild(doc, dataNode);
          step_node = true;
          }break;
        case nb_sample:{
          d_nb_sample = xmlReadIntChild(doc, dataNode);
          nb_sample_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_sample : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }
  
  this->m_ecm.setSampleStep(d_stp);
  this->m_ecm.setNbTotalSample(d_nb_sample);

  if(!step_node)
    std::cout << "WARNING: In SAMPLE Node, STEP Node not specified, default value used : " << this->m_ecm.getSampleStep() << std::endl;
  else
    std::cout <<"sample : sample_step "<< this->m_ecm.getSampleStep()<<std::endl;
  
  if(!nb_sample_node)
    std::cout << "WARNING: In SAMPLE Node, NB_SAMPLE Node not specified, default value used : " << this->m_ecm.getNbTotalSample() << std::endl;
  else
    std::cout <<"sample : n_total_sample "<< this->m_ecm.getNbTotalSample()<<std::endl;
}

/*!
  Read camera information.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera information.
*/
void 
vpMbtXmlParser::read_camera (xmlDocPtr doc, xmlNodePtr node)
{
  bool height_node = false;
  bool width_node = false;
  bool u0_node = false;
  bool v0_node = false;
  bool px_node = false;
  bool py_node = false;
  
    // current data values.
// 	int d_height=0 ;
// 	int d_width= 0 ;
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

/*!
  Read face information.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the camera information.
*/
void 
vpMbtXmlParser::read_face(xmlDocPtr doc, xmlNodePtr node)
{
  bool angle_appear_node = false;
  bool angle_disappear_node = false;
  bool near_clipping_node = false;
  bool far_clipping_node = false;
  
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
  
  if(!near_clipping_node)
    std::cout << "WARNING: In FACE Node, NEAR_CLIPPING Node not specified, no near clipping used" << std::endl;
  else
    std::cout << "face : Near Clipping : "<< nearClipping <<std::endl;
  
  if(!far_clipping_node)
    std::cout << "WARNING: In FACE Node, FAR_CLIPPING Node not specified, no far clipping used" << std::endl;
  else
    std::cout << "face : Far Clipping : "<< farClipping <<std::endl;
}

/*!
  Read mask information for the vpMeSite.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the mask information.
*/
void 
vpMbtXmlParser::read_mask (xmlDocPtr doc, xmlNodePtr node)
{
  bool size_node = false;
  bool nb_mask_node = false;
  
    // current data values.
  unsigned int d_size = this->m_ecm.getMaskSize();
  unsigned int d_nb_mask = this->m_ecm.getMaskNumber();
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case size:{
          d_size = xmlReadUnsignedIntChild(doc, dataNode);
          size_node = true;
          }break;
        case nb_mask:{
          d_nb_mask = xmlReadUnsignedIntChild(doc, dataNode);
          nb_mask_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_mask : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  this->m_ecm.setMaskSize(d_size) ;
  this->m_ecm.setMaskNumber(d_nb_mask);
  
  if(!size_node)
    std::cout << "WARNING: In MASK Node, SIZE Node not specified, default value used : " << this->m_ecm.getMaskSize() << std::endl;
  else
    std::cout << "ecm : mask : size "<< this->m_ecm.getMaskSize() <<std::endl;
  
  if(!nb_mask_node)
    std::cout << "WARNING: In MASK Node, NB_MASK Node not specified, default value used : " << this->m_ecm.getMaskNumber() << std::endl;
  else
    std::cout << "ecm : mask : nb_mask "<< this->m_ecm.getMaskNumber() <<std::endl; 
}

/*!
  Read range information for the vpMeSite.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the range information.
*/
void 
vpMbtXmlParser::read_range (xmlDocPtr doc, xmlNodePtr node)
{
  bool tracking_node = false;
  
    // current data values.
	unsigned int m_range_tracking = this->m_ecm.getRange();
	
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case tracking:{
          m_range_tracking = xmlReadUnsignedIntChild(doc, dataNode);
          tracking_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_range : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  this->m_ecm.setRange(m_range_tracking);
  
  if(!tracking_node)
    std::cout << "WARNING: In RANGE Node, TRACKING Node not specified, default value used : " << this->m_ecm.getRange() << std::endl;
  else
    std::cout <<"ecm : range : tracking "<< this->m_ecm.getRange()<<std::endl;  
}


/*!
  Read the contrast information from the xml file.
  
  \throw vpException::fatalError if there was an unexpected number of data. 
  
  \param doc : Pointer to the document.
  \param node : Pointer to the node of the contrast information.
*/
void
vpMbtXmlParser::read_contrast (xmlDocPtr doc, xmlNodePtr node)
{
  bool edge_threshold_node = false;
  bool mu1_node = false;
  bool mu2_node = false;
  
    // current data values.
	double d_edge_threshold = this->m_ecm.getThreshold();
	double d_mu1 = this->m_ecm.getMu1();
	double d_mu2 = this->m_ecm.getMu2();
  
  for(xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL;  dataNode = dataNode->next)  {
    if(dataNode->type == XML_ELEMENT_NODE){
      std::map<std::string, int>::iterator iter_data= this->nodeMap.find((char*)dataNode->name);
      if(iter_data != nodeMap.end()){
        switch (iter_data->second){
        case edge_threshold:{
          d_edge_threshold = xmlReadDoubleChild(doc, dataNode);
          edge_threshold_node = true;
          }break;
        case mu1:{
          d_mu1 = xmlReadDoubleChild(doc, dataNode);
          mu1_node = true;
          }break;
        case mu2:{
          d_mu2 = xmlReadDoubleChild(doc, dataNode);
          mu2_node = true;
          }break;
        default:{
//          vpTRACE("unknown tag in read_contrast : %d, %s", iter_data->second, (iter_data->first).c_str());
          }break;
        }
      }
    }
  }

  this->m_ecm.setMu1(d_mu1);
  this->m_ecm.setMu2(d_mu2);
  this->m_ecm.setThreshold(d_edge_threshold);
  
  if(!edge_threshold_node)
    std::cout << "WARNING: In CONTRAST Node, EDGE_THRESHOLD Node not specified, default value used : " << this->m_ecm.getThreshold() << std::endl;
  else
    std::cout <<"ecm : contrast : threshold " << this->m_ecm.getThreshold()<<std::endl;
  
  if(!mu1_node)
    std::cout << "WARNING: In CONTRAST Node, mu1 Node not specified, default value used : " << this->m_ecm.getMu1() << std::endl;
  else
    std::cout <<"ecm : contrast : mu1 " << this->m_ecm.getMu1()<<std::endl;
  
  if(!mu2_node)
    std::cout << "WARNING: In CONTRAST Node, mu2 Node not specified, default value used : " << this->m_ecm.getMu2() << std::endl;
  else
    std::cout <<"ecm : contrast : mu2 " << this->m_ecm.getMu2()<<std::endl;
}

#endif

