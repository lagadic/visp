/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <iostream>
#include <map>

#include <libxml/xmlmemory.h> /* Fonctions de la lib XML.                */

#include <visp3/mbt/vpMbtXmlParser.h>

/*!
  Default constructor.

*/
vpMbtXmlParser::vpMbtXmlParser() : m_ecm() { init(); }

/*!
  Default destructor.
*/
vpMbtXmlParser::~vpMbtXmlParser() {}

/*!
  Initialise internal variables (including the map).
*/
void vpMbtXmlParser::init()
{
  vpMbXmlParser::init();

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
}

/*!
  Write info to file.

  \warning Useless, so not yet implemented => Throw exception.
*/
void vpMbtXmlParser::writeMainClass(xmlNodePtr /*node*/)
{
  throw vpException(vpException::notImplementedError, "Not yet implemented.");
}

/*!
  Read the parameters of the class from the file given by its document pointer
  and by its root node.

  \param doc : Document to parse.
  \param node : Root node.
*/
void vpMbtXmlParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
  bool camera_node = false;
  bool face_node = false;
  bool ecm_node = false;
  bool lod_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case camera: {
          this->read_camera(doc, dataNode);
          camera_node = true;
        } break;
        case face: {
          this->read_face(doc, dataNode);
          face_node = true;
        } break;
        case ecm: {
          this->read_ecm(doc, dataNode);
          ecm_node = true;
        } break;
        case sample: {
          this->read_sample_deprecated(doc, dataNode);
        } break;
        case lod: {
          this->read_lod(doc, dataNode);
          lod_node = true;
        } break;
        default: {
          //          vpTRACE("unknown tag in read_sample : %d, %s",
          //          iter_data->second, (iter_data->first).c_str());
        } break;
        }
      }
    }
  }

  if (!camera_node) {
    std::cout << "camera : u0 : " << this->cam.get_u0() << " (default)" << std::endl;
    std::cout << "camera : v0 : " << this->cam.get_v0() << " (default)" << std::endl;
    std::cout << "camera : px : " << this->cam.get_px() << " (default)" << std::endl;
    std::cout << "camera : py : " << this->cam.get_py() << " (default)" << std::endl;
  }

  if (!face_node) {
    std::cout << "face : Angle Appear : " << angleAppear << " (default)" << std::endl;
    std::cout << "face : Angle Disappear : " << angleDisappear << " (default)" << std::endl;
  }

  if (!ecm_node) {
    std::cout << "ecm : mask : size : " << this->m_ecm.getMaskSize() << " (default)" << std::endl;
    std::cout << "ecm : mask : nb_mask : " << this->m_ecm.getMaskNumber() << " (default)" << std::endl;
    std::cout << "ecm : range : tracking : " << this->m_ecm.getRange() << " (default)" << std::endl;
    std::cout << "ecm : contrast : threshold : " << this->m_ecm.getThreshold() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu1 : " << this->m_ecm.getMu1() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu2 : " << this->m_ecm.getMu2() << " (default)" << std::endl;
    std::cout << "ecm : sample : sample_step : " << this->m_ecm.getSampleStep() << " (default)" << std::endl;
  }

  if (!lod_node) {
    std::cout << "lod : use lod : " << useLod << " (default)" << std::endl;
    std::cout << "lod : min line length threshold : " << minLineLengthThreshold << " (default)" << std::endl;
    std::cout << "lod : min polygon area threshold : " << minPolygonAreaThreshold << " (default)" << std::endl;
  }
}

/*!
  Read ecm information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the ecm information.
*/
void vpMbtXmlParser::read_ecm(xmlDocPtr doc, xmlNodePtr node)
{
  bool mask_node = false;
  bool range_node = false;
  bool contrast_node = false;
  bool sample_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case mask: {
          this->read_mask(doc, dataNode);
          mask_node = true;
        } break;
        case range: {
          this->read_range(doc, dataNode);
          range_node = true;
        } break;
        case contrast: {
          this->read_contrast(doc, dataNode);
          contrast_node = true;
        } break;
        case sample: {
          this->read_sample(doc, dataNode);
          sample_node = true;
        } break;
        default: {
          //          vpTRACE("unknown tag in read_ecm : %d, %s",
          //          iter_data->second, (iter_data->first).c_str());
        } break;
        }
      }
    }
  }

  if (!mask_node) {
    std::cout << "ecm : mask : size : " << this->m_ecm.getMaskSize() << " (default)" << std::endl;
    std::cout << "ecm : mask : nb_mask : " << this->m_ecm.getMaskNumber() << " (default)" << std::endl;
  }

  if (!range_node) {
    std::cout << "ecm : range : tracking : " << this->m_ecm.getRange() << " (default)" << std::endl;
  }

  if (!contrast_node) {
    std::cout << "ecm : contrast : threshold " << this->m_ecm.getThreshold() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu1 " << this->m_ecm.getMu1() << " (default)" << std::endl;
    std::cout << "ecm : contrast : mu2 " << this->m_ecm.getMu2() << " (default)" << std::endl;
  }

  if (!sample_node) {
    std::cout << "ecm : sample : sample_step : " << this->m_ecm.getSampleStep() << " (default)" << std::endl;
  }
}

/*!
  Read sample information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the sample information.
*/
void vpMbtXmlParser::read_sample(xmlDocPtr doc, xmlNodePtr node)
{
  bool step_node = false;

  // current data values.
  double d_stp = this->m_ecm.getSampleStep();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case step: {
          d_stp = xmlReadIntChild(doc, dataNode);
          step_node = true;
        } break;
        default: {
          //          vpTRACE("unknown tag in read_sample : %d, %s",
          //          iter_data->second, (iter_data->first).c_str());
        } break;
        }
      }
    }
  }

  this->m_ecm.setSampleStep(d_stp);
  //  this->m_ecm.setNbTotalSample(d_nb_sample);

  if (!step_node)
    std::cout << "ecm : sample : sample_step : " << this->m_ecm.getSampleStep() << " (default)" << std::endl;
  else
    std::cout << "ecm : sample : sample_step : " << this->m_ecm.getSampleStep() << std::endl;
}

/*!
  Read sample information.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the sample information.
*/
void vpMbtXmlParser::read_sample_deprecated(xmlDocPtr doc, xmlNodePtr node)
{
  bool step_node = false;
  // bool nb_sample_node = false;

  // current data values.
  double d_stp = this->m_ecm.getSampleStep();
  //  int d_nb_sample = this->m_ecm.getNbTotalSample();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case step: {
          d_stp = xmlReadIntChild(doc, dataNode);
          step_node = true;
        } break;
        //        case nb_sample:{
        //          d_nb_sample = xmlReadIntChild(doc, dataNode);
        //          nb_sample_node = true;
        //          }break;
        default: {
          //          vpTRACE("unknown tag in read_sample : %d, %s",
          //          iter_data->second, (iter_data->first).c_str());
        } break;
        }
      }
    }
  }

  this->m_ecm.setSampleStep(d_stp);
  //  this->m_ecm.setNbTotalSample(d_nb_sample);

  if (!step_node)
    std::cout << "[DEPRECATED] sample : sample_step : " << this->m_ecm.getSampleStep() << " (default)" << std::endl;
  else
    std::cout << "[DEPRECATED] sample : sample_step : " << this->m_ecm.getSampleStep() << std::endl;

  //  if(!nb_sample_node)
  //    std::cout <<"sample : n_total_sample : "<<
  //    this->m_ecm.getNbTotalSample()<< " (default)"<<std::endl;
  //  else
  //    std::cout <<"sample : n_total_sample : "<<
  //    this->m_ecm.getNbTotalSample()<<std::endl;

  std::cout << "  WARNING : This node (sample) is deprecated." << std::endl;
  std::cout << "  It should be moved in the ecm node (ecm : sample)." << std::endl;
}

/*!
  Read mask information for the vpMeSite.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the mask information.
*/
void vpMbtXmlParser::read_mask(xmlDocPtr doc, xmlNodePtr node)
{
  bool size_node = false;
  bool nb_mask_node = false;

  // current data values.
  unsigned int d_size = this->m_ecm.getMaskSize();
  unsigned int d_nb_mask = this->m_ecm.getMaskNumber();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case size: {
          d_size = xmlReadUnsignedIntChild(doc, dataNode);
          size_node = true;
        } break;
        case nb_mask: {
          d_nb_mask = xmlReadUnsignedIntChild(doc, dataNode);
          nb_mask_node = true;
        } break;
        default: {
          //          vpTRACE("unknown tag in read_mask : %d, %s",
          //          iter_data->second, (iter_data->first).c_str());
        } break;
        }
      }
    }
  }

  this->m_ecm.setMaskSize(d_size);
  // Check to ensure that d_nb_mask > 0
  if (!d_nb_mask)
    throw(vpException(vpException::badValue, "Model-based tracker mask size "
                                             "parameter should be different "
                                             "from zero in xml file"));
  this->m_ecm.setMaskNumber(d_nb_mask);

  if (!size_node)
    std::cout << "ecm : mask : size : " << this->m_ecm.getMaskSize() << " (default)" << std::endl;
  else
    std::cout << "ecm : mask : size : " << this->m_ecm.getMaskSize() << std::endl;

  if (!nb_mask_node)
    std::cout << "ecm : mask : nb_mask : " << this->m_ecm.getMaskNumber() << " (default)" << std::endl;
  else
    std::cout << "ecm : mask : nb_mask : " << this->m_ecm.getMaskNumber() << std::endl;
}

/*!
  Read range information for the vpMeSite.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the range information.
*/
void vpMbtXmlParser::read_range(xmlDocPtr doc, xmlNodePtr node)
{
  bool tracking_node = false;

  // current data values.
  unsigned int m_range_tracking = this->m_ecm.getRange();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case tracking: {
          m_range_tracking = xmlReadUnsignedIntChild(doc, dataNode);
          tracking_node = true;
        } break;
        default: {
          //          vpTRACE("unknown tag in read_range : %d, %s",
          //          iter_data->second, (iter_data->first).c_str());
        } break;
        }
      }
    }
  }

  this->m_ecm.setRange(m_range_tracking);

  if (!tracking_node)
    std::cout << "ecm : range : tracking : " << this->m_ecm.getRange() << " (default)" << std::endl;
  else
    std::cout << "ecm : range : tracking : " << this->m_ecm.getRange() << std::endl;
}

/*!
  Read the contrast information from the xml file.

  \throw vpException::fatalError if there was an unexpected number of data.

  \param doc : Pointer to the document.
  \param node : Pointer to the node of the contrast information.
*/
void vpMbtXmlParser::read_contrast(xmlDocPtr doc, xmlNodePtr node)
{
  bool edge_threshold_node = false;
  bool mu1_node = false;
  bool mu2_node = false;

  // current data values.
  double d_edge_threshold = this->m_ecm.getThreshold();
  double d_mu1 = this->m_ecm.getMu1();
  double d_mu2 = this->m_ecm.getMu2();

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case edge_threshold: {
          d_edge_threshold = xmlReadDoubleChild(doc, dataNode);
          edge_threshold_node = true;
        } break;
        case mu1: {
          d_mu1 = xmlReadDoubleChild(doc, dataNode);
          mu1_node = true;
        } break;
        case mu2: {
          d_mu2 = xmlReadDoubleChild(doc, dataNode);
          mu2_node = true;
        } break;
        default: {
          //          vpTRACE("unknown tag in read_contrast : %d, %s",
          //          iter_data->second, (iter_data->first).c_str());
        } break;
        }
      }
    }
  }

  this->m_ecm.setMu1(d_mu1);
  this->m_ecm.setMu2(d_mu2);
  this->m_ecm.setThreshold(d_edge_threshold);

  if (!edge_threshold_node)
    std::cout << "ecm : contrast : threshold " << this->m_ecm.getThreshold() << " (default)" << std::endl;
  else
    std::cout << "ecm : contrast : threshold " << this->m_ecm.getThreshold() << std::endl;

  if (!mu1_node)
    std::cout << "ecm : contrast : mu1 " << this->m_ecm.getMu1() << " (default)" << std::endl;
  else
    std::cout << "ecm : contrast : mu1 " << this->m_ecm.getMu1() << std::endl;

  if (!mu2_node)
    std::cout << "ecm : contrast : mu2 " << this->m_ecm.getMu2() << " (default)" << std::endl;
  else
    std::cout << "ecm : contrast : mu2 " << this->m_ecm.getMu2() << std::endl;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(vpMbtXmlParser.cpp.o) has no
// symbols
void dummy_vpMbtXmlParser(){};
#endif
