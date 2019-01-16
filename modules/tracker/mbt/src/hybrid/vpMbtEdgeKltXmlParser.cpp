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
 * Load XML parameters of the Model based tracker (using edges and point
 *features).
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/
#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <iostream>
#include <map>

#include <libxml/xmlmemory.h> /* Fonctions de la lib XML.                */

#include <visp3/mbt/vpMbtEdgeKltXmlParser.h>

/*!
  Default constructor.

*/
vpMbtEdgeKltXmlParser::vpMbtEdgeKltXmlParser() { init(); }

/*!
  Default destructor.
*/
vpMbtEdgeKltXmlParser::~vpMbtEdgeKltXmlParser() {}

/*!
  Initialise internal variables (including the map).
*/
void vpMbtEdgeKltXmlParser::init()
{
  vpMbtXmlParser::init();
  vpMbtKltXmlParser::init();

  nodeMap["camera"] = vpMbtEdgeKltXmlParser::camera;
  nodeMap["face"] = vpMbtEdgeKltXmlParser::face;
  nodeMap["klt"] = vpMbtEdgeKltXmlParser::klt;
  nodeMap["ecm"] = vpMbtEdgeKltXmlParser::ecm;
  nodeMap["lod"] = vpMbtEdgeKltXmlParser::lod;
}

/*!
  Write info to file.

  \warning Useless, so not yet implemented => Throw exception.
*/
void vpMbtEdgeKltXmlParser::writeMainClass(xmlNodePtr /*node*/)
{
  throw vpException(vpException::notImplementedError, "Not yet implemented.");
}

/*!
  Read the parameters of the class from the file given by its document pointer
  and by its root node.

  \param doc : Document to parse.
  \param node : Root node.
*/
void vpMbtEdgeKltXmlParser::readMainClass(xmlDocPtr doc, xmlNodePtr node)
{
  bool camera_node = false;
  bool face_node = false;
  bool ecm_node = false;
  bool klt_node = false;
  bool lod_node = false;

  for (xmlNodePtr dataNode = node->xmlChildrenNode; dataNode != NULL; dataNode = dataNode->next) {
    if (dataNode->type == XML_ELEMENT_NODE) {
      std::map<std::string, int>::iterator iter_data = this->nodeMap.find((char *)dataNode->name);
      if (iter_data != nodeMap.end()) {
        switch (iter_data->second) {
        case vpMbtEdgeKltXmlParser::camera: {
          this->read_camera(doc, dataNode);
          camera_node = true;
        } break;
        case vpMbtEdgeKltXmlParser::face: {
          this->read_face(doc, dataNode);
          face_node = true;
        } break;
        case vpMbtEdgeKltXmlParser::klt: {
          this->read_klt(doc, dataNode);
          klt_node = true;
        } break;
        case vpMbtEdgeKltXmlParser::ecm: {
          this->read_ecm(doc, dataNode);
          ecm_node = true;
        } break;
        case sample: {
          this->read_sample_deprecated(doc, dataNode);
        } break;
        case vpMbtEdgeKltXmlParser::lod: {
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

  if (!klt_node) {
    std::cout << "klt : Mask Border : " << maskBorder << " (default)" << std::endl;
    std::cout << "klt : Max Features : " << maxFeatures << " (default)" << std::endl;
    std::cout << "klt : Windows Size : " << winSize << " (default)" << std::endl;
    std::cout << "klt : Quality : " << qualityValue << " (default)" << std::endl;
    std::cout << "klt : Min Distance : " << minDist << " (default)" << std::endl;
    std::cout << "klt : Harris Parameter : " << harrisParam << " (default)" << std::endl;
    std::cout << "klt : Block Size : " << blockSize << " (default)" << std::endl;
    std::cout << "klt : Pyramid Levels : " << pyramidLevels << " (default)" << std::endl;
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

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_mbt.a(vpMbtEdgeKltXmlParser.cpp.o)
// has no symbols
void dummy_vpMbtEdgeKltXmlParser(){};
#endif
