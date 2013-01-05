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

/*!
 * \file vpMbtXmlParser.h
 * \brief Parse an Xml file to extract configuration parameters of a mbtConfig object.
*/

#ifndef vpMbtXmlParser_HH
#define vpMbtXmlParser_HH

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h>      /* Fonctions de la lib XML.                */

#include <visp/vpXmlParser.h>
#include <visp/vpMe.h>
#include <visp/vpCameraParameters.h>



/*!
  \class vpMbtXmlParser
  \brief Parse an Xml file to extract configuration parameters of a mbtConfig object.
  \ingroup ModelBasedTracking

  Data parser for the model based tracker.

 */
class VISP_EXPORT vpMbtXmlParser: public vpXmlParser
{
protected:
  //! Moving edges parameters.
  vpMe m_ecm;
  //! Camera parameters.
  vpCameraParameters cam;
  //! Angle to determine if a face appeared
  double angleAppear;
  //! Angle to determine if a face disappeared
  double angleDisappear;
    
  typedef enum{
    conf,
    ecm,
    mask,
    size,
    nb_mask,
    range,
    tracking,
    contrast,
    edge_threshold,
    mu1,
    mu2,
    sample,
    step,
    nb_sample,
    face,
    angle_appear,
    angle_disappear,
    camera,
    height,
    width,
    u0,
    v0,
    px,
    py
  } dataToParse;


public:

	vpMbtXmlParser();
	virtual ~vpMbtXmlParser();

  /*!
    Get the angle to determine if a face appeared.

    \return angleAppear
  */
  inline double getAngleAppear() const {return angleAppear;}
  
  /*!
    Get the angle to determine if a face disappeared.

    \return angleDisappear
  */
  inline double getAngleDisappear() const {return angleDisappear;}
  
  void getCameraParameters(vpCameraParameters& _cam) const { _cam = this->cam;}
  void getMe(vpMe& _ecm) const { _ecm = this->m_ecm;}
  
	void parse(const char * filename);

  void readMainClass(xmlDocPtr doc, xmlNodePtr node);
	void read_ecm (xmlDocPtr doc, xmlNodePtr node);
	void read_sample (xmlDocPtr doc, xmlNodePtr node);
	void read_camera (xmlDocPtr doc, xmlNodePtr node);
	void read_mask (xmlDocPtr doc, xmlNodePtr node);
	void read_range (xmlDocPtr doc, xmlNodePtr node);
	void read_contrast (xmlDocPtr doc, xmlNodePtr node);
  void read_face(xmlDocPtr doc, xmlNodePtr node);
  
  /*!
    Set the angle to determine if a face appeared.

    \param aappear : New angleAppear
  */
  inline void setAngleAppear(const double &aappear) {angleAppear = aappear;}
  
  /*!
    Set the angle to determine if a face disappeared.

    \param adisappear : New angleDisappear
  */
  inline void setAngleDisappear(const double &adisappear) {angleDisappear = adisappear;}
  
  void setCameraParameters(const vpCameraParameters &_cam){ cam = _cam; }
  void setMovingEdge(const vpMe &_ecm){ m_ecm = _ecm; }
	
  void writeMainClass(xmlNodePtr node);
	
protected:
  void init();

};

#endif

#endif /* NMBTXMLPARSER_H_ */



