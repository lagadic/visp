/****************************************************************************
 *
 * $Id: vpXmlParserCamera.h,v 1.6 2008-01-31 14:43:50 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
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
 * XML parser to load and save camera intrinsic parameters.
 *
 * Authors:
 * Anthony Saunier
 *
 *****************************************************************************/



/*!
  \file vpXmlParserCamera.h
  \brief Declaration of the vpXmlParserCamera class.
  Class vpXmlParserCamera allowed to load and save intrinsic camera parameters

*/


#ifndef vpXMLPARSERCAMERA_H
#define vpXMLPARSERCAMERA_H

#include <visp/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <string>
#include <visp/vpCameraParameters.h>
#include <libxml/xmlmemory.h>      /* Functions of libxml.                */

/*!
  \class vpXmlParserCamera
  \brief XML Parser to load and save intrinsic camera parameters.

  Example of loading existing camera parameters :
  \code

    vpXmlParserCamera p;
    vpCameraParameters cam;

    // Parse the xml file "myXmlFile.xml" to find the intrinsic camera parameters
    // of the camera named "myCamera" for the image sizes 320x240 for the
    // projection model projModel.
    // The size of the image are optional.

    p.parse(cam,"myXmlFile.xml","myCamera",projModel,320,240);

    cam.printParameters();
    // Work on camera parameters
    cam.setPrincipalPoint(162.321,122.456);

    // Save the new camera parameters in another file.
    p.save(cam,"myNewXmlFile.xml",p.getCameraName(),p.getWidth(),p.getHeight());
    // Or in the same file with another camera name.
    p.save(cam,"myXmlFile.xml","myNewCamera",width,height)

  \endcode
*/

class VISP_EXPORT vpXmlParserCamera
{

public:

  /* --- XML Code------------------------------------------------------------ */
  typedef enum 
    {
      CODE_XML_BAD = -1,
      CODE_XML_OTHER,
      CODE_XML_CAMERA,
      CODE_XML_CAMERA_NAME,
      CODE_XML_HEIGHT,
      CODE_XML_WIDTH,
      CODE_XML_SUBSAMPLING_WIDTH,
      CODE_XML_SUBSAMPLING_HEIGHT,
      CODE_XML_FULL_HEIGHT,
      CODE_XML_FULL_WIDTH,
      CODE_XML_MODEL,
      CODE_XML_MODEL_TYPE,
      CODE_XML_U0,
      CODE_XML_V0,
      CODE_XML_PX,
      CODE_XML_PY,
      CODE_XML_KUD,
      CODE_XML_KDU
    } vpXmlCodeType;

  typedef enum 
    {
      SEQUENCE_OK    ,
      SEQUENCE_ERROR
    } vpXmlCodeSequenceType;

private :

  vpCameraParameters camera;
  std::string camera_name;
  unsigned int image_width;
  unsigned int image_height;
  unsigned int subsampling_width;
  unsigned int subsampling_height;
  unsigned int full_width;
  unsigned int full_height;


public:

  vpXmlParserCamera();
  vpXmlParserCamera(vpXmlParserCamera& twinParser);
  vpXmlParserCamera& operator =(const vpXmlParserCamera& twinparser);
  ~vpXmlParserCamera(){}

  int parse(vpCameraParameters &cam, const char * filename,
	    const std::string &camera_name,
      const vpCameraParameters::vpCameraParametersProjType &projModel,  
	    const unsigned int image_width = 0, const unsigned int image_height = 0);
  int save(const vpCameraParameters &cam, const char * filename,
	   const std::string &camera_name,
	   const unsigned int image_width = 0, const unsigned int image_height = 0);

  // get/set functions
  std::string getCameraName(){return this->camera_name;}
  unsigned int getWidth(){ return this->image_width; }
  unsigned int getHeight(){ return this->image_height; }
  unsigned int getSubsampling_width(){return this->subsampling_width;}
  unsigned int getSubsampling_height(){return this->subsampling_height;}
  vpCameraParameters getCameraParameters(){return this->camera;}

  void setCameraName(const std::string& camera_name){
    this->camera_name = camera_name;
  }
  void setWidth(const unsigned int width){ this->image_width = width ; }
  void setHeight(const unsigned int height){ this->image_height = height ; }
  void setSubsampling_width(const unsigned int subsampling_width){
    this->subsampling_width = subsampling_width ;
  }
  void setSubsampling_height(const unsigned int subsampling_height){
    this->subsampling_height = subsampling_height ;
  }

private:
  int read (xmlDocPtr doc, xmlNodePtr node,
	    const std::string& camera_name,
      const vpCameraParameters::vpCameraParametersProjType &projModel,
      const unsigned int image_width  = 0,
	    const unsigned int image_height = 0,
	    const unsigned int subsampling_width = 0,
	    const unsigned int subsampling_height = 0);

  int count (xmlDocPtr doc, xmlNodePtr node,
	     const std::string& camera_name,
       const vpCameraParameters::vpCameraParametersProjType &projModel,
       const unsigned int image_width  = 0,
	     const unsigned int image_height = 0,
	     const unsigned int subsampling_width = 0,
	     const unsigned int subsampling_height = 0);

  int read_camera (xmlDocPtr doc, xmlNodePtr node,
		   const std::string& camera_name,
       const vpCameraParameters::vpCameraParametersProjType &projModel,
       const unsigned int image_width  = 0,
		   const unsigned int image_height = 0,
		   const unsigned int subsampling_width = 0,
		   const unsigned int subsampling_height = 0);
  
  xmlNodePtr find_camera (xmlDocPtr doc, xmlNodePtr node,
                   const std::string& camera_name,
                   const unsigned int image_width  = 0,
                   const unsigned int image_height = 0,
                   const unsigned int subsampling_width = 0,
                   const unsigned int subsampling_height = 0);
 
  vpXmlCodeSequenceType read_camera_model (xmlDocPtr doc, xmlNodePtr node,
					                                 vpCameraParameters &camera);
  
  int read_camera_header (xmlDocPtr doc, xmlNodePtr node,
                          const std::string& camera_name,
                          const unsigned int image_width = 0,
                          const unsigned int image_height = 0,
                          const unsigned int subsampling_width = 0,
                          const unsigned int subsampling_height = 0);
   
  static vpXmlCodeSequenceType str2xmlcode (char * str, vpXmlCodeType & res);
  void myXmlReadIntChild (xmlDocPtr doc,
			  xmlNodePtr node,
			  int &res,
			  vpXmlCodeSequenceType &code_error);

  void myXmlReadDoubleChild (xmlDocPtr doc,
			     xmlNodePtr node,
			     double &res,
			     vpXmlCodeSequenceType &code_error);

  void myXmlReadCharChild (xmlDocPtr doc,
			   xmlNodePtr node,
			   char **res);
  int write (xmlNodePtr node, const std::string& camera_name,
	     const unsigned int image_width  = 0,
	     const unsigned int image_height = 0,
	     const unsigned int subsampling_width = 0,
	     const unsigned int subsampling_height = 0);
  int write_camera(xmlNodePtr node_camera);
};
#endif //VISP_HAVE_XML2
#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */

