/****************************************************************************
 *
 * $Id: vpXmlParserCamera.h,v 1.10 2008-11-13 11:13:57 fspindle Exp $
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

  \ingroup CameraModelTransformation CameraModel

  \brief XML parser to load and save intrinsic camera parameters.

  To have a complete description of the camera parameters and the
  corresponding projection model implemented in ViSP, see
  vpCameraParameters.

  Example of an XML file "myXmlFile.xml" containing intrinsic camera
  parameters:

  \code
  <?xml version="1.0"?>
  <root>
    <camera>
      <name>myCamera</name>
      <image_width>640</image_width>
      <image_height>480</image_height>
      <model>
        <type>perspectiveProjWithoutDistortion</type>
        <px>1129.0</px>
        <py>1130.6</py>
        <u0>317.9</u0>
        <v0>229.1</v0>
      </model>
      <model>
        <type>perspectiveProjWithDistortion</type>
        <px>1089.9</px>
        <py>1090.1</py>
        <u0>326.1</u0>
        <v0>230.5</v0>
        <kud>-0.196</kud>
        <kdu>0.204</kdu>
      </model>
    </camera>
  </root>
  \endcode

  Example of loading existing camera parameters from an XML file:
  \code
#include <visp/vpCameraParameters.h>
#include <visp/vpXmlParserCamera.h>

int main()
{
  vpCameraParameters cam; // Create a camera parameter container

#ifdef VISP_HAVE_XML2
  vpXmlParserCamera p; // Create a XML parser
  vpCameraParameters::vpCameraParametersProjType projModel; // Projection model
  // Use a perspective projection model without distorsion
  projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
  // Parse the xml file "myXmlFile.xml" to find the intrinsic camera 
  // parameters of the camera named "myCamera" for the image sizes 640x480,
  // for the projection model projModel. The size of the image is optional
  // if camera parameters are given only for one image size.
  p.parse(cam,"myXmlFile.xml","myCamera",projModel,640,480);

  // cout the parameters
  cam.printParameters();

  // Now we modify the principal point (u0,v0) for example to add noise
  cam.setPrincipalPoint(cam.get_u0()*0.9, cam.get_v0()*0.9);

  // Save the parameters in a new file "myXmlFileWithNoise.xml"
  p.save(cam,"myXmlFileWithNoise.xml",p.getCameraName(),p.getWidth(),p.getHeight());
#endif
}
  \endcode

  Example of writing an XML file containing intrinsic camera parameters:
  \code
#include <visp/vpCameraParameters.h>
#include <visp/vpXmlParserCamera.h>

int main()
{
  // Create a camera parameter container. We want to set these parameters 
  // for a 320x240 image, and we want to use the perspective projection 
  // modelisation without distortion.
  vpCameraParameters cam; 

  // Set the principal point coordinates (u0,v0)
  cam.setPrincipalPoint(162.3,122.4);
  // Set the pixel ratio (px, py)
  cam.setPixelRatio (563.2, 564.1);

#ifdef VISP_HAVE_XML2
  // Create a XML parser
  vpXmlParserCamera p;
  // Save the camera parameters in an XML file.
  p.save(cam,"myNewXmlFile.xml","myNewCamera",320,240);
#endif
}
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

