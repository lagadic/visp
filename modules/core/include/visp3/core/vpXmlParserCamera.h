/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_XML2

#include <libxml/xmlmemory.h> /* Functions of libxml.                */
#include <string>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParser.h>

/*!
  \class vpXmlParserCamera

  \ingroup group_core_camera

  \brief XML parser to load and save intrinsic camera parameters.

  \warning This class is only available if libxml2 is installed and detected
by ViSP. Installation instructions are provided here
https://visp.inria.fr/3rd_xml2.

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
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>

int main()
{
  vpCameraParameters cam; // Create a camera parameter container

#ifdef VISP_HAVE_XML2
  vpXmlParserCamera p; // Create a XML parser
  vpCameraParameters::vpCameraParametersProjType projModel; // Projection
model
  // Use a perspective projection model without distortion
  projModel = vpCameraParameters::perspectiveProjWithoutDistortion;
  // Parse the xml file "myXmlFile.xml" to find the intrinsic camera
  // parameters of the camera named "myCamera" for the image sizes 640x480,
  // for the projection model projModel. The size of the image is optional
  // if camera parameters are given only for one image size.
  if (p.parse(cam, "myXmlFile.xml", "myCamera", projModel,640,480) != vpXmlParserCamera::SEQUENCE_OK) {
   std::cout << "Cannot found myCamera" << std::endl;
  }

  // cout the parameters
  cam.printParameters();

  // Get the camera parameters for the model without distortion
  double px = cam.get_px();
  double py = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();

  // Now we modify the principal point (u0,v0) for example to add noise
  u0 *= 0.9;
  v0 *= 0.8;

  // Set the new camera parameters
  cam.initPersProjWithoutDistortion(px, py, u0, v0);

  // Save the parameters in a new file "myXmlFileWithNoise.xml"
  p.save(cam,"myXmlFileWithNoise.xml",p.getCameraName(),p.getWidth(),p.getHeight());

  // Clean up memory allocated by the xml library
  vpXmlParser::cleanup();
#endif
}
  \endcode

  Example of writing an XML file containing intrinsic camera parameters:
  \code
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpXmlParserCamera.h>

int main()
{
  // Create a camera parameter container. We want to set these parameters
  // for a 320x240 image, and we want to use the perspective projection
  // modelisation without distortion.
  vpCameraParameters cam;

  // Set the principal point coordinates (u0,v0)
  double u0 = 162.3;
  double v0 = 122.4;
  // Set the pixel ratio (px, py)
  double px = 563.2;
  double py = 564.1;

  // Set the camera parameters for a model without distortion
  cam.initPersProjWithoutDistortion(px, py, u0, v0);

#ifdef VISP_HAVE_XML2
  // Create a XML parser
  vpXmlParserCamera p;
  // Save the camera parameters in an XML file.
  if (p.save(cam, "myNewXmlFile.xml", "myNewCamera", 320, 240) != vpXmlParserCamera::SEQUENCE_OK) {
    std::cout << "Cannot save camera parameters" << std::endl;
  }

  // Clean up memory allocated by the xml library
  vpXmlParser::cleanup();
#endif
}
  \endcode
*/

class VISP_EXPORT vpXmlParserCamera : public vpXmlParser
{

public:
  /* --- XML Code------------------------------------------------------------
   */
  typedef enum {
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
    CODE_XML_KDU,
    CODE_XML_ADDITIONAL_INFO
  } vpXmlCodeType;

  typedef enum { SEQUENCE_OK, SEQUENCE_ERROR } vpXmlCodeSequenceType;

private:
  vpCameraParameters camera;
  std::string camera_name;
  unsigned int image_width;
  unsigned int image_height;
  unsigned int subsampling_width;
  unsigned int subsampling_height;
  unsigned int full_width;
  unsigned int full_height;

  //! Allowed size difference between input image and data from the xml parser
  //! to handle minor differences (ex. FORMAT7 can creates 648*488 images).
  static const int allowedPixelDiffOnImageSize = 15;

public:
  vpXmlParserCamera();
  vpXmlParserCamera(vpXmlParserCamera &twinParser);
  //! Default destructor.
  virtual ~vpXmlParserCamera() {}

  // get/set functions
  std::string getCameraName() { return this->camera_name; }
  vpCameraParameters getCameraParameters() { return this->camera; }
  unsigned int getHeight() { return this->image_height; }
  unsigned int getSubsampling_width() { return this->subsampling_width; }
  unsigned int getSubsampling_height() { return this->subsampling_height; }
  unsigned int getWidth() { return this->image_width; }

  vpXmlParserCamera &operator=(const vpXmlParserCamera &twinparser);

  int parse(vpCameraParameters &cam, const std::string &filename, const std::string &camera_name,
            const vpCameraParameters::vpCameraParametersProjType &projModel, const unsigned int image_width = 0,
            const unsigned int image_height = 0);

  int save(const vpCameraParameters &cam, const std::string &filename, const std::string &camera_name,
           const unsigned int image_width = 0, const unsigned int image_height = 0,
           const std::string &additionalInfo = "");

  void setCameraName(const std::string &name) { this->camera_name = name; }
  void setHeight(const unsigned int height) { this->image_height = height; }
  void setSubsampling_width(const unsigned int subsampling) { this->subsampling_width = subsampling; }
  void setSubsampling_height(const unsigned int subsampling) { this->subsampling_height = subsampling; }
  void setWidth(const unsigned int width) { this->image_width = width; }

private:
  int read(xmlDocPtr doc, xmlNodePtr node, const std::string &camera_name,
           const vpCameraParameters::vpCameraParametersProjType &projModel, const unsigned int image_width = 0,
           const unsigned int image_height = 0, const unsigned int subsampling_width = 0,
           const unsigned int subsampling_height = 0);

  int count(xmlDocPtr doc, xmlNodePtr node, const std::string &camera_name,
            const vpCameraParameters::vpCameraParametersProjType &projModel, const unsigned int image_width = 0,
            const unsigned int image_height = 0, const unsigned int subsampling_width = 0,
            const unsigned int subsampling_height = 0);

  int read_camera(xmlDocPtr doc, xmlNodePtr node, const std::string &camera_name,
                  const vpCameraParameters::vpCameraParametersProjType &projModel, const unsigned int image_width = 0,
                  const unsigned int image_height = 0, const unsigned int subsampling_width = 0,
                  const unsigned int subsampling_height = 0);

  xmlNodePtr find_camera(xmlDocPtr doc, xmlNodePtr node, const std::string &camera_name,
                         const unsigned int image_width = 0, const unsigned int image_height = 0,
                         const unsigned int subsampling_width = 0, const unsigned int subsampling_height = 0);

  xmlNodePtr find_additional_info(xmlNodePtr node);

  vpXmlCodeSequenceType read_camera_model(xmlDocPtr doc, xmlNodePtr node, vpCameraParameters &camera);

  int read_camera_header(xmlDocPtr doc, xmlNodePtr node, const std::string &camera_name,
                         const unsigned int image_width = 0, const unsigned int image_height = 0,
                         const unsigned int subsampling_width = 0, const unsigned int subsampling_height = 0);

  static vpXmlCodeSequenceType str2xmlcode(char *str, vpXmlCodeType &res);
  void myXmlReadIntChild(xmlDocPtr doc, xmlNodePtr node, int &res, vpXmlCodeSequenceType &code_error);

  void myXmlReadDoubleChild(xmlDocPtr doc, xmlNodePtr node, double &res, vpXmlCodeSequenceType &code_error);

  void myXmlReadCharChild(xmlDocPtr doc, xmlNodePtr node, char **res);
  int write(xmlNodePtr node, const std::string &camera_name, const unsigned int image_width = 0,
            const unsigned int image_height = 0, const unsigned int subsampling_width = 0,
            const unsigned int subsampling_height = 0);
  int write_camera(xmlNodePtr node_camera);

private:
  /*!

    \param 2doc : a pointer representing the document
    \param node : the root node of the document
  */
  virtual void readMainClass(xmlDocPtr, xmlNodePtr){};

  /*!


    \param node2 : the root node of the document
  */
  virtual void writeMainClass(xmlNodePtr){};
};
#endif // VISP_HAVE_XML2
#endif
