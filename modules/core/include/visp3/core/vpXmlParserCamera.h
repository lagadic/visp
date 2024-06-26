/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
  \file vpXmlParserCamera.h
  \brief Declaration of the vpXmlParserCamera class.
  Class vpXmlParserCamera allowed to load and save intrinsic camera parameters

*/

#ifndef VP_XML_PARSER_CAMERA_H
#define VP_XML_PARSER_CAMERA_H

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PUGIXML)
#include <visp3/core/vpCameraParameters.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpXmlParserCamera

  \ingroup group_core_camera

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
  #include <visp3/core/vpCameraParameters.h>
  #include <visp3/core/vpXmlParserCamera.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpCameraParameters cam; // Create a camera parameter container
    vpXmlParserCamera p; // Create a XML parser
    vpCameraParameters::vpCameraParametersProjType projModel; // Projection model
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
  }
  \endcode

  Example of writing an XML file containing intrinsic camera parameters:
  \code
  #include <visp3/core/vpCameraParameters.h>
  #include <visp3/core/vpXmlParserCamera.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    // Create a camera parameter container. We want to set these parameters
    // for a 320x240 image, and we want to use the perspective projection
    // modelization without distortion.
    vpCameraParameters cam;

    // Set the principal point coordinates (u0,v0)
    double u0 = 162.3;
    double v0 = 122.4;
    // Set the pixel ratio (px, py)
    double px = 563.2;
    double py = 564.1;

    // Set the camera parameters for a model without distortion
    cam.initPersProjWithoutDistortion(px, py, u0, v0);

    // Create a XML parser
    vpXmlParserCamera p;
    // Save the camera parameters in an XML file.
    if (p.save(cam, "myNewXmlFile.xml", "myNewCamera", 320, 240) != vpXmlParserCamera::SEQUENCE_OK) {
      std::cout << "Cannot save camera parameters" << std::endl;
    }
  }
  \endcode
*/

class VISP_EXPORT vpXmlParserCamera
{
public:
  enum vpXmlCodeSequenceType { SEQUENCE_OK, SEQUENCE_ERROR };

  vpXmlParserCamera();
  ~vpXmlParserCamera();

  // get/set functions
  std::string getCameraName() const;
  vpCameraParameters getCameraParameters() const;
  unsigned int getHeight() const;
  unsigned int getSubsampling_width() const;
  unsigned int getSubsampling_height() const;
  unsigned int getWidth() const;

  int parse(vpCameraParameters &cam, const std::string &filename, const std::string &camera_name,
            const vpCameraParameters::vpCameraParametersProjType &projModel, unsigned int image_width = 0,
            unsigned int image_height = 0, bool verbose = true);

  int save(const vpCameraParameters &cam, const std::string &filename, const std::string &camera_name,
           unsigned int image_width = 0, unsigned int image_height = 0, const std::string &additionalInfo = "",
           bool verbose = true);

  void setCameraName(const std::string &name);
  void setHeight(unsigned int height);
  void setSubsampling_width(unsigned int subsampling);
  void setSubsampling_height(unsigned int subsampling);
  void setWidth(unsigned int width);

private:
  vpXmlParserCamera(const vpXmlParserCamera &c);            // noncopyable
  vpXmlParserCamera &operator=(const vpXmlParserCamera &c); //

  // PIMPL idiom
  class Impl;
  Impl *m_impl;
};
END_VISP_NAMESPACE
#endif
#endif
