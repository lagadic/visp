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
 * Test vpXmlParserCamera parse / save.
 *
 *****************************************************************************/

/*!
  \file testXmlParserCamera.cpp

  Test vpXmlParserCamera parse / save.
*/

#include <visp3/core/vpXmlParserCamera.h>
#include <visp3/core/vpIoTools.h>

int main()
{
#ifdef VISP_HAVE_PUGIXML
#if defined(_WIN32)
  std::string tmp_dir = "C:/temp/";
#else
  std::string tmp_dir = "/tmp/";
#endif

  // Get the user login name
  std::string username;
  vpIoTools::getUserName(username);

  tmp_dir += username + "/test_xml_parser_camera/";
  vpIoTools::remove(tmp_dir);
  std::cout << "Create: " << tmp_dir << std::endl;
  vpIoTools::makeDirectory(tmp_dir);

  {
    vpCameraParameters cam;
    cam.initPersProjWithoutDistortion(278.4691184118, 273.9196496040, 162.0747539621, 113.1741829586);
    std::string filename = tmp_dir + "test_write_cam_without_distortion.xml";
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << std::endl;
      if (xml.save(cam, filename, "Camera", 320, 240) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam_read;
    {
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera", vpCameraParameters::perspectiveProjWithoutDistortion, 320, 240);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << std::endl;
    vpCameraParameters cam;
    cam.initPersProjWithDistortion(276.2969237503, 271.9362132652, 162.3242102636, 113.4435399636, 0.0272549570, -0.0270531436);
    std::string filename = tmp_dir + "test_write_cam_with_distortion.xml";
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << std::endl;
      if (xml.save(cam, filename, "Camera", 320, 240) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam_read;
    {
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera", vpCameraParameters::perspectiveProjWithDistortion, 320, 240);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  vpIoTools::remove(tmp_dir);
#endif

  return EXIT_SUCCESS;
}
