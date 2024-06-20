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
 * Test vpXmlParserCamera parse / save.
 */

/*!
  \example testXmlParserCamera.cpp

  Test vpXmlParserCamera parse / save.
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserCamera.h>

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if defined(VISP_HAVE_PUGIXML)

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
    std::cout << "-- Test to save/load one single camera without distortion in a single file" << std::endl;
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

    {
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera", vpCameraParameters::perspectiveProjWithoutDistortion, 320, 240, false);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      // Without specifying image size
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera", vpCameraParameters::perspectiveProjWithoutDistortion, 0, 0, false);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "-- Test to save/load one single camera with distortion in a single file" << std::endl;
    std::string filename = tmp_dir + "test_write_cam_with_distortion.xml";
    vpCameraParameters cam;
    cam.initPersProjWithDistortion(200, 200, 160, 120, 0.02, -0.02);
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << std::endl;
      if (xml.save(cam, filename, "Camera", 320, 240) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera", vpCameraParameters::perspectiveProjWithDistortion, 320, 240, false);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      // Without specifying image size
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera", vpCameraParameters::perspectiveProjWithDistortion, 0, 0, false);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "-- Test to save/load multiple cameras with and without distortion in a single file" << std::endl;
    std::string filename = tmp_dir + "test_write_cam_multiple.xml";
    vpCameraParameters cam1_w_d;
    cam1_w_d.initPersProjWithDistortion(200, 200, 160, 120, 0.02, -0.02);
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << " camera:\n" << cam1_w_d << std::endl;
      if (xml.save(cam1_w_d, filename, "Camera 1", 320, 240) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam1_wo_d;
    cam1_wo_d.initPersProjWithoutDistortion(200, 200, 160, 120);
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << " camera:\n" << cam1_wo_d << std::endl;
      if (xml.save(cam1_wo_d, filename, "Camera 1", 320, 240) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
    vpCameraParameters cam2_w_d;
    cam2_w_d.initPersProjWithDistortion(400, 400, 320, 240, 0.02, -0.02);
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << " camera:\n" << cam2_w_d << std::endl;
      if (xml.save(cam2_w_d, filename, "Camera 2", 640, 480) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam2_wo_d;
    cam2_wo_d.initPersProjWithoutDistortion(400, 400, 320, 240);
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << " camera:\n" << cam2_wo_d << std::endl;
      if (xml.save(cam2_wo_d, filename, "Camera 2", 640, 480) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera 1", vpCameraParameters::perspectiveProjWithDistortion, 320, 240, false);
      std::cout << "Cam write:\n" << cam1_w_d << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam1_w_d != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
    {
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera 1", vpCameraParameters::perspectiveProjWithoutDistortion, 320, 240, false);
      std::cout << "Cam write:\n" << cam1_wo_d << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam1_wo_d != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
    {
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera 2", vpCameraParameters::perspectiveProjWithDistortion, 640, 480, false);
      std::cout << "Cam write:\n" << cam2_w_d << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam2_w_d != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
    {
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera 2", vpCameraParameters::perspectiveProjWithoutDistortion, 640, 480, false);
      std::cout << "Cam write:\n" << cam2_wo_d << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam2_wo_d != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "-- Test to save/load one single camera with Kannala Brandt distortion in a single file" << std::endl;
    vpCameraParameters cam;
    std::vector<double> distortion_coeffs;
    distortion_coeffs.push_back(-0.00297341705299914);
    distortion_coeffs.push_back(0.0352853797376156);
    distortion_coeffs.push_back(-0.032205019146204);
    distortion_coeffs.push_back(0.004446716979146);
    distortion_coeffs.push_back(0);
    cam.initProjWithKannalaBrandtDistortion(285.523895263672, 286.6708984375, 420.874114990234, 381.085388183594,
                                            distortion_coeffs);
    std::string filename = tmp_dir + "test_write_cam_with_KannalaBrandt_distortion.xml";
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << std::endl;
      if (xml.save(cam, filename, "Camera", 800, 848) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam_read;
    {
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "Camera", vpCameraParameters::ProjWithKannalaBrandtDistortion, 800, 848, false);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "-- Test to save/load one single camera with Kannala Brandt distortion in a single file wo name" << std::endl;
    vpCameraParameters cam;
    std::vector<double> distortion_coeffs;
    distortion_coeffs.push_back(-0.00297341705299914);
    distortion_coeffs.push_back(0.0352853797376156);
    distortion_coeffs.push_back(-0.032205019146204);
    distortion_coeffs.push_back(0.004446716979146);
    distortion_coeffs.push_back(0);
    cam.initProjWithKannalaBrandtDistortion(285.523895263672, 286.6708984375, 420.874114990234, 381.085388183594,
                                            distortion_coeffs);
    std::string filename = tmp_dir + "test_write_cam_with_KannalaBrandt_distortion_wo_name.xml";
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << std::endl;
      if (xml.save(cam, filename, "Camera", 800, 848) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam_read;
    {
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "", vpCameraParameters::ProjWithKannalaBrandtDistortion, 800, 848, false);
      std::cout << "Cam write:\n" << cam << std::endl;
      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  {
    std::cout << "-- Test to save 2 cameras and parse them wo name thanks they differ in distortion" << std::endl;
    vpCameraParameters cam1;

    std::string filename = tmp_dir + "test_write_2_cam_differ_in_distortion.xml";

    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << std::endl;
      std::cout << "Cam write:\n" << cam1 << std::endl;
      if (xml.save(cam1, filename, "Camera 1", 320, 240, "", false) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    vpCameraParameters cam2;
    std::vector<double> distortion_coeffs;
    distortion_coeffs.push_back(-0.00297341705299914);
    distortion_coeffs.push_back(0.0352853797376156);
    distortion_coeffs.push_back(-0.032205019146204);
    distortion_coeffs.push_back(0.004446716979146);
    distortion_coeffs.push_back(0);
    cam2.initProjWithKannalaBrandtDistortion(285.523895263672, 286.6708984375, 420.874114990234, 381.085388183594,
                                            distortion_coeffs);
    {
      vpXmlParserCamera xml;
      std::cout << "Write to: " << filename << std::endl;
      std::cout << "Cam write:\n" << cam2 << std::endl;
      if (xml.save(cam2, filename, "Camera 2", 800, 848, "", false) != vpXmlParserCamera::SEQUENCE_OK) {
        std::cerr << "Cannot save XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::cout << "Attempt to read camera with perspective projection without distortion and without name" << std::endl;
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "", vpCameraParameters::perspectiveProjWithoutDistortion, 320, 240, false);

      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam1 != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }

    {
      std::cout << "Attempt to read camera with Kannala Brandt distortion and without name" << std::endl;
      vpCameraParameters cam_read;
      vpXmlParserCamera xml;
      xml.parse(cam_read, filename, "", vpCameraParameters::ProjWithKannalaBrandtDistortion, 800, 848, false);

      std::cout << "Cam read:\n" << cam_read << std::endl;
      if (cam2 != cam_read) {
        std::cerr << "Issue when parsing XML file: " << filename << std::endl;
        return EXIT_FAILURE;
      }
    }
  }

  vpIoTools::remove(tmp_dir);

  std::cout << "Test succeed" << std::endl;
#endif

  return EXIT_SUCCESS;
}
