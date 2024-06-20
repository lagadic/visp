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
 * Test vpXmlParserHomogeneousMatrix parse / save.
 */

/*!
  \file testXmlParserHomogeneousMatrix.cpp

  Test vpXmlParserHomogeneousMatrix parse / save.
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserHomogeneousMatrix.h>

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

  tmp_dir += username + "/test_xml_parser_homogeneous/";
  vpIoTools::remove(tmp_dir);
  std::cout << "Create: " << tmp_dir << std::endl;
  vpIoTools::makeDirectory(tmp_dir);

  vpTranslationVector t(0.264, -0.441, 1.284);
  vpThetaUVector tu(vpMath::rad(12.7), vpMath::rad(-38.23), vpMath::rad(24.45));
  vpHomogeneousMatrix cMo(t, tu);
  std::string filename = tmp_dir + "test_write_homogeneous.xml";
  {
    vpXmlParserHomogeneousMatrix xml;
    std::cout << "Write to: " << filename << std::endl;
    if (xml.save(cMo, filename, "cMo") != vpXmlParserHomogeneousMatrix::SEQUENCE_OK) {
      std::cerr << "Cannot save XML file: " << filename << std::endl;
      return EXIT_FAILURE;
    }
  }

  vpHomogeneousMatrix cMo_read;
  {
    vpXmlParserHomogeneousMatrix xml;
    xml.parse(cMo_read, filename, "cMo");
    std::cout << "cMo write:\n" << cMo << std::endl;
    std::cout << "cMo read:\n" << cMo_read << std::endl;
    for (unsigned int i = 0; i < 3; i++) {
      for (unsigned int j = 0; j < 3; j++) {
        if (!vpMath::equal(cMo[i][j], cMo_read[i][j], std::numeric_limits<double>::epsilon())) {
          std::cerr << "Issue when parsing XML file: " << filename << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
  }

  vpIoTools::remove(tmp_dir);
#endif

  return EXIT_SUCCESS;
}
