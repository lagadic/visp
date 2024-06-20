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
 * Test vpXmlParserRectOriented parse / save.
 */

/*!
  \file testXmlParserRectOriented.cpp

  Test vpXmlParserRectOriented parse / save.
*/

#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpXmlParserRectOriented.h>

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

  tmp_dir += username + "/test_xml_parser_rect_oriented/";
  vpIoTools::remove(tmp_dir);
  std::cout << "Create: " << tmp_dir << std::endl;
  vpIoTools::makeDirectory(tmp_dir);

  vpRectOriented rect_oriented(vpImagePoint(124.2489, 251.4513), 254.34, 413.04, vpMath::rad(41.367));
  std::string filename = tmp_dir + "test_write_rect_oriented.xml";
  {
    vpXmlParserRectOriented xml;
    xml.setRectangle(rect_oriented);
    std::cout << "Write to: " << filename << std::endl;
    xml.save(filename);
  }

  vpRectOriented rect_oriented_read;
  {
    vpXmlParserRectOriented xml;
    xml.parse(filename);
    rect_oriented_read = xml.getRectangle();

    double eps = std::numeric_limits<double>::epsilon();
    if (rect_oriented.getCenter() != rect_oriented_read.getCenter() ||
        !vpMath::equal(rect_oriented.getWidth(), rect_oriented_read.getWidth(), eps) ||
        !vpMath::equal(rect_oriented.getHeight(), rect_oriented_read.getHeight(), eps) ||
        !vpMath::equal(rect_oriented.getOrientation(), rect_oriented_read.getOrientation(), eps)) {
      std::cerr << "Issue when parsing XML file: " << filename << std::endl;
      return EXIT_FAILURE;
    }
  }

  vpIoTools::remove(tmp_dir);
#endif

  return EXIT_SUCCESS;
}
