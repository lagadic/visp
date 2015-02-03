/****************************************************************************
 *
 * $Id: testIoTools.cpp 5210 2015-01-26 10:51:11Z strinh $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Test functions in vpIoTools.
 *
 * Authors:
 * Souriya Trinh
 *
 *****************************************************************************/

/*!

  \example testConvert.cpp

  \brief Test functions in Convert.

*/

#include <iostream>     // std::cout
#include <limits>       // std::numeric_limits
#include <visp/vpConfig.h>
#include <visp/vpConvert.h>


bool areSame(double a, double b) {
    return fabs(a - b) < std::numeric_limits<double>::epsilon();
}

void testConvertFromImagePointToPoint2d() {
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  vpImagePoint imPt1(12.5, .85);
  vpImagePoint imPt2(-44.26, 125.11);
  vpImagePoint imPt3(0, -1.756e-10);

  cv::Point2d pt1, pt2, pt3;
  vpConvert::convertToOpenCV(imPt1, pt1);
  vpConvert::convertToOpenCV(imPt2, pt2);
  vpConvert::convertToOpenCV(imPt3, pt3);

  int nbOk = 0, nbNOk = 0;
  if(areSame(imPt1.get_u(), pt1.x) && areSame(imPt1.get_v(), pt1.y)) nbOk++; else nbNOk++;
  if(areSame(imPt2.get_u(), pt2.x) && areSame(imPt2.get_v(), pt2.y)) nbOk++; else nbNOk++;
  if(areSame(imPt3.get_u(), pt3.x) && areSame(imPt3.get_v(), pt3.y)) nbOk++; else nbNOk++;

  std::vector<vpImagePoint> listOfImPts(3);
  listOfImPts[0] = imPt1;
  listOfImPts[1] = imPt2;
  listOfImPts[2] = imPt3;

  std::vector<cv::Point2d> listOfPts;
  vpConvert::convertToOpenCV(listOfImPts, listOfPts);

  if(listOfImPts.size() == listOfPts.size()) {
    for(size_t i = 0; i < 3; i++) {
      if(areSame(listOfImPts[i].get_u(), listOfPts[i].x) && areSame(listOfImPts[i].get_v(), listOfPts[i].y)) nbOk++; else nbNOk++;
    }
  } else {
    nbNOk += 3;
  }

  std::cout << "testConvertFromImagePointToPoint2f=" << nbOk << "/" << (nbOk + nbNOk) << std::endl;
#endif
}

int main() {
  testConvertFromImagePointToPoint2d();

	return 0;
}
