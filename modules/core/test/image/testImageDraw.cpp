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
 * Test for vpImageDraw class.
 */
/*!
 \example testImageDraw.cpp

 \brief Test for vpImageDraw class.
*/

#include <iostream>
#include <visp3/core/vpFont.h>
#include <visp3/core/vpImageDraw.h>
#include <visp3/io/vpImageIo.h>

int main(int argc, char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  bool save = false;
  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--save") {
      save = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0] << " [--save] [--help] [-h]\n" << std::endl;
      return EXIT_SUCCESS;
    }
  }
  std::cout << "Save: " << save << std::endl;

  const std::string visp = "ViSP: Open source Visual Servoing Platform";

  // vpRGBa
  {
    vpImage<vpRGBa> I(480, 640, vpRGBa(255));
    vpImagePoint iP1, iP2;

    iP1.set_i(20);
    iP1.set_j(10);
    iP2.set_i(20);
    iP2.set_j(30);
    vpImageDraw::drawArrow(I, iP1, iP2, vpColor::green, 4, 2, 3);

    iP1.set_i(20);
    iP1.set_j(60);
    vpFont font(32);
    font.drawText(I, "Test...", iP1, vpColor::blue);

    iP1.set_i(60);
    iP1.set_j(60);
    font.drawText(I, "...tse", iP1, vpColor::red);

    iP1.set_i(60);
    iP1.set_j(260);
    font.drawText(I, "&é(-è_çà)", iP1, vpColor::red);

    iP1.set_i(200);
    iP1.set_j(200);
    font.drawText(I, "Test...", iP1, vpColor::white, vpColor::black);

    vpFont font2(20);
    vpImagePoint textSize = font2.getMeasure(visp);
    vpImagePoint textPos = vpImagePoint(24, 620 - textSize.get_j());
    font2.drawText(I, visp, textPos, vpColor::darkGreen);
    vpImageDraw::drawRectangle(I, vpRect(textPos.get_u(), textPos.get_v(), textSize.get_u(), textSize.get_v()),
                               vpColor::darkRed);

    iP1.set_i(80);
    iP1.set_j(220);
    iP2.set_i(80);
    iP2.set_j(480);
    vpImageDraw::drawCircle(I, iP1, 30, vpColor::red, 3);

    iP1.set_i(20);
    iP1.set_j(220);
    vpImageDraw::drawCross(I, iP1, 5, vpColor::blue, 1);

    iP1.set_i(140);
    iP1.set_j(10);
    iP2.set_i(140);
    iP2.set_j(50);
    vpImageDraw::drawDottedLine(I, iP1, iP2, vpColor::blue, 3);

    iP1.set_i(120);
    iP1.set_j(180);
    iP2.set_i(160);
    iP2.set_j(250);
    vpImageDraw::drawDottedLine(I, iP1, iP2, vpColor::blue, 3);

    iP1.set_i(160);
    iP1.set_j(280);
    iP2.set_i(120);
    iP2.set_j(340);
    vpImageDraw::drawDottedLine(I, iP1, iP2, vpColor::blue, 3);

    iP1.set_i(220);
    iP1.set_j(400);
    iP2.set_i(120);
    iP2.set_j(400);
    vpImageDraw::drawDottedLine(I, iP1, iP2, vpColor::cyan, 3);

    iP1.set_i(220);
    iP1.set_j(480);
    iP2.set_i(120);
    iP2.set_j(450);
    vpImageDraw::drawDottedLine(I, iP1, iP2, vpColor::green, 3);

    vpHomogeneousMatrix cMo(vpTranslationVector(0.15, -0.07, 0.37), vpRotationMatrix(vpRxyzVector(0.1, -0.4, 0.41)));
    vpCameraParameters cam(600, 600, 320, 240);
    vpImageDraw::drawFrame(I, cMo, cam, 0.05, vpColor::none, 3);

    iP1.set_i(140);
    iP1.set_j(80);
    iP2.set_i(140);
    iP2.set_j(150);
    vpImageDraw::drawLine(I, iP1, iP2, vpColor::orange, 3);

    iP1.set_i(140);
    iP1.set_j(400);
    vpImageDraw::drawPoint(I, iP1, vpColor::red);

    iP1.set_i(350);
    iP1.set_j(20);
    int w = 60;
    int h = 50;
    vpImageDraw::drawRectangle(I, vpRect(iP1, w, h), vpColor::red, false, 3);

    iP1.set_i(350);
    iP1.set_j(110);
    vpImageDraw::drawRectangle(I, vpRect(iP1, w, h), vpColor::red, true, 3);

    iP1.set_i(350);
    iP1.set_j(200);
    iP2.set_i(400);
    iP2.set_j(260);
    vpImageDraw::drawRectangle(I, vpRect(iP1, iP2), vpColor::orange, false, 3);

    iP1.set_i(350);
    iP1.set_j(290);
    iP2.set_i(400);
    iP2.set_j(350);
    vpRect rectangle(iP1, iP2);
    vpImageDraw::drawRectangle(I, rectangle, vpColor::yellow, false, 3);

    std::vector<vpImagePoint> polygon;
    polygon.push_back(vpImagePoint(250, 500));
    polygon.push_back(vpImagePoint(350, 600));
    polygon.push_back(vpImagePoint(450, 500));
    polygon.push_back(vpImagePoint(350, 400));
    vpImageDraw::drawPolygon(I, polygon, vpColor::green, 3);

    polygon.clear();
    polygon.push_back(vpImagePoint(300, 500));
    polygon.push_back(vpImagePoint(350, 550));
    polygon.push_back(vpImagePoint(400, 500));
    polygon.push_back(vpImagePoint(350, 450));
    vpImageDraw::drawPolygon(I, polygon, vpColor::cyan, 3, false);

    if (save) {
      std::string filename = "canvas_color.png";
      std::cout << "Save " << filename << std::endl;
      vpImageIo::write(I, filename);
    }
  }

  // unsigned char
  {
    vpImage<unsigned char> I(480, 640, 0);
    vpImagePoint iP1, iP2;
    unsigned char color = 255;

    iP1.set_i(20);
    iP1.set_j(10);
    iP2.set_i(20);
    iP2.set_j(30);
    vpImageDraw::drawArrow(I, iP1, iP2, color, 4, 2, 3);

    iP1.set_i(20);
    iP1.set_j(60);
    vpFont font(32);
    font.drawText(I, "Test...", iP1, color);

    iP1.set_i(60);
    iP1.set_j(60);
    font.drawText(I, "...tse", iP1, color);

    iP1.set_i(60);
    iP1.set_j(260);
    font.drawText(I, "&é(-è_çà)", iP1, color);

    iP1.set_i(200);
    iP1.set_j(200);
    font.drawText(I, "Test...", iP1, 0, 255);

    vpFont font2(20);
    vpImagePoint textSize = font2.getMeasure(visp);
    vpImagePoint textPos = vpImagePoint(24, 620 - textSize.get_j());
    font2.drawText(I, visp, textPos, 255);
    vpImageDraw::drawRectangle(I, vpRect(textPos.get_u(), textPos.get_v(), textSize.get_u(), textSize.get_v()), 255);

    iP1.set_i(80);
    iP1.set_j(220);
    iP2.set_i(80);
    iP2.set_j(480);
    vpImageDraw::drawCircle(I, iP1, 30, color, 3);

    iP1.set_i(20);
    iP1.set_j(220);
    vpImageDraw::drawCross(I, iP1, 5, color, 1);

    iP1.set_i(140);
    iP1.set_j(10);
    iP2.set_i(140);
    iP2.set_j(50);
    vpImageDraw::drawDottedLine(I, iP1, iP2, color, 3);

    iP1.set_i(120);
    iP1.set_j(180);
    iP2.set_i(160);
    iP2.set_j(250);
    vpImageDraw::drawDottedLine(I, iP1, iP2, color, 3);

    iP1.set_i(160);
    iP1.set_j(280);
    iP2.set_i(120);
    iP2.set_j(340);
    vpImageDraw::drawDottedLine(I, iP1, iP2, color, 3);

    iP1.set_i(220);
    iP1.set_j(400);
    iP2.set_i(120);
    iP2.set_j(400);
    vpImageDraw::drawDottedLine(I, iP1, iP2, color, 3);

    iP1.set_i(220);
    iP1.set_j(480);
    iP2.set_i(120);
    iP2.set_j(450);
    vpImageDraw::drawDottedLine(I, iP1, iP2, color, 3);

    vpHomogeneousMatrix cMo(vpTranslationVector(0.15, -0.07, 0.37), vpRotationMatrix(vpRxyzVector(0.1, -0.4, 0.41)));
    vpCameraParameters cam(600, 600, 320, 240);
    vpImageDraw::drawFrame(I, cMo, cam, 0.05, color, 3);

    iP1.set_i(140);
    iP1.set_j(80);
    iP2.set_i(140);
    iP2.set_j(150);
    vpImageDraw::drawLine(I, iP1, iP2, color, 3);

    iP1.set_i(140);
    iP1.set_j(400);
    vpImageDraw::drawPoint(I, iP1, color);

    iP1.set_i(350);
    iP1.set_j(20);
    int w = 60;
    int h = 50;
    vpImageDraw::drawRectangle(I, vpRect(iP1, w, h), color, false, 3);

    iP1.set_i(350);
    iP1.set_j(110);
    vpImageDraw::drawRectangle(I, vpRect(iP1, w, h), color, true, 3);

    iP1.set_i(350);
    iP1.set_j(200);
    iP2.set_i(400);
    iP2.set_j(260);
    vpImageDraw::drawRectangle(I, vpRect(iP1, iP2), color, false, 3);

    iP1.set_i(350);
    iP1.set_j(290);
    iP2.set_i(400);
    iP2.set_j(350);
    vpRect rectangle(iP1, iP2);
    vpImageDraw::drawRectangle(I, rectangle, color, false, 3);

    std::vector<vpImagePoint> polygon;
    polygon.push_back(vpImagePoint(250, 500));
    polygon.push_back(vpImagePoint(350, 600));
    polygon.push_back(vpImagePoint(450, 500));
    polygon.push_back(vpImagePoint(350, 400));
    vpImageDraw::drawPolygon(I, polygon, color, 3);

    polygon.clear();
    polygon.push_back(vpImagePoint(300, 500));
    polygon.push_back(vpImagePoint(350, 550));
    polygon.push_back(vpImagePoint(400, 500));
    polygon.push_back(vpImagePoint(350, 450));
    vpImageDraw::drawPolygon(I, polygon, color, 3, false);

    if (save) {
      std::string filename = "canvas_gray.png";
      std::cout << "Save " << filename << std::endl;
      vpImageIo::write(I, filename);
    }
  }

  return EXIT_SUCCESS;
}
