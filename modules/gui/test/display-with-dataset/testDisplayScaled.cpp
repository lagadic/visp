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
 * Display testing.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <sstream>

#include <visp3/core/vpImageTools.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpImageIo.h>

template <typename Type> bool test(const std::string &display, vpImage<Type> &I, unsigned int scale, bool click)
{
  bool success = true;
  unsigned int radius(I.getHeight() / 4);
  int scale_ = (int)scale;
  int radius_ = (int)radius;
  unsigned int thickness = 2;
  vpImagePoint center(I.getHeight() / 2, I.getWidth() / 2);
  vpImagePoint offset(30, 160);
  vpImagePoint v_offset(radius, 0);
  vpImagePoint h_offset(0, radius);
  vpRect roi(center, radius_ + scale_, radius_);
  std::string itype;

  // backup the input image
  vpImage<Type> Ibackup(I);

  vpDisplay *d = NULL;
  if (display == "GDI") {
#ifdef VISP_HAVE_GDI
    d = new vpDisplayGDI;
#endif
  } else if (display == "GTK") {
#ifdef VISP_HAVE_GTK
    d = new vpDisplayGTK;
#endif
  } else if (display == "X") {
#ifdef VISP_HAVE_X11
    d = new vpDisplayX;
#endif
  } else if (display == "OpenCV") {
#ifdef VISP_HAVE_OPENCV
    d = new vpDisplayOpenCV;
#endif
  } else if (display == "D3D9") {
#ifdef VISP_HAVE_D3D9
    d = new vpDisplayD3D;
#endif
  }
  std::cout << "Start test for " << display << " renderer..." << std::endl;
  std::cout << "  Screen resolution: " << d->getScreenWidth() << " " << d->getScreenHeight() << std::endl;
  d->setDownScalingFactor(scale);
  d->init(I);
  vpDisplay::display(I);
  vpDisplay::flush(I);

  vpImage<Type> crop;
  vpImageTools::crop(I, vpImagePoint(0, 245), (unsigned int)roi.getHeight(), (unsigned int)roi.getWidth(), crop);
  I.insert(crop, roi.getTopLeft());
  vpDisplay::displayROI(I, roi);
  vpDisplay::flush(I);

  // Compare input and rendered images
  if (sizeof(Type) == 1) {
    itype = "uchar";
    vpImage<Type> Iinsert = I;
    vpImage<vpRGBa> Isampled;
    vpImage<vpRGBa> Icolor;
    vpImageConvert::convert(Iinsert, Icolor);
    Icolor.subsample(scale, scale, Isampled);

    vpImage<vpRGBa> Irendered;
    vpDisplay::getImage(I, Irendered);

    if (Isampled != Irendered) {
      success = false;
      std::cout << "  -- Test width scale= " << scale << " type= " << itype << ": failed" << std::endl;

      std::stringstream ss;
      ss << "Isampled-" << itype << "-scale-" << scale;
#ifdef VISP_HAVE_OPENCV
      ss << ".png";
#else
      ss << ".ppm";
#endif

      vpImageIo::write(Isampled, ss.str());

      ss.str("");
      ss.clear();
      ss << "Irendered-" << itype << "-scale-" << scale;
#ifdef VISP_HAVE_OPENCV
      ss << ".png";
#else
      ss << ".ppm";
#endif

      vpImageIo::write(Irendered, ss.str());
      vpImage<vpRGBa> Idiff;
      vpImageTools::imageDifference(Isampled, Irendered, Idiff);

      ss.str("");
      ss.clear();
      ss << "Idiff-" << itype << "-scale-" << scale;
#ifdef VISP_HAVE_OPENCV
      ss << ".png";
#else
      ss << ".ppm";
#endif

      vpImageIo::write(Idiff, ss.str());
    } else {
      std::cout << "  ++ Test width scale= " << scale << " type= " << itype << ": succeed" << std::endl;
    }
  } else {
    itype = "rgba";
    vpImage<Type> Iinsert = I;
    vpImage<Type> Isampled; // vpRGBa necessary
    Iinsert.subsample(scale, scale, Isampled);

    vpImage<vpRGBa> Irendered; // vpRGBa necessary
    vpDisplay::getImage(I, Irendered);

    vpImage<vpRGBa> IsampledCopy; // vpRGBa necessary

    vpImageConvert::convert(Isampled, IsampledCopy);
    if (IsampledCopy != Irendered) {
      success = false;
      std::cout << "  -- Test width scale= " << scale << " type= " << itype << ": failed" << std::endl;

      std::stringstream ss;
      ss << "Isampled-" << itype << "-scale-" << scale;
#ifdef VISP_HAVE_OPENCV
      ss << ".png";
#else
      ss << ".ppm";
#endif

      vpImageIo::write(Isampled, ss.str());

      ss.str("");
      ss.clear();
      ss << "Irendered-" << itype << "-scale-" << scale;
#ifdef VISP_HAVE_OPENCV
      ss << ".png";
#else
      ss << ".ppm";
#endif

      vpImageIo::write(Irendered, ss.str());
      vpImage<vpRGBa> Idiff;
      vpImageTools::imageDifference(IsampledCopy, Irendered, Idiff);
      ss.str("");
      ss.clear();
      ss << "Idiff-" << itype << "-scale-" << scale;
#ifdef VISP_HAVE_OPENCV
      ss << ".png";
#else
      ss << ".ppm";
#endif

      vpImageIo::write(Idiff, ss.str());

    } else {
      std::cout << "  ++ Test width scale= " << scale << " type= " << itype << ": succeed" << std::endl;
    }
  }

  vpDisplay::displayRectangle(I, center - v_offset - h_offset, radius, radius, vpColor::blue, false, thickness);
  vpDisplay::displayRectangle(I, center, center + v_offset + h_offset, vpColor::blue, false, thickness);
  vpDisplay::displayRectangle(I, vpRect(center - v_offset - h_offset, center + v_offset + h_offset), vpColor::blue,
                              false, thickness);
  vpDisplay::displayRectangle(I, center - v_offset * 3. / 2 + h_offset, radius / 2, radius / 2, vpColor::green, true);
  vpDisplay::displayCircle(I, center, radius, vpColor::blue, false, thickness);
  vpDisplay::displayArrow(I, center, center - v_offset / 4 - h_offset, vpColor::red, 10, 6, thickness);
  vpDisplay::displayCross(I, center - radius / 2., radius, vpColor::green, thickness);
  vpDisplay::displayDotLine(I, center - v_offset - h_offset, center, vpColor::cyan, thickness);
  vpDisplay::displayLine(I, center + v_offset - h_offset, center - v_offset + h_offset, vpColor::cyan, thickness);
  int nbpoints = (int)(radius * sqrt(2.) / 8 / scale);
  for (int i = 0; i < nbpoints; i++) {
    vpDisplay::displayPoint(
        I, center - h_offset / 2. + vpImagePoint(-i * radius_ / (nbpoints * 2), i * radius_ / (nbpoints * 2)),
        vpColor::cyan);
    vpDisplay::displayPoint(I, center - h_offset + vpImagePoint(-i * radius_ / nbpoints, i * radius_ / nbpoints),
                            vpColor::cyan, thickness);
  }

  if (click)
    vpDisplay::displayText(I, 10 * scale_, 10 * scale_, "A click to continue", vpColor::red);
  else
    vpDisplay::displayText(I, 10 * scale_, 10 * scale_, "This is an image", vpColor::red);

  vpDisplay::flush(I);

  vpImage<vpRGBa> Irendered;
  vpDisplay::getImage(I, Irendered);

  std::stringstream ss;
  ss << "overlay-" << display << "-" << itype << "-scale-" << scale;
#ifdef VISP_HAVE_OPENCV
  ss << ".png";
#else
  ss << ".ppm";
#endif
  std::cout << "   Overlay saved in: " << ss.str() << std::endl;
  vpImageIo::write(Irendered, ss.str());

  if (click)
    vpDisplay::getClick(I);

  // Restore the input image
  I = Ibackup;

  vpDisplay::close(I);

  if (d != NULL)
    delete d;

  if (success)
    return true;
  else
    return false;
}

int main(int argc, const char *argv[])
{
  bool opt_click = true;
  bool opt_display = true;
  std::string opt_ipath;
  std::string env_ipath;
  std::string ipath;

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "-c")
      opt_click = false;
    else if (std::string(argv[i]) == "-d")
      opt_display = false;
    else if (std::string(argv[i]) == "-i")
      opt_ipath = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0] << " [-i <image path>] [-c] [-d] [--help]\n" << std::endl;
      std::cout << "\nOptions: " << std::endl;
      std::cout << "  -i <input image path> : set image input path.\n"
                << "       From this path read \"Klimt/Klimt.pgm\" image.\n"
                << "       Setting the VISP_INPUT_IMAGE_PATH environment\n"
                << "       variable produces the same behaviour than using\n"
                << "       this option." << std::endl;
      std::cout << "  -c : disable mouse click" << std::endl;
      std::cout << "  -d : disable display" << std::endl;
      std::cout << "  -h, --help : print this help\n" << std::endl;
      return 0;
    }
  }

  // Get the visp-images-data package path or VISP_INPUT_IMAGE_PATH
  // environment variable value
  env_ipath = vpIoTools::getViSPImagesDataPath();

  // Set the default input path
  if (!env_ipath.empty())
    ipath = env_ipath;

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;

  std::string filename;

  std::vector<std::string> display;
  if (opt_display) {
#ifdef VISP_HAVE_GDI
    display.push_back("GDI");
#endif
#ifdef VISP_HAVE_GTK
    display.push_back("GTK");
#endif
#ifdef VISP_HAVE_X11
    display.push_back("X");
#endif
#ifdef VISP_HAVE_OPENCV
    display.push_back("OpenCV");
#endif
#ifdef VISP_HAVE_D3D9
    display.push_back("D3D9");
#endif

    if (display.size() == 0) {
      std::cout << "No display available. We stop here." << std::endl;
      return 0;
    }
    vpImage<unsigned char> I;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.pgm");
    vpImageIo::read(I, filename);

    vpImage<vpRGBa> C;
    filename = vpIoTools::createFilePath(ipath, "Klimt/Klimt.ppm");
    vpImageIo::read(C, filename);

    int nbfailure = 0;

    for (unsigned int i = 0; i < display.size(); i++) {

      for (unsigned int scale = 1; scale < 4; scale++) {
        if (!test(display[i], I, scale, opt_click))
          nbfailure++;
        if (!test(display[i], C, scale, opt_click))
          nbfailure++;
      }
    }
    if (nbfailure == 0)
      std::cout << "Test succeed" << std::endl;
    else
      std::cout << "Test failed with " << nbfailure << " failures" << std::endl;
  }

  return 0;
}
