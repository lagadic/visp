/****************************************************************************
 *
 * $Id: testVideoDevice.cpp,v 1.11 2008-06-13 13:37:40 asaunier Exp $
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
 * This file is part of the ViSP toolkit
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
 * Test for image display.
 *
 * Authors:
 * Anthony Saunier
 *
 *****************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <string>

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if (defined (VISP_HAVE_GTK) || defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) || defined(VISP_HAVE_OPENCV))

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpRect.h>

#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayOpenCV.h>

/*!
  \example testDisplays.cpp

  \brief Test all the displays. Draws several shapes.

*/

// List of allowed command line options
#define GETOPTARGS	"hl:dc"

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

 */
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test video devices or display.\n\
\n\
SYNOPSIS\n\
  %s [-t <type of video device>] [-l] [-c] [-d] [-h]\n\
", name);

  std::string display;

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c\n\
     Disable the mouse click. Usefull to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -l\n\
     Print the list of video-devices available and exit.\n\
\n\
  -h\n\
     Print the help.\n\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}

/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param list : To get the list of available display.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &list,
		bool &click_allowed, bool &display )
{
  const char *optarg;
  int	c;
  std::string sDisplayType;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'l': list = true; break;
    case 'h': usage(argv[0], NULL); return false; break;
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;

    default:
      usage(argv[0], optarg); return false; break;
    }
  }


  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


void draw(vpImage<vpRGBa> &I)
{
  vpImagePoint iP1, iP2;
  unsigned int w,h;

  iP1.set_i(20);
  iP1.set_j(10);
  iP2.set_i(20);
  iP2.set_j(30);
  vpDisplay::displayArrow (I, iP1, iP2, vpColor::green, 4, 2, 3);

  iP1.set_i(20);
  iP1.set_j(60);
  vpDisplay::displayCharString (I, iP1, "Test...", vpColor::black);

  iP1.set_i(80);
  iP1.set_j(220);
  iP2.set_i(80);
  iP2.set_j(480);
  vpDisplay::displayCircle (I, iP1, 30, vpColor::red, false, 3);
  vpDisplay::displayCircle (I, iP2, 30, vpColor::red, true, 3);

  iP1.set_i(20);
  iP1.set_j(220);
  vpDisplay::displayCross (I, iP1, 5, vpColor::blue, 1);

  iP1.set_i(140);
  iP1.set_j(10);
  iP2.set_i(140);
  iP2.set_j(50);
  vpDisplay::displayDotLine (I, iP1, iP2, vpColor::blue, 3);

  iP1.set_i(120);
  iP1.set_j(180);
  iP2.set_i(160);
  iP2.set_j(250);
  vpDisplay::displayDotLine (I, iP1, iP2, vpColor::blue, 3);

  iP1.set_i(160);
  iP1.set_j(280);
  iP2.set_i(120);
  iP2.set_j(340);
  vpDisplay::displayDotLine (I, iP1, iP2, vpColor::blue, 3);

//static void 	displayFrame (const vpImage< vpRGBa > &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam, double size, vpColor::vpColorType color)

  iP1.set_i(140);
  iP1.set_j(80);
  iP2.set_i(140);
  iP2.set_j(150);
  vpDisplay::displayLine (I, iP1, iP2, vpColor::orange, 3);

  iP1.set_i(140);
  iP1.set_j(400);
  vpDisplay::displayPoint (I, iP1, vpColor::red);

  iP1.set_i(350);
  iP1.set_j(20);
  w = 60;
  h = 50;
  vpDisplay::displayRectangle (I, iP1, w, h, vpColor::red, false, 3);

  iP1.set_i(350);
  iP1.set_j(110);
  vpDisplay::displayRectangle (I, iP1, w, h, vpColor::red, true, 3);

  iP1.set_i(350);
  iP1.set_j(200);
  iP2.set_i(400);
  iP2.set_j(260);
  vpDisplay::displayRectangle (I, iP1, iP2, vpColor::orange, false, 3);

  iP1.set_i(350);
  iP1.set_j(290);
  iP2.set_i(400);
  iP2.set_j(350);
  vpRect rectangle(iP1, iP2);
  vpDisplay::displayRectangle (I, rectangle, vpColor::yellow, false, 3);

  iP1.set_i(380);
  iP1.set_j(400);
  vpDisplay::displayRectangle (I, iP1, 45, w, h, vpColor::green, 3);


}

int
main(int argc, const char ** argv)
{
  try{
    bool opt_list = false; // To print the list of video devices
    bool opt_click_allowed = true;
    bool opt_display = true;


    // Read the command line options
    if (getOptions(argc, argv, opt_list,
            opt_click_allowed, opt_display) == false) {
      exit (-1);
    }

    // Print the list of video-devices available
    if (opt_list) {
      unsigned nbDevices = 0;
      std::cout << "List of video-devices available: \n";
#if defined VISP_HAVE_GTK
      std::cout << "  GTK (use \"-t GTK\" option to use it)\n";
      nbDevices ++;
#endif
#if defined VISP_HAVE_X11
      std::cout << "  X11 (use \"-t X11\" option to use it)\n";
      nbDevices ++;
#endif
#if defined VISP_HAVE_GDI
      std::cout << "  GDI (use \"-t GDI\" option to use it)\n";
      nbDevices ++;
#endif
#if defined VISP_HAVE_D3D9
      std::cout << "  D3D (use \"-t D3D\" option to use it)\n";
      nbDevices ++;
#endif
#if defined VISP_HAVE_OPENCV
      std::cout << "  CV (use \"-t CV\" option to use it)\n";
      nbDevices ++;
#endif   
      if (!nbDevices) {
        std::cout << "  No display is available\n";
      }
      return (0);
    }

    // Create a grey level image
    vpImage<unsigned char> I;
    // Create a color image
    vpImage<vpRGBa> Irgba;

    // Create a color image for each display.
    vpImage<vpRGBa> Ix;
    vpImage<vpRGBa> Igtk;
    vpImage<vpRGBa> Icv;
    vpImage<vpRGBa> Igdi;
    vpImage<vpRGBa> Id3d;

#if defined VISP_HAVE_X11
      vpDisplayX *displayX = NULL;
      displayX = new vpDisplayX;
      Ix.init(480, 640, 255);
      if (opt_display)
      {
        displayX->init(Ix, 100, 100,"Display X11") ;
        vpDisplay::display(Ix) ;
        draw(Ix);
        vpDisplay::flush(Ix);
        if (opt_click_allowed)
          vpDisplay::getClick(Ix);
      }
#endif

#if defined VISP_HAVE_OPENCV
      vpDisplayOpenCV *displayCv = NULL;
      displayCv = new vpDisplayOpenCV;
      Icv.init(480, 640, 255);
      if (opt_display)
      {
        displayCv->init(Icv, 100, 100,"Display OpenCV") ;
        vpDisplay::display(Icv) ;
        draw(Icv);
        vpDisplay::flush(Icv);
      if (opt_click_allowed)
        vpDisplay::getClick(Icv);
      }
#endif

#if defined VISP_HAVE_GTK
      vpDisplayGTK *displayGtk = NULL;
      displayGtk = new vpDisplayGTK;
      Igtk.init(480, 640, 255);
      if (opt_display)
      {
        displayGtk->init(Igtk, 100, 100,"Display GTK") ;
        vpDisplay::display(Igtk) ;
        draw(Igtk);
        vpDisplay::flush(Igtk);
        if (opt_click_allowed)
          vpDisplay::getClick(Igtk);
      }
#endif

#if defined VISP_HAVE_GDI
      vpDisplayGDI *displayGdi = NULL;
      displayGdi = new vpDisplayGDI;
      Igdi.init(480, 640, 255);
      if (opt_display)
      {
        displayGdi->init(Igdi, 100, 100,"Display GDI") ;
        vpDisplay::display(Igdi) ;
        draw(Igdi);
        vpDisplay::flush(Igdi);
        if (opt_click_allowed)
          vpDisplay::getClick(Igdi);
      }
#endif

#if defined VISP_HAVE_D3D9
      vpDisplayD3D *displayD3d = NULL;
      displayD3d = new vpDisplayD3D;
      Id3d.init(480, 640, 255);
      if (opt_display)
      {
        displayD3d->init(Id3d, 100, 100,"Display Direct 3D") ;
        vpDisplay::display(Id3d) ;
        draw(Id3d);
        vpDisplay::flush(Id3d);
        if (opt_click_allowed)
          vpDisplay::getClick(Id3d);
      }
#endif



#if defined VISP_HAVE_X11
      delete displayX;
#endif

#if defined VISP_HAVE_GTK
      delete displayGtk;
#endif

#if defined VISP_HAVE_OPENCV
      delete displayCv;
#endif

#if defined VISP_HAVE_GDI
      delete displayGdi;
#endif

#if defined VISP_HAVE_D3D9
      delete displayD3d;
#endif
  }
  catch(...) {
    vpERROR_TRACE("Error while displaying the image") ;
    exit(-1);
  }
}

#else
int
main()
{
  vpERROR_TRACE("You do not have display functionalities...");
}

#endif
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
