/****************************************************************************
 *
 * $Id: imageDiskRW.cpp 2158 2009-05-07 07:24:51Z fspindle $
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
 * Reading a video file.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file videoReader.cpp
  \brief   reading a video file using vpVideoReader class.
 */

/*!
  \example videoReader.cpp
  Reading a video file using vpVideoReader class.
 */

#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>
#include <visp/vpDebug.h>
#include <visp/vpVideoReader.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GTK)

// List of allowed command line options
#define GETOPTARGS	"cdi:p:h"

void usage(const char *name, const char *badparam, std::string ipath, std::string ppath);
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath,
                bool &click_allowed, bool &display);

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.
\param ipath : Input video path.
\param ppath : Personal video path.

 */
void usage(const char *name, const char *badparam, std::string ipath, std::string ppath)
{
  fprintf(stdout, "\n\
Read a video file on the disk.\n\
\n\
SYNOPSIS\n\
  %s [-i <input video path>] \n\
     [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input video path>                                %s\n\
     Set video input path.\n\
     From this path read \"ViSP-images/video/video.mpeg\"\n\
     video.\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
\n\
  -p <personal video path>                             %s\n\
     Specify a personal folder containing a video \n\
     to process.\n\
     Example : \"/Temp/ViSP-images/video/video.mpeg\"\n\
\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n\n",
	  ipath.c_str(), ppath.c_str());

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input video path.
  \param ppath : Personal video path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.
  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, std::string &ipath, std::string &ppath,
                bool &click_allowed, bool &display)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'i': ipath = optarg_; break;
    case 'p': ppath = optarg_; break;
    case 'h': usage(argv[0], NULL, ipath, ppath); return false; break;

    default:
      usage(argv[0], optarg_, ipath, ppath); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}



int
main(int argc, const char ** argv)
{
  try {
    std::string env_ipath;
    std::string opt_ipath;
    std::string ipath;
    std::string opt_ppath;
    std::string filename;
    bool opt_click_allowed = true;
    bool opt_display = true;

    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout <<  "  videoReader.cpp" <<std::endl << std::endl ;

    std::cout <<  "  reading a video file" << std::endl ;
    std::cout <<  "-------------------------------------------------------" << std::endl ;
    std::cout << std::endl ;


    // Get the VISP_IMAGE_PATH environment variable value
    char *ptenv = getenv("VISP_INPUT_IMAGE_PATH");
    if (ptenv != NULL)
      env_ipath = ptenv;

    // Set the default input path
    if (! env_ipath.empty())
      ipath = env_ipath;

    // Read the command line options
    if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_click_allowed,
                   opt_display) == false) {
      exit (-1);
    }

    // Get the option values
    if (!opt_ipath.empty())
      ipath = opt_ipath;

    // Compare ipath and env_ipath. If they differ, we take into account
    // the input path comming from the command line option
    if (!opt_ipath.empty() && !env_ipath.empty() && opt_ppath.empty()) {
      if (ipath != env_ipath) {
        std::cout << std::endl
                  << "WARNING: " << std::endl;
        std::cout << "  Since -i <visp image path=" << ipath << "> "
                  << "  is different from VISP_IMAGE_PATH=" << env_ipath << std::endl
                  << "  we skip the environment variable." << std::endl;
      }
    }

    // Test if an input path is set
    if (opt_ipath.empty() && env_ipath.empty() && opt_ppath.empty()){
      usage(argv[0], NULL, ipath, opt_ppath);
      std::cerr << std::endl
                << "ERROR:" << std::endl;
      std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
                << std::endl
                << "  environment variable to specify the location of the " << std::endl
                << "  video path where test images are located." << std::endl << std::endl;
      exit(-1);
    }


    /////////////////////////////////////////////////////////////////////


    // vpImage is a template class you can declare vpImage of ... everything...
    vpImage<vpRGBa> I ;

    //Create the video Reader
    vpVideoReader reader;

    if (opt_ppath.empty())
    {
      filename = ipath +  vpIoTools::path("/ViSP-images/video/cube.mpeg");
    }
    else
    {
      filename.assign(opt_ppath);
    }

    //Initialize the reader and get the first frame.
    reader.setFileName(filename.c_str());
    reader.open(I);

    // We open a window using either X11, GTK, GDI or OpenCV.
#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#elif defined VISP_HAVE_OPENCV
    vpDisplayOpenCV display;
#endif

    if (opt_display) {
      // Display size is automatically defined by the image (I) size
      display.init(I, 100, 100,"Display video frame") ;
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }

    //   if (opt_display && opt_click_allowed)
    //   {
    //     std::cout << "Click on the image to read and display the last key frame" << std::endl;
    //     vpDisplay::getClick(I);
    //   }
    //
    //   reader.getFrame(I,reader.getLastFrameIndex());
    //
    //   if (opt_display)
    //   {
    //     vpDisplay::display(I) ;
    //     vpDisplay::flush(I);
    //   }

    if (opt_display && opt_click_allowed)
    {
      std::cout << "Click to see the video" << std::endl;
      vpDisplay::getClick(I);
    }

    int lastFrame = reader.getLastFrameIndex();
    //To go to the beginning of the video
    reader.getFrame(I,0);

    for (int i = 0; i <= lastFrame; i++)
    {
      reader.acquire(I);
      if (opt_display)
      {
        vpDisplay::display(I) ;
        vpDisplay::flush(I);
      }
    }

    if (opt_display && opt_click_allowed)
    {
      std::cout << "Click to exit the test" << std::endl;
      vpDisplay::getClick(I);
    }
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
  }
  return 0;
}
#else
int main()
{
  std::cout << "Sorry, no display is available. We quit this example." 
            << std::endl;
  return 0;
}
#endif
