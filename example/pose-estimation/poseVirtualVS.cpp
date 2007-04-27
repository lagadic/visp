/****************************************************************************
 *
 * $Id: poseVirtualVS.cpp,v 1.5 2007-04-27 16:40:14 fspindle Exp $
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
 * This file is part of the ViSP toolkit.
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
 * Pose computation on an object made of dots.
 *   reading of PGM image
 *   Display image using either the X11 or GTK or GDI display
 *   track 4 dots (vpDots) in the image
 *   compute the pose
 *
 * Authors:
 * Eric Marchand
 * Anthony Saunier
 *
 *****************************************************************************/
/*!
  \file poseVirtualVS.cpp

  \brief Example of dots tracking in an image sequence and pose computation.

  Pose computation on an object made of dots :
    reading of PGM image,
    Display image using either the X11 or GTK or GDI display,
    track 4 dots (vpDots) in the image,
    compute the pose.

*/

/*!
  \example poseVirtualVS.cpp
  Example of dots tracking in an image sequence and pose
  computation.
*/


#include <stdio.h>
#include <sstream>
#include <iomanip>
#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(WIN32))

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>

#if defined VISP_HAVE_X11
#include <visp/vpDisplayX.h>
#elif defined VISP_HAVE_GTK
#include <visp/vpDisplayGTK.h>
#elif defined WIN32
#include <visp/vpDisplayGDI.h>
#endif

#include <visp/vpPose.h>
#include <visp/vpDot.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>


// List of allowed command line options
#define GETOPTARGS	"cdi:p:hf:n:s:"

/*!

Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param nimages : Number of images to manipulate.
  \param step : Step between two images.

 */
void usage(char *name, char *badparam, std::string ipath, std::string ppath,
	   unsigned first, unsigned nimages, unsigned step)
{
  fprintf(stdout, "\n\
Test dot tracking.\n\
\n\
SYNOPSIS\n\
  %s [-i <input image path>] [-p <personal image path>]\n\
     [-f <first image>] [-n <number of images>] [-s <step>][-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -i <input image path>                                %s\n\
     Set image input path.\n\
     From this path read images \n\
     \"ViSP-images/cube/image.%%04d.pgm\"\n\
     Setting the VISP_INPUT_IMAGE_PATH environment\n\
     variable produces the same behaviour than using\n\
     this option.\n\
 \n\
  -p <personal image path>                             %s\n\
     Specify a personal sequence containing images \n\
     to process.\n\
     By image sequence, we mean one file per image.\n\
     The following image file formats PNM (PGM P5, PPM P6)\n\
     are supported. The format is selected by analysing \n\
     the filename extension.\n\
     Example : \"/Temp/ViSP-images/cube/image.%%04d.pgm\"\n\
     %%04d is for the image numbering.\n\
 \n\
  -f <first image>                                     %u\n\
     First image number of the sequence.\n\
 \n\
  -n <number of images>                                %u\n\
     Number of images to load from the sequence.\n\
 \n\
  -s <step>                                            %u\n\
     Step between two images.\n\
\n\
  -c\n\
     Disable the mouse click. Usefull to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n",
	  ipath.c_str(),ppath.c_str(), first, nimages, step);

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param ppath : Personal image path.
  \param first : First image.
  \param nimages : Number of images to display.
  \param step : Step between two images.
  \param display : Set as true, activates the image display. This is
the default configuration. When set to false, the display is
disabled. This can be usefull for automatic tests using crontab
under Unix or using the task manager under Windows.
\param click_allowed : set to false, disable the mouse click.

\return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv, std::string &ipath, std::string &ppath,
		unsigned &first, unsigned &nimages, unsigned &step,
		bool &click_allowed, bool &display)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'i': ipath = optarg; break;
    case 'p': ppath = optarg; break;
    case 'f': first = (unsigned) atoi(optarg); break;
    case 'n': nimages = (unsigned) atoi(optarg); break;
    case 's': step = (unsigned) atoi(optarg); break;
    case 'h': usage(argv[0], NULL, ipath, ppath, first, nimages, step); return false; break;

    default:
      usage(argv[0], optarg, ipath, ppath, first, nimages, step);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL, ipath, ppath, first, nimages, step);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}




int
main(int argc, char** argv)
{
  std::string env_ipath;
  std::string opt_ipath;
  std::string ipath;
  std::string opt_ppath;
  std::string dirname;
  std::string filename;
  unsigned opt_first = 0;
  unsigned opt_nimages = 80;
  unsigned opt_step = 1;
  bool opt_click_allowed = true;
  bool opt_display = true;


  int i ;

  std::cout <<  "-------------------------------------------------------" << std::endl ;
  std::cout <<  "  poseVirtualVS.cpp" <<std::endl << std::endl ;

  std::cout <<  "  Example of dots tracking in an image sequence and pose computation" << std::endl ;
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
  if (getOptions(argc, argv, opt_ipath, opt_ppath, opt_first, opt_nimages, opt_step, opt_click_allowed, opt_display) == false) {
    exit (-1);
  }

  // Get the option values
  if (!opt_ipath.empty())
    ipath = opt_ipath;

  // Compare ipath and env_ipath. If they differ, we take into account
  // the input path comming from the command line option
  if (opt_ipath.empty() && opt_ppath.empty()) {
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
    usage(argv[0], NULL, ipath, opt_ppath, opt_first, opt_nimages, opt_step);
    std::cerr << std::endl
	 << "ERROR:" << std::endl;
    std::cerr << "  Use -i <visp image path> option or set VISP_INPUT_IMAGE_PATH "
	 << std::endl
	 << "  environment variable to specify the location of the " << std::endl
	 << "  image path where test images are located." << std::endl
	 << "  Use -p <personal image path> option if you want to "<< std::endl
	 << "  use personal images" <<std::endl << std::endl;
    exit(-1);
  }




  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I ;

  unsigned iter = opt_first ;
  std::ostringstream s;
  char cfilename[FILENAME_MAX];

  if (opt_ppath.empty()){

    // Warning :
    // the image sequence is not provided with the ViSP package
    // therefore the program will return you an error :
    //  !!    vpImageIoPnm.cpp: readPGM(#210) :couldn't read file /ViSP-images/cube/image.0001.pgm
    //  !!    poseExample.cpp: main(#95) :Error while reading the image
    //  terminate called after throwing an instance of 'vpImageException'
    //
    //  The sequence is available on the visp www site
    //  http://www.irisa.fr/lagadic/visp/visp.html
    //  in the download section. It is named "ViSP-images-x.y.z.tar.gz"

    // directory name
    dirname = ipath +  vpIoTools::path("/ViSP-images/cube/");


    // Build the name of the image file

    s.setf(std::ios::right, std::ios::adjustfield);
    s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
    filename = dirname + s.str();
  }
  else {

    sprintf(cfilename, opt_ppath.c_str(), iter) ;
    filename = cfilename;
  }

  // define the vpDot structure, here 4 dots will tracked
  vpDot d[4] ;

  for (i=0 ; i < 4 ; i++)
    {
      // by using setGraphics, we request to see the all the pixel of the dot
      // in green on the screen.
      // It uses the overlay image plane.
      // The default of this setting is that it is time consumming

      if (opt_display) {
	d[i].setGraphics(true) ;
      }
      else {
	d[i].setGraphics(false) ;
      }
    }

  // Read the PGM image named "s" on the disk, and put the bitmap into the
  // image structure I.
  // I is initialized to the correct size
  //
  // exception readPGM may throw various exception if, for example,
  // the file does not exist, or if the memory cannot be allocated
  try{
    vpImageIo::readPGM(I,filename) ;
  }
  catch(...)
    {
      // an exception is throwned if an exception from readPGM has been catched
      // here this will result in the end of the program
      // Note that another error message has been printed from readPGM
      // to give more information about the error
      if (opt_ppath.empty()) {
	std::cerr << std::endl
	     << "ERROR:" << std::endl;
	std::cerr << "  Cannot read " << filename << std::endl;
	std::cerr << "  Check your -i " << ipath << " option, " << std::endl
	     << "  or VISP_INPUT_IMAGE_PATH environment variable"
	     << std::endl;
      }
      else {
	std::cerr << std::endl
	     << "ERROR:" << std::endl;
	std::cerr << "  Cannot read " << filename << std::endl;
	std::cerr << "  or your -p " << opt_ppath << " option " <<std::endl
	     << std::endl;
      }
      exit(-1);

    }

  // We open a window using either the X11 or GTK or GDI window manager
  // it will be located in 100,100 and titled "tracking using vpDot"
  // its size is automatically defined by the image (I) size
#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined WIN32
  vpDisplayGDI display;
#endif
  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display.init(I,100,100,"tracking using vpDot");
      // display the image
      // The image class has a member that specify a pointer toward
      // the display that has been initialized in the display declaration
      // therefore is is no longuer necessary to make a reference to the
      // display variable.
      vpDisplay::display(I) ;
    }
    catch(...)
      {
	vpERROR_TRACE("Error while displaying the image") ;
	return(-1) ;
      }
  }



  double u[4],v[4] ;
  try{
    if (opt_display && opt_click_allowed) {
      // dot coordinates (u,v) = (column,row)
      std::cout << "Click the four white  on the object corner clockwise" <<std::endl  ;
      for (i=0 ; i < 4 ; i++)
	{
	  // tracking is initalized
	  // if no other parameters are given to the iniTracking(..) method
	  // a right mouse click on the dot is expected
	  // dot location can also be specified explicitely in the initTracking
	  // method  : d.initTracking(I,u,v)  where u is the column index and v is
	  // the row index

	  d[i].initTracking(I) ;
	  // track the dot and returns its coordinates in the image
	  // results are given in float since many many are usually considered
	  //
	  // an expcetion is thrown by the track method if
	  //  - dot is lost
	  //  - the number of pixel is too small
	  //  - too many pixels are detected (this is usual when a "big" specularity
	  //    occurs. The threshold can be modified using the
	  //    setNbMaxPoint(int) method
	  d[i].track(I,u[i],v[i]) ;
	}
    }
    else{
      d[0].initTracking(I,194,88 );
      d[0].track(I,u[0],v[0]);
      d[1].initTracking(I, 225, 84);
      d[1].track(I,u[1],v[1]);
      d[2].initTracking(I, 242, 114);
      d[2].track(I,u[2],v[2]);
      d[3].initTracking(I, 212, 131);
      d[3].track(I,u[3],v[3]);
    }
  }
  catch(...)
    {
      vpERROR_TRACE("Error in tracking initialization ") ;
      return(-1) ;
    }

  if (opt_display)
    {

      // display a red cross (size 10) in the image at the dot center
      // of gravity location
      //
      // WARNING
      // in the vpDisplay class member's when pixel coordinates
      // are considered the first element is the row index and the second
      // is the column index:
      //   vpDisplay::displayCross(Image, row index, column index, size, color)
      //   therefore u and v are inverted wrt to the vpDot specification
      // Alternatively, to avoid this problem another set of member have
      // been defined in the vpDisplay class.
      // If the method name is postfixe with _uv the specification is :
      //   vpDisplay::displayCross_uv(Image, column index, row index, size, color)

      for (i=0 ; i < 4 ; i++)
	vpDisplay::displayCross(I,
				(int)v, (int)u,
				10,
				vpColor::red) ;

      // flush the X11 buffer
      vpDisplay::flush(I) ;
    }

  // --------------------------------------------------------
  // Now wil compute the pose
  //

  // The pose will be contained in an homogeneous matrix cMo
  vpHomogeneousMatrix cMo ;


  // We need a structure that content both the 3D coordinates of the point
  // in the object frame and the 2D coordinates of the point expressed in meter
  // the vpPoint class is ok for that
  vpPoint P[4]  ;

  // The vpPose class mainly contents a list of vpPoint (that is (X,Y,Z, x, y) )
  vpPose pose ;
  //  the list of point is cleared (if that's not done before)
  pose.clearPoint() ;

  // we set the 3D points coordinates (in meter !) in the object/world frame
  double L=0.04 ;
  P[0].setWorldCoordinates(-L,-L, 0 ) ; // (X,Y,Z)
  P[1].setWorldCoordinates(L,-L, 0 ) ;
  P[2].setWorldCoordinates(L,L, 0 ) ;
  P[3].setWorldCoordinates(-L,L, 0 ) ;


  // set the camera intrinsic parameters
  // see more details about the model in vpCameraParameters
  double px = 600 ;
  double py = 600 ;
  double u0 = 192 ;
  double v0 = 144 ;
  vpCameraParameters cam(px,py,u0,v0) ;

  // pixel-> meter conversion
  for (i=0 ; i < 4 ; i++)
    {
      // u[i]. v[i] are expressed in pixel
      // conversion in meter is achieved using
      // x = (u-u0)/px
      // y = (v-v0)/py
      // where px, py, u0, v0 are the intrinsic camera parameters
      double x,y ;
      vpPixelMeterConversion::convertPoint(cam,
					   u[i], v[i],
					   x,y)  ;
      P[i].set_x(x) ;
      P[i].set_y(y) ;
    }


  // The pose structure is build, we put in the point list the set of point
  // here both 2D and 3D world coordinates are known
  for (i=0 ; i < 4 ; i++)
    {
      pose.addPoint(P[i]) ; // and added to the pose computation point list
    }

  // compute the initial pose using Dementhon method followed by a non linear
  // minimisation method

  // Pose by Lagrange it provides an initialization of the pose
  pose.computePose(vpPose::LAGRANGE, cMo) ;
  // the pose is now refined using the virtual visual servoing approach
  // Warning: cMo needs to be initialized otherwise it may  diverge
  pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
  if( opt_display ){
    // display the compute pose
    pose.display(I,cMo,cam, 0.05, vpColor::red) ;
  }

  unsigned niter = 0;
  // this is the loop over the image sequence
  while (iter < opt_nimages)
    {
      try {
	// set the new image name

	if (opt_ppath.empty()){
	  s.str("");
	  s << "image." << std::setw(4) << std::setfill('0') << iter << ".pgm";
	  filename = dirname + s.str();
	}
	else {
	  sprintf( cfilename, opt_ppath.c_str(), iter) ;
	  filename = cfilename ;
	}

	// read the image
	vpImageIo::readPGM(I,filename) ;
	if (opt_display){
	  // Display the image
	  vpDisplay::display(I) ;
	}
	// kill the point list
	pose.clearPoint() ;

	// track the dot
	for (i=0 ; i < 4 ; i++)
	  {
	    // track the point
	    d[i].track(I,u[i],v[i]) ;
	    if (opt_display){
	      // display point location
	      vpDisplay::displayCross(I,(int)v[i], (int)u[i],
				      10,vpColor::red) ;
	    }
	    // pixel->meter conversion
	    {
	      double x,y ;
	      vpPixelMeterConversion::convertPoint(cam,
						   u[i], v[i],
						   x,y)  ;
	      P[i].set_x(x) ;
	      P[i].set_y(y) ;
	    }

	    // and added to the pose computation point list
	    pose.addPoint(P[i]) ;
	  }
	// the pose structure has been updated


	// the pose is now updated using the virtual visual servoing approach
	// Dementhon or lagrange is no longuer necessary, pose at the
	// previous iteration is sufficient
	pose.computePose(vpPose::VIRTUAL_VS, cMo) ;
	if (opt_display) {
	  // display the compute pose
	  pose.display(I,cMo,cam, 0.05, vpColor::red) ;

	  vpDisplay::flush(I) ;
	}
	niter++ ;
      }
      catch(...){
	vpERROR_TRACE("Error in tracking loop") ;
	return(-1) ;
      }
      iter += opt_step ;
    }
}
#else
int
main()
{
  vpERROR_TRACE("You do not have X11 or GTK functionalities to display images...");
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
