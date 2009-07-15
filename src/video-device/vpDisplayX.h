/****************************************************************************
 *
 * $Id$
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
 * Image display.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpDisplayX_h
#define vpDisplayX_h

#include <visp/vpConfig.h>
#ifdef VISP_HAVE_X11

//namespace X11name
//{
#include <X11/Xlib.h>
#include <X11/Xutil.h>
//#include <X11/Xatom.h>
//#include <X11/cursorfont.h>
//} ;

//using namespace X11name ;

#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <visp/vpRect.h>



/*!
  \file vpDisplayX.h
  \brief Define the X11 console to display images.
*/


/*!
  \class vpDisplayX

  \ingroup ImageGUI

  \brief Define the X11 console to display images.

  \note Ready to use with a Unix System (tested for Linux and SunOS)

  This class define the X11 console to display  images
  It also define method to display some geometric feature (point, line, circle)
  in the image.

  The example below shows how to display an image with this video device.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpImagePoint.h>

int main() 
{
#if defined(VISP_HAVE_X11)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
  vpImageIo::readPGM(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");

  vpDisplayX d; 

  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My X11 display");

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(10);
  topLeftCorner.set_j(20);
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::red, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for a click in the display window
  vpDisplay::getClick(I);
#endif
}
  \endcode

*/

class VISP_EXPORT vpDisplayX: public vpDisplay
{
private:
  // true if X11 display is ready to use
  bool Xinitialise ;

  int num_Xdisplay ;
  Display 	*display ;
  Window   	window ;
  XImage   	*Ximage ;
  Colormap	lut ;
  GC		context ;
  int      	screen ;
  int		planes;
  XEvent	event;
  Pixmap	pixmap;
  unsigned long	*x_color; // Array of predefined colors
  int screen_depth  ;
  unsigned short  colortable[256];
  XColor        xcolor;
  XGCValues     values;
  int size ;
  bool ximage_data_init;
  
public:
  vpDisplayX() ;
  vpDisplayX(int winx, int winy, const char *title=NULL) ;
  vpDisplayX(vpImage<unsigned char> &I, int winx=-1, int winy=-1,
	     const char *title=NULL) ;
  vpDisplayX(vpImage<vpRGBa> &I, int winx=-1, int winy=-1,
	     const char *title=NULL) ;


  virtual ~vpDisplayX() ;

  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    const char *title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	   int winx=-1, int winy=-1,
	   const char *title=NULL)  ;

  void init(unsigned int width, unsigned int height,
	    int winx=-1, int winy=-1 ,
	    const char *title=NULL) ;
  
  unsigned int getScreenDepth();
  void getScreenSize(unsigned int &width, unsigned int &height);
  void getImage(vpImage<vpRGBa> &I) ;

protected:

  void setFont( const char *font );
  void setTitle(const char *title) ;
  void setWindowPosition(int winx, int winy);

  void clearDisplay(vpColor color=vpColor::white) ;

  void closeDisplay() ;

  void displayArrow(const vpImagePoint &ip1, 
		    const vpImagePoint &ip2,
		    vpColor color=vpColor::white,
		    unsigned int w=4,unsigned int h=2,
		    unsigned int thickness=1) ;

  void displayCharString(const vpImagePoint &ip, const char *text,
			 vpColor color=vpColor::green) ;

  void displayCircle(const vpImagePoint &center, unsigned int radius,
		     vpColor color,
		     bool fill = false,
		     unsigned int thickness=1);
  void displayCross(const vpImagePoint &ip, unsigned int size,
		    vpColor color, unsigned int thickness=1) ;
  void displayDotLine(const vpImagePoint &ip1, 
		      const vpImagePoint &ip2,
		      vpColor color, unsigned int thickness=1) ;

  void displayImage(const vpImage<vpRGBa> &I) ;
  void displayImage(const vpImage<unsigned char> &I) ;
  void displayImage(const unsigned char *I) ;

  void displayLine(const vpImagePoint &ip1, 
		   const vpImagePoint &ip2,
		   vpColor color, unsigned int thickness=1) ;
  void displayPoint(const vpImagePoint &ip, vpColor color) ;

  void displayRectangle(const vpImagePoint &topLeft,
			unsigned int width, unsigned int height,
			vpColor color, bool fill = false,
			unsigned int thickness=1) ;
  void displayRectangle(const vpImagePoint &topLeft,
			const vpImagePoint &bottomRight,
			vpColor color, bool fill = false,
			unsigned int thickness=1) ;
  void displayRectangle(const vpRect &rectangle,
			vpColor color, bool fill = false,
			unsigned int thickness=1) ;

  void flushDisplay() ;

  bool getClick(bool blocking=true) ;
  bool getClick(vpImagePoint &ip, bool blocking=true);
  bool getClick(vpImagePoint &ip,
		vpMouseButton::vpMouseButtonType& button,
		bool blocking=true) ;
  bool getClickUp(vpImagePoint &ip,
		  vpMouseButton::vpMouseButtonType& button,
		  bool blocking=true);

  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }
} ;

#endif
#endif
