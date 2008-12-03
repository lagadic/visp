/****************************************************************************
 *
 * $Id: vpDisplayX.h,v 1.24 2008-12-03 10:25:11 nmelchio Exp $
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
  \brief Define the X11 console to display images
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

  // Set the display window title
  vpDisplay::setTitle(I, "My X11 display");

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

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
  unsigned char *bitmap_bis ;
  unsigned long	x_color[vpColor::none];
  int screen_depth  ;
  unsigned short  colortable[256];
  XColor        color;
  XGCValues     values;
  int size ;
  bool ximage_data_init;
  


public:
  vpDisplayX(vpImage<unsigned char> &I, int _winx=-1, int _winy=-1,
	     const char *title=NULL) ;
  vpDisplayX(vpImage<vpRGBa> &I, int _winx=-1, int _winy=-1,
	     const char *title=NULL) ;

  vpDisplayX(int _winx, int _winy, const char *title=NULL) ;

  vpDisplayX() ;
  virtual ~vpDisplayX() ;


  unsigned int getScreenDepth();
  void getScreenSize(unsigned int &width, unsigned int &height);
  void getImage(vpImage<vpRGBa> &I) ;

  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    const char *_title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	   int winx=-1, int winy=-1,
	   const char *_title=NULL)  ;

  void init(unsigned int width, unsigned int height,
	    int winx=-1, int winy=-1 ,
	    const char *_title=NULL) ;
  
protected:

  void clearDisplay(vpColor::vpColorType c=vpColor::white) ;

  void closeDisplay() ;

  void displayArrow(int i1, int j1, int i2, int j2,
		    vpColor::vpColorType col=vpColor::white,
		    unsigned int L=4,unsigned int l=2) ;

  void displayCharString(int i, int j,const char *s,
			 vpColor::vpColorType c=vpColor::green) ;

  void displayCircle(int i, int j, unsigned int r,
		     vpColor::vpColorType c);
  void displayCross(int i, int j, unsigned int size,
		    vpColor::vpColorType col) ;
  void displayCrossLarge(int i, int j, unsigned int size,
			 vpColor::vpColorType col) ;
  void displayDotLine(int i1, int j1, int i2, int j2,
		      vpColor::vpColorType col, unsigned int e=1) ;
  void displayImage(const vpImage<vpRGBa> &I) ;
  void displayImage(const vpImage<unsigned char> &I) ;
  void displayImage(const unsigned char *I) ;

  void displayLine(int i1, int j1, int i2, int j2,
		   vpColor::vpColorType col, unsigned int e=1) ;


  void displayPoint(int i, int j, vpColor::vpColorType col) ;
  void displayRectangle(int i, int j,
			unsigned int width, unsigned int height,
			vpColor::vpColorType col, bool fill = false,
			unsigned int e=1);
  void displayRectangle(const vpRect &rect,
			vpColor::vpColorType col, bool fill = false,
			unsigned int e=1);
  void flushDisplay() ;
  void setTitle(const char *string) ;
  void setFont( const char *string );

  

  bool getClick(unsigned int& i, unsigned int& j, bool blocking=true);
  bool getClick(unsigned int& i, unsigned int& j,
		vpMouseButton::vpMouseButtonType& button,
		bool blocking=true) ;
  bool getClick(bool blocking=true) ;
  bool getClickUp(unsigned int& i, unsigned int& j,
		  vpMouseButton::vpMouseButtonType& button,
		  bool blocking=true);

  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }

  void setWindowPosition(int /*_winx*/, int /*_winy*/) { ; }
  /*!
    @name Deprecated functions
  */
  void flushTitle(const char *string) ;


} ;

#endif
#endif
