/****************************************************************************
 *
 * $Id: vpDisplayX.h,v 1.14 2007-06-11 15:52:34 asaunier Exp $
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
  \brief Define the X11 console to display images2323

  \date June 1999

  \author  Fabien Spindler, Irisa / Inria Rennes

  \note Ready to use with a Unix System (tested for Linux and SunOS)

  This class define the X11 console to display  images
  It also define method to display some geometric feature (point, line, circle)
  in the image.
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


protected:
  void setWindowPosition(int /*_winx*/, int /*_winy*/) { ; }
  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }

public:
  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    char *_title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	   int winx=-1, int winy=-1,
	   char *_title=NULL)  ;

  void init(unsigned int width, unsigned int height,
	    int winx=-1, int winy=-1 ,
	    char *_title=NULL) ;
  // only the constructor/destructor are public
public:
  vpDisplayX(vpImage<unsigned char> &I, int _winx=-1, int _winy=-1,
	     char *title=NULL) ;
  vpDisplayX(vpImage<vpRGBa> &I, int _winx=-1, int _winy=-1,
	     char *title=NULL) ;

  vpDisplayX(int _winx, int _winy, char *title=NULL) ;

  vpDisplayX() ;
  virtual ~vpDisplayX() ;

protected:
  void displayImage(const vpImage<vpRGBa> &I) ;
  void displayImage(const vpImage<unsigned char> &I) ;
  void displayImage(const unsigned char *I) ;

  void closeDisplay() ;
  void flushDisplay() ;
  void flushTitle(const char *string) ;

  void clearDisplay(vpColor::vpColorType c=vpColor::white) ;

  void displayPoint(unsigned int x,unsigned int y,vpColor::vpColorType col) ;
  void displayCross(unsigned int x,unsigned int y, unsigned int size,
		    vpColor::vpColorType col) ;
  void displayCrossLarge(unsigned int x,unsigned int y, unsigned int size,
			 vpColor::vpColorType col) ;
  void displayCircle(unsigned int i, unsigned int j, unsigned int r,
		     vpColor::vpColorType c);
  void displayLine(unsigned int x1, unsigned int y1,
		   unsigned int x2, unsigned int y2,
		   vpColor::vpColorType col, unsigned int e=1) ;
  void displayDotLine(unsigned int x1, unsigned int y1,
		      unsigned int x2, unsigned int y2,
		      vpColor::vpColorType col, unsigned int e=1) ;


  void displayArrow(unsigned int i1,unsigned int j1,
		    unsigned int i2, unsigned int j2,
		    vpColor::vpColorType col=vpColor::white,
		    unsigned int L=4,unsigned int l=2) ;

  void displayRectangle(unsigned int i, unsigned int j,
			unsigned int width, unsigned int height,
			vpColor::vpColorType col, bool fill = false);
  void displayRectangle(const vpRect &rect,
			vpColor::vpColorType col, bool fill = false);
  void displayCharString(unsigned int i,unsigned int j,char *s,
			 vpColor::vpColorType c=vpColor::green) ;

  bool  getClick(unsigned int& i, unsigned int& j) ;
  bool  getClick(unsigned int& i, unsigned int& j,
		 vpMouseButton::vpMouseButtonType& button) ;
  void  getClick() ;
  bool  getClickUp(unsigned int& i, unsigned int& j,
		   vpMouseButton::vpMouseButtonType& button) ;

public:

  unsigned int getScreenDepth();
  void getScreenSize(unsigned int &width, unsigned int &height);
  void getImage(vpImage<vpRGBa> &I) ;
} ;

#endif
#endif
