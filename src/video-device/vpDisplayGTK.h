/****************************************************************************
 *
 * $Id: vpDisplayGTK.h,v 1.23 2008-12-03 10:25:11 nmelchio Exp $
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
 * Christophe Collewet
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpDisplayGTK_h
#define vpDisplayGTK_h

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_GTK) )

#include <visp/vpImage.h>
#include <visp/vpDisplay.h>

#include <gtk/gtk.h>
#include <gdk/gdkrgb.h>


/*!
  \file vpDisplayGTK.h
  \brief  Define the GTK console to display images
*/


/*!

  \class vpDisplayGTK

  \ingroup ImageGUI

  \brief The vpDisplayGTK allows to display image using the GTK+ library
  version 1.2.

  \author Christophe Collewet (Christophe.Collewet@irisa.fr),
  imported in ViSP by Eric Marchand (Eric.Marchand@irisa.fr)
  Irisa / Inria Rennes

  The GTK+ 1.2 library has to be available on the system.

  The example below shows how to display an image with this video device.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayGTK.h>

int main() 
{
#if defined(VISP_HAVE_GTK)
  vpImage<unsigned char> I; // Grey level image

  // Read an image in PGM P5 format
#ifdef UNIX
  vpImageIo::readPGM(I, "/local/soft/ViSP/ViSP-images/Klimt/Klimt.pgm");
#elif WIN32
  vpImageIo::readPGM(I, "C:/temp/ViSP-images/Klimt/Klimt.pgm");
#endif

  vpDisplayGTK d; 

  // Initialize the display with the image I. Display and image are
  // now link together.
  d.init(I);

  // Specify the window location
  vpDisplay::setWindowPosition(I, 400, 100);

  // Set the display window title
  vpDisplay::setTitle(I, "My GTK display");

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

class VISP_EXPORT vpDisplayGTK: public vpDisplay
{
private:
  //! true if GTK display is ready to use
  bool GTKinitialized ;
  GtkWidget *widget;
  GdkPixmap *background;
  GdkGC *gc;
  GdkColor blue,red,yellow,green,cyan,magenta,goldenrod,coral,orange,white, black;
  GdkFont *Police1,*Police2;
  guchar  *vectgtk;
  int windowXPosition ; int  windowYPosition ;
  GdkColor **col ;
  int ncol, nrow ;

public:
  vpDisplayGTK(vpImage<unsigned char> &I, int _winx=-1, int _winy=-1,
	       const char *title=NULL) ;
  vpDisplayGTK(vpImage<vpRGBa> &I, int _winx=-1, int _winy=-1,
	       const char *title=NULL) ;

  vpDisplayGTK(int _winx, int _winy, const char *title=NULL) ;

  vpDisplayGTK() ;
  virtual ~vpDisplayGTK() ;

  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    const char *_title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	    int winx=-1, int winy=-1,
	    const char *_title=NULL)  ;

  void init(unsigned int width, unsigned int height,
	    int winx=-1, int winy=-1 ,
	    const char *_title=NULL) ;

  unsigned int getScreenDepth();
  void getScreenSize(unsigned int &width, unsigned int &height);

  void getImage(vpImage<vpRGBa> &I) ;

protected:
  void setFont( const char *fontname );
  void setTitle(const char *string) ;
  void setWindowPosition(int winx, int winy);

  void clearDisplay(vpColor::vpColorType c=vpColor::white) ;

  void closeDisplay() ;

  void displayArrow(int i1, int j1, int i2, int j2,
		    vpColor::vpColorType col=vpColor::white,
		    unsigned int L=4, unsigned int l=2) ;
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

  bool getClick(bool blocking=true) ;
  bool getClick(vpImagePoint &ip, bool blocking=true) ;
  bool getClick(vpImagePoint &ip,
		vpMouseButton::vpMouseButtonType& button,
		bool blocking=true) ;
  bool getClickUp(vpImagePoint &ip,
		  vpMouseButton::vpMouseButtonType& button,
		  bool blocking=true) ;

  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }


} ;

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
