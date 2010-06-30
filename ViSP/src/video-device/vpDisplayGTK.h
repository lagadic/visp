/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
  \brief Define the GTK console to display images.
*/


/*!

  \class vpDisplayGTK

  \ingroup ImageGUI

  \brief The vpDisplayGTK allows to display image using the GTK+ library
  version 1.2.

  The GTK+ 1.2 library has to be available on the system.

  The example below shows how to display an image with this video device.
  \code
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpImagePoint.h>

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

  // Draw a red rectangle in the display overlay (foreground)
  vpImagePoint topLeftCorner;
  topLeftCorner.set_i(50);
  topLeftCorner.set_j(10);
  vpDisplay::displayRectangle(I, topLeftCorner, 100, 20, vpColor::green, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Get non blocking keyboard events
  std::cout << "Check keyboard events..." << std::endl; 
  char key[10];
  bool ret;
  for (int i=0; i< 200; i++) {
    bool ret = vpDisplay::getKeyboardEvent(I, key, false);
    if (ret) 
      std::cout << "keyboard event: key: " << "\"" << key << "\"" << std::endl;
    vpTime::wait(40);
  }

  // Get a blocking keyboard event
  std::cout << "Wait for a keyboard event..." << std::endl; 
  ret = vpDisplay::getKeyboardEvent(I, key, true);
  std::cout << "keyboard event: " << ret << std::endl;
  if (ret) 
    std::cout << "key: " << "\"" << key << "\"" << std::endl;
  
  // Wait for a click in the display window
  std::cout << "Wait for a button click..." << std::endl;
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
  GdkColor blue,red,yellow,green,cyan,orange,white, black, gdkcolor,
           lightBlue, darkBlue, lightRed, darkRed,lightGreen, darkGreen,
	   purple, lightGray, gray, darkGray;
  GdkColormap  *colormap;

  GdkFont *Police1,*Police2;
  guchar  *vectgtk;
  int windowXPosition ; int  windowYPosition ;
  GdkColor **col ;
  int ncol, nrow ;

  typedef enum {
    id_black=0,
    id_white,
    id_lightGray,
    id_gray,
    id_darkGray,
    id_lightRed,
    id_red,
    id_darkRed,
    id_lightGreen,
    id_green,
    id_darkGreen,
    id_lightBlue,
    id_blue,
    id_darkBlue,
    id_yellow,
    id_cyan,
    id_orange,
    id_purple,
    id_npredefined // Number of predefined colors
  } vpColorIdentifier;

public:
  vpDisplayGTK() ;
  vpDisplayGTK(int winx, int winy, const char *title=NULL) ;
  vpDisplayGTK(vpImage<unsigned char> &I, int winx=-1, int winy=-1,
	       const char *title=NULL) ;
  vpDisplayGTK(vpImage<vpRGBa> &I, int winx=-1, int winy=-1,
	       const char *title=NULL) ;

  virtual ~vpDisplayGTK() ;

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

  void clearDisplay(const vpColor &color=vpColor::white) ;

  void closeDisplay() ;

  void displayArrow(const vpImagePoint &ip1, 
		    const vpImagePoint &ip2,
		    const vpColor &color=vpColor::white,
		    unsigned int w=4,unsigned int h=2,
		    unsigned int thickness=1) ;
  void displayCharString(const vpImagePoint &ip, const char *text,
			 const vpColor &color=vpColor::green) ;

  void displayCircle(const vpImagePoint &center, unsigned int radius,
		     const vpColor &color,
		     bool fill = false,
		     unsigned int thickness=1);
  void displayCross(const vpImagePoint &ip, unsigned int size,
		    const vpColor &color, unsigned int thickness=1) ;
  void displayDotLine(const vpImagePoint &ip1, 
		      const vpImagePoint &ip2,
		      const vpColor &color, unsigned int thickness=1) ;

  void displayImage(const vpImage<vpRGBa> &I) ;
  void displayImage(const vpImage<unsigned char> &I) ;
  void displayImage(const unsigned char *I) ;

  void displayLine(const vpImagePoint &ip1, 
		   const vpImagePoint &ip2,
		   const vpColor &color, unsigned int thickness=1) ;

  void displayPoint(const vpImagePoint &ip, const vpColor &color) ;
  void displayRectangle(const vpImagePoint &topLeft,
			unsigned int width, unsigned int height,
			const vpColor &color, bool fill = false,
			unsigned int thickness=1) ;
  void displayRectangle(const vpImagePoint &topLeft,
			const vpImagePoint &bottomRight,
			const vpColor &color, bool fill = false,
			unsigned int thickness=1) ;
  void displayRectangle(const vpRect &rectangle,
			const vpColor &color, bool fill = false,
			unsigned int thickness=1) ;

  void flushDisplay() ;

  bool getClick(bool blocking=true) ;
  bool getClick(vpImagePoint &ip, bool blocking=true) ;
  bool getClick(vpImagePoint &ip,
		vpMouseButton::vpMouseButtonType& button,
		bool blocking=true) ;
  bool getClickUp(vpImagePoint &ip,
		  vpMouseButton::vpMouseButtonType& button,
		  bool blocking=true) ;
  bool getKeyboardEvent(bool blocking=true);
  bool getKeyboardEvent(char *string, bool blocking=true);
  bool getPointerMotionEvent (vpImagePoint &ip);
  bool getPointerPosition (vpImagePoint &ip);

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
