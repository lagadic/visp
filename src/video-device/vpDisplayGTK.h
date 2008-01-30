/****************************************************************************
 *
 * $Id: vpDisplayGTK.h,v 1.17 2008-01-30 15:32:26 fspindle Exp $
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

  \brief The  vpDisplayGTK allows to display image using the GTK+ library
  version 1.2.

  \author Christophe Collewet (Christophe.Collewet@irisa.fr),
  imported in ViSP by Eric Marchand (Eric.Marchand@irisa.fr)
  Irisa / Inria Rennes

  The GTK+ 1.2 library has to be available on the system


  \date December 2005
*/

class VISP_EXPORT vpDisplayGTK: public vpDisplay
{
private:
  //! true if GTK display is ready to use
  bool GTKinitialized ;
  GdkWindow *window;
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
	       char *title=NULL) ;
  vpDisplayGTK(vpImage<vpRGBa> &I, int _winx=-1, int _winy=-1,
	       char *title=NULL) ;

  vpDisplayGTK(int _winx, int _winy, char *title=NULL) ;

  vpDisplayGTK() ;
  virtual ~vpDisplayGTK() ;

  unsigned int getScreenDepth();
  void getScreenSize(unsigned int &width, unsigned int &height);

  void getImage(vpImage<vpRGBa> &I) ;
  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    char *_title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	    int winx=-1, int winy=-1,
	    char *_title=NULL)  ;

  void init(unsigned int width, unsigned int height,
	    int winx=-1, int winy=-1 ,
	    char *_title=NULL) ;
protected:
  void clearDisplay(vpColor::vpColorType c=vpColor::white) ;

  void closeDisplay() ;

  void displayArrow(int i1, int j1, int i2, int j2,
		    vpColor::vpColorType col=vpColor::white,
		    unsigned int L=4, unsigned int l=2) ;
  void displayCharString(int i, int j,char *s,
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
  void flushTitle(const char *string) ;
  bool getClick(unsigned int& i, unsigned int& j, bool blocking=true) ;
  bool getClick(unsigned int& i, unsigned int& j,
		 vpMouseButton::vpMouseButtonType& button,
		bool blocking=true) ;
  bool getClick(bool blocking=true) ;
  bool getClickUp(unsigned int& i, unsigned int& j,
		  vpMouseButton::vpMouseButtonType& button,
		  bool blocking=true) ;

  inline  unsigned int getWidth() const  { return width ; }
  inline  unsigned int getHeight() const { return height ; }

  void setWindowPosition(int /* winx */, int /* winy */) { ; }

} ;

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
