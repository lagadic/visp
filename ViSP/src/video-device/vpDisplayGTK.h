/****************************************************************************
 *
 * $Id: vpDisplayGTK.h,v 1.5 2007-02-26 17:26:58 fspindle Exp $
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


/*

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
  GdkColor blue,red,yellow,green,cyan,magenta,goldenrod,coral,orange,white;
  GdkFont *Police1,*Police2;
  guchar  *vectgtk;
  int windowXPosition ; int  windowYPosition ;
  GdkColor **col ;
  int ncol, nrow ;

protected:
  void setWindowPosition(int _winx, int _winy) { ; }
  inline  unsigned getWidth() const  { return width ; }
  inline  unsigned getHeight() const { return height ; }

public:
  void init(vpImage<unsigned char> &I,
	    int winx=-1, int winy=-1,
	    char *_title=NULL)  ;
  void init(vpImage<vpRGBa> &I,
	    int winx=-1, int winy=-1,
	    char *_title=NULL)  ;

  void init(unsigned width, unsigned height,
	    int winx=-1, int winy=-1 ,
	    char *_title=NULL) ;
  // only the constructor/destructor are public
public:
  vpDisplayGTK(vpImage<unsigned char> &I, int _winx=-1, int _winy=-1, 
	       char *title=NULL) ;
  vpDisplayGTK(vpImage<vpRGBa> &I, int _winx=-1, int _winy=-1,
	       char *title=NULL) ;

  vpDisplayGTK(int _winx, int _winy, char *title=NULL) ;

  vpDisplayGTK() ;
  ~vpDisplayGTK() ;

protected:
  void displayImage(const vpImage<vpRGBa> &I) ;
  void displayImage(const vpImage<unsigned char> &I) ;
  void displayImage(const unsigned char *I) ;

  void closeDisplay() ;
  void flushDisplay() ;
  void flushTitle(const char *string) ;

  void clearDisplay(vpColor::vpColorType c=vpColor::white) ;

  void displayPoint(unsigned x,unsigned y,vpColor::vpColorType col) ;
  void displayCross(unsigned x,unsigned y, unsigned size,
		    vpColor::vpColorType col) ;
  void displayCrossLarge(unsigned x,unsigned y, unsigned size,
			 vpColor::vpColorType col) ;
  void displayCircle(unsigned i, unsigned j, unsigned r, 
		     vpColor::vpColorType c);
  void displayLine(unsigned x1, unsigned y1, unsigned x2, unsigned y2,
		   vpColor::vpColorType col, unsigned e=1) ;
  void displayDotLine(unsigned x1, unsigned y1, unsigned x2, unsigned y2, 
		      vpColor::vpColorType col, unsigned e=1) ;


  void displayArrow(unsigned i1,unsigned j1, unsigned i2, unsigned j2,
		    vpColor::vpColorType col=vpColor::white, unsigned L=4,
		    unsigned l=2) ;

  void displayRectangle(unsigned i, unsigned j, 
			unsigned width, unsigned height, 
			vpColor::vpColorType col);
  void displayCharString(unsigned i,unsigned j,char *s, 
			 vpColor::vpColorType c=vpColor::green) ;

  bool  getClick(unsigned& i, unsigned& j) ;
  bool  getClick(unsigned& i, unsigned& j, 
		 vpMouseButton::vpMouseButtonType& button) ;
  void  getClick() ;
  bool  getClickUp(unsigned& i, unsigned& j, 
		   vpMouseButton::vpMouseButtonType& button) ;

public:

  unsigned  getScreenDepth();
  void getScreenSize(unsigned &width, unsigned &height);



  void   getImage(vpImage<vpRGBa> &I) ;
} ;

#endif
#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
