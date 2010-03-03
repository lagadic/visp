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
 * Fabien Spindler
 * Anthony Saunier
 *
 *****************************************************************************/

/*!
  \file vpDisplayX.cpp
  \brief Define the X11 console to display images.
*/


#include <visp/vpConfig.h>
#ifdef VISP_HAVE_X11

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// Display stuff
#include <visp/vpDisplay.h>
#include <visp/vpDisplayX.h>

//debug / exception
#include <visp/vpDebug.h>
#include <visp/vpDisplayException.h>

// math
#include <visp/vpMath.h>

/*!

  Constructor : initialize a display to visualize a gray level image
  (8 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
vpDisplayX::vpDisplayX ( vpImage<unsigned char> &I,
                         int x,
                         int y,
                         const char *title ) : vpDisplay()
{
  x_color = NULL;
  init ( I, x, y, title ) ;
}



/*!
  Constructor : initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : Image to be displayed (not that image has to be initialized).
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.
*/
vpDisplayX::vpDisplayX ( vpImage<vpRGBa> &I,
                         int x,
                         int y,
                         const char *title )
{
  x_color = NULL;
  title = NULL ;
  init ( I, x, y, title ) ;
}

/*!

  Constructor that just initialize the display position in the screen
  and the display title.

  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

  To initialize the display size, you need to call init().

  \code
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

int main()
{
  vpDisplayX d(100, 200, "My display");
  vpImage<unsigned char> I(240, 384);
  d.init(I);
}
  \endcode
*/
vpDisplayX::vpDisplayX ( int x, int y, const char *title )
{
  displayHasBeenInitialized = false ;
  windowXPosition = x ;
  windowYPosition = y ;

  this->x_color = NULL;
  this->title = NULL ;

  if ( title != NULL )
  {
    this->title = new char[strlen ( title ) + 1] ; // Modif Fabien le 19/04/02
    strcpy ( this->title, title ) ;
  }

  ximage_data_init = false;
}

/*!
  Basic constructor.

  To initialize the window position, title and size you may call
  init(vpImage<unsigned char> &, int, int, const char *) or
  init(vpImage<vpRGBa> &, int, int, const char *).

  \code
#include <visp/vpDisplayX.h>
#include <visp/vpImage.h>

int main()
{
  vpDisplayX d;
  vpImage<unsigned char> I(240, 384);
  d.init(I, 100, 200, "My display");
}
  \endcode
*/
vpDisplayX::vpDisplayX()
{
  x_color = NULL;
  displayHasBeenInitialized =false ;
  windowXPosition = windowYPosition = -1 ;

  title = NULL ;
  if ( title != NULL )
  {
    delete [] title ;
    title = NULL ;
  }
  title = new char[1] ;
  strcpy ( title,"" ) ;

  Xinitialise = false ;
  ximage_data_init = false;
}

/*!
  Destructor.
*/
vpDisplayX::~vpDisplayX()
{
  closeDisplay() ;
}

/*!
  Initialize the display (size, position and title) of a gray level image.

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void
vpDisplayX::init ( vpImage<unsigned char> &I, int x, int y, const char *title )
{

  displayHasBeenInitialized =true ;

  if (x_color == NULL) {
    // id_unknown = number of predefined colors
    x_color= new unsigned long [vpColor::id_unknown]; 
  }

  XSizeHints  hints;
  windowXPosition = x ;
  windowYPosition = y ;
  {
    if ( this->title != NULL )
    {
      //   vpTRACE(" ") ;
      delete [] this->title ;
      this->title = NULL ;
    }

    if ( title != NULL )
    {
      this->title = new char[strlen ( title ) + 1] ;
      strcpy ( this->title, title ) ;
    }
  }

  // Positionnement de la fenetre dans l'�cran.
  if ( ( windowXPosition < 0 ) || ( windowYPosition < 0 ) )
  {
    hints.flags = 0;
  }
  else
  {
    hints.flags = USPosition;
    hints.x = windowXPosition;
    hints.y = windowYPosition;
  }


  // setup X11 --------------------------------------------------
  width  = I.getWidth();
  height = I.getHeight();
  display = XOpenDisplay ( NULL );
  if ( display == NULL )
  {
    vpERROR_TRACE ( "Can't connect display on server %s.\n", XDisplayName ( NULL ) );
    throw ( vpDisplayException ( vpDisplayException::connexionError,
                                 "Can't connect display on server." ) ) ;
  }

  screen       = DefaultScreen ( display );
  lut          = DefaultColormap ( display, screen );
  screen_depth = DefaultDepth ( display, screen );

  if ( ( window =
              XCreateSimpleWindow ( display, RootWindow ( display, screen ),
                                    windowXPosition, windowYPosition, width, height, 1,
                                    BlackPixel ( display, screen ),
                                    WhitePixel ( display, screen ) ) ) == 0 )
  {
    vpERROR_TRACE ( "Can't create window." );
    throw ( vpDisplayException ( vpDisplayException::cannotOpenWindowError,
                                 "Can't create window." ) ) ;
  }

  //
  // Create color table for 8 and 16 bits screen
  //
  if ( screen_depth == 8 )
  {
    lut = XCreateColormap ( display, window,
                            DefaultVisual ( display, screen ), AllocAll ) ;
    xcolor.flags = DoRed | DoGreen | DoBlue ;

    for ( unsigned int i = 0 ; i < 256 ; i++ )
    {
      xcolor.pixel = i ;
      xcolor.red = 256 * i;
      xcolor.green = 256 * i;
      xcolor.blue = 256 * i;
      XStoreColor ( display, lut, &xcolor );
    }

    XSetWindowColormap ( display, window, lut ) ;
    XInstallColormap ( display, lut ) ;
  }

  else if ( screen_depth == 16 )
  {
    for ( unsigned int i = 0; i < 256; i ++ )
    {
      xcolor.pad = 0;
      xcolor.red = xcolor.green = xcolor.blue = 256 * i;
      if ( XAllocColor ( display, lut, &xcolor ) == 0 )
      {
        vpERROR_TRACE ( "Can't allocate 256 colors. Only %d allocated.", i );
        throw ( vpDisplayException ( vpDisplayException::colorAllocError,
                                     "Can't allocate 256 colors." ) ) ;
      }
      colortable[i] = xcolor.pixel;
    }

    XSetWindowColormap ( display, window, lut ) ;
    XInstallColormap ( display, lut ) ;

  }

  //
  // Create colors for overlay
  //
  switch ( screen_depth )
  {
    case 8:
      // Color BLACK and WHITE are set properly by default.
      
      // Color LIGHT GRAY.
      x_color[vpColor::id_lightGray] = 254;
      xcolor.pixel  = x_color[vpColor::id_lightGray] ;
      xcolor.red    = 256 * 192;
      xcolor.green  = 256 * 192;
      xcolor.blue   = 256 * 192;
      XStoreColor ( display, lut, &xcolor );

      // Color GRAY.
      x_color[vpColor::id_gray] = 253;
      xcolor.pixel  = x_color[vpColor::id_gray] ;
      xcolor.red    = 256 * 128;
      xcolor.green  = 256 * 128;
      xcolor.blue   = 256 * 128;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK GRAY.
      x_color[vpColor::id_darkGray] = 252;
      xcolor.pixel  = x_color[vpColor::id_darkGray] ;
      xcolor.red    = 256 * 64;
      xcolor.green  = 256 * 64;
      xcolor.blue   = 256 * 64;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT RED.
      x_color[vpColor::id_lightRed] = 251;
      xcolor.pixel  = x_color[vpColor::id_lightRed] ;
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 140;
      XStoreColor ( display, lut, &xcolor );

      // Color RED.
      x_color[vpColor::id_red] = 250;
      xcolor.pixel  = x_color[vpColor::id_red] ;
      xcolor.red    = 256 * 255;
      xcolor.green  = 0;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK RED.
      x_color[vpColor::id_darkRed] = 249;
      xcolor.pixel  = x_color[vpColor::id_darkRed] ;
      xcolor.red    = 256 * 128;
      xcolor.green  = 0;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT GREEN.
      x_color[vpColor::id_lightGreen] = 248;
      xcolor.pixel  = x_color[vpColor::id_lightGreen] ;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 256 * 140;
      XStoreColor ( display, lut, &xcolor );

      // Color GREEN.
      x_color[vpColor::id_green] = 247;
      xcolor.pixel  = x_color[vpColor::id_green];
      xcolor.red    = 0;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK GREEN.
      x_color[vpColor::id_darkGreen] = 246;
      xcolor.pixel  = x_color[vpColor::id_darkGreen] ;
      xcolor.red    = 0;
      xcolor.green  = 256 * 128;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT BLUE.
      x_color[vpColor::id_lightBlue] = 245;
      xcolor.pixel  = x_color[vpColor::id_lightBlue] ;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 255;
      XStoreColor ( display, lut, &xcolor );

      // Color BLUE.
      x_color[vpColor::id_blue] = 244;
      xcolor.pixel  = x_color[vpColor::id_blue];
      xcolor.red    = 0;
      xcolor.green  = 0;
      xcolor.blue   = 256 * 255;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK BLUE.
      x_color[vpColor::id_darkBlue] = 243;
      xcolor.pixel  = x_color[vpColor::id_darkBlue] ;
      xcolor.red    = 0;
      xcolor.green  = 0;
      xcolor.blue   = 256 * 128;
      XStoreColor ( display, lut, &xcolor );

      // Color YELLOW.
      x_color[vpColor::id_yellow] = 242;
      xcolor.pixel  = x_color[vpColor::id_yellow];
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );

      // Color ORANGE.
      x_color[vpColor::id_orange] = 241;
      xcolor.pixel  = x_color[vpColor::id_orange];
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 165;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );

      // Color CYAN.
      x_color[vpColor::id_cyan] = 240;
      xcolor.pixel  = x_color[vpColor::id_cyan];
      xcolor.red    = 0;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 256 * 255;
      XStoreColor ( display, lut, &xcolor );
      
      // Color PURPLE.
      x_color[vpColor::id_purple] = 239;
      xcolor.pixel  = x_color[vpColor::id_purple];
      xcolor.red    = 256 * 128;
      xcolor.green  = 0;
      xcolor.blue   = 256 * 128;
      XStoreColor ( display, lut, &xcolor );

      break;

    case 16:
    case 24:
    {
      xcolor.flags = DoRed | DoGreen | DoBlue ;

      // Couleur BLACK.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 0;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_black] = xcolor.pixel;

      // Couleur WHITE.
      xcolor.pad   = 0;
      xcolor.red   = 256* 255;
      xcolor.green = 256* 255;
      xcolor.blue  = 256* 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_white] = xcolor.pixel;
      
      // Couleur LIGHT GRAY.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 192;
      xcolor.green  = 256 * 192;
      xcolor.blue   = 256 * 192;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightGray] = xcolor.pixel;

      // Couleur GRAY.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 128;
      xcolor.green  = 256 * 128;
      xcolor.blue   = 256 * 128;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_gray] = xcolor.pixel;
      
      // Couleur DARK GRAY.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 64;
      xcolor.green  = 256 * 64;
      xcolor.blue   = 256 * 64;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkGray] = xcolor.pixel;
      
      // Couleur LIGHT RED.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 140;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightRed] = xcolor.pixel;

      // Couleur RED.
      xcolor.pad   = 0;
      xcolor.red   = 256* 255;
      xcolor.green = 0;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_red] = xcolor.pixel;
      
      // Couleur DARK RED.
      xcolor.pad   = 0;
      xcolor.red   = 256* 128;
      xcolor.green = 0;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkRed] = xcolor.pixel;
      
      // Couleur LIGHT GREEN.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 256 * 140;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightGreen] = xcolor.pixel;

      // Couleur GREEN.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 256*255;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_green] = xcolor.pixel;
      
      // Couleur DARK GREEN.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 256* 128;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkGreen] = xcolor.pixel;
      
      // Couleur LIGHT Blue.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightBlue] = xcolor.pixel;

      // Couleur BLUE.
      xcolor.pad = 0;
      xcolor.red = 0;
      xcolor.green = 0;
      xcolor.blue  = 256* 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_blue] = xcolor.pixel;
      
      // Couleur DARK BLUE.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 0;
      xcolor.blue  = 256* 128;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkBlue] = xcolor.pixel;

      // Couleur YELLOW.
      xcolor.pad = 0;
      xcolor.red = 256 * 255;
      xcolor.green = 256 * 255;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_yellow] = xcolor.pixel;

      // Couleur ORANGE.
      xcolor.pad = 0;
      xcolor.red = 256 * 255;
      xcolor.green = 256 * 165;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_orange] = xcolor.pixel;

      // Couleur CYAN.
      xcolor.pad = 0;
      xcolor.red = 0;
      xcolor.green = 256 * 255;
      xcolor.blue  = 256 * 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_cyan] = xcolor.pixel;
      
      // Couleur PURPLE.
      xcolor.pad = 0;
      xcolor.red = 256 * 128;
      xcolor.green = 0;
      xcolor.blue  = 256 * 128;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_purple] = xcolor.pixel;
      break;
    }
  }

  XSetStandardProperties ( display, window, title, title, None, 0, 0, &hints );
  XMapWindow ( display, window ) ;
  // Selection des evenements.
  XSelectInput ( display, window,
                 ExposureMask | 
		 ButtonPressMask | ButtonReleaseMask | 
		 KeyPressMask | KeyReleaseMask | 
		 StructureNotifyMask |
		 PointerMotionMask);

  // graphic context creation
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel ( display, screen );
  values.background = BlackPixel ( display, screen );
  context = XCreateGC ( display, window,
                        GCPlaneMask  | GCFillStyle | GCForeground | GCBackground,
                        &values );

  if ( context == NULL )
  {
    vpERROR_TRACE ( "Can't create graphics context." );
    throw ( vpDisplayException ( vpDisplayException::XWindowsError,
                                 "Can't create graphics context" ) ) ;

  }

  // Pixmap creation.
  pixmap = XCreatePixmap ( display, window, width, height, screen_depth );

  do
    XNextEvent ( display, &event );
  while ( event.xany.type != Expose );

  {
    Ximage = XCreateImage ( display, DefaultVisual ( display, screen ),
                            screen_depth, ZPixmap, 0, NULL,
                            I.getWidth() , I.getHeight(), XBitmapPad ( display ), 0 );

    Ximage->data = ( char * ) malloc ( I.getWidth() * I.getHeight() * Ximage->bits_per_pixel / 8 );
    ximage_data_init = true;

  }
  Xinitialise = true ;
  setTitle ( title ) ;
  XSync ( display, 1 );

  I.display = this ;
}

/*!  
  Initialize the display (size, position and title) of a color
  image in RGBa format.

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void
vpDisplayX::init ( vpImage<vpRGBa> &I, int x, int y, const char *title )
{

  displayHasBeenInitialized =true ;

  XSizeHints  hints;
  windowXPosition = x ;
  windowYPosition = y ;

  if (x_color == NULL) {
    // id_unknown = number of predefined colors
    x_color= new unsigned long [vpColor::id_unknown]; 
  }

  {
    if ( this->title != NULL )
    {
      delete [] this->title ;
      this->title = NULL ;
    }

    if ( title != NULL )
    {
      this->title = new char[strlen ( title ) + 1] ; // Modif Fabien le 19/04/02
      strcpy ( this->title, title ) ;
    }
  }

  // Positionnement de la fenetre dans l'�cran.
  if ( ( windowXPosition < 0 ) || ( windowYPosition < 0 ) )
  {
    hints.flags = 0;
  }
  else
  {
    hints.flags = USPosition;
    hints.x = windowXPosition;
    hints.y = windowYPosition;
  }


  // setup X11 --------------------------------------------------
  width = I.getWidth();
  height = I.getHeight();

  if ( ( display = XOpenDisplay ( NULL ) ) == NULL )
  {
    vpERROR_TRACE ( "Can't connect display on server %s.\n", XDisplayName ( NULL ) );
    throw ( vpDisplayException ( vpDisplayException::connexionError,
                                 "Can't connect display on server." ) ) ;
  }

  screen       = DefaultScreen ( display );
  lut          = DefaultColormap ( display, screen );
  screen_depth = DefaultDepth ( display, screen );

  vpDEBUG_TRACE ( 1, "Screen depth: %d\n", screen_depth );

  if ( ( window = XCreateSimpleWindow ( display, RootWindow ( display, screen ),
                                        windowXPosition, windowYPosition,
                                        width, height, 1,
                                        BlackPixel ( display, screen ),
                                        WhitePixel ( display, screen ) ) ) == 0 )
  {
    vpERROR_TRACE ( "Can't create window." );
    throw ( vpDisplayException ( vpDisplayException::cannotOpenWindowError,
                                 "Can't create window." ) ) ;
  }

  //
  // Create color table for 8 and 16 bits screen
  //
  if ( screen_depth == 8 )
  {
    lut = XCreateColormap ( display, window,
                            DefaultVisual ( display, screen ), AllocAll ) ;
    xcolor.flags = DoRed | DoGreen | DoBlue ;

    for ( unsigned int i = 0 ; i < 256 ; i++ )
    {
      xcolor.pixel = i ;
      xcolor.red = 256 * i;
      xcolor.green = 256 * i;
      xcolor.blue = 256 * i;
      XStoreColor ( display, lut, &xcolor );
    }

    XSetWindowColormap ( display, window, lut ) ;
    XInstallColormap ( display, lut ) ;
  }

  else if ( screen_depth == 16 )
  {
    for ( unsigned int i = 0; i < 256; i ++ )
    {
      xcolor.pad = 0;
      xcolor.red = xcolor.green = xcolor.blue = 256 * i;
      if ( XAllocColor ( display, lut, &xcolor ) == 0 )
      {
        vpERROR_TRACE ( "Can't allocate 256 colors. Only %d allocated.", i );
        throw ( vpDisplayException ( vpDisplayException::colorAllocError,
                                     "Can't allocate 256 colors." ) ) ;
      }
      colortable[i] = xcolor.pixel;
    }

    XSetWindowColormap ( display, window, lut ) ;
    XInstallColormap ( display, lut ) ;

  }


  //
  // Create colors for overlay
  //
  switch ( screen_depth )
  {

    case 8:
      // Color BLACK and WHITE are set properly.
      
      // Color LIGHT GRAY.
      x_color[vpColor::id_lightGray] = 254;
      xcolor.pixel  = x_color[vpColor::id_lightGray] ;
      xcolor.red    = 256 * 192;
      xcolor.green  = 256 * 192;
      xcolor.blue   = 256 * 192;
      XStoreColor ( display, lut, &xcolor );

      // Color GRAY.
      x_color[vpColor::id_gray] = 253;
      xcolor.pixel  = x_color[vpColor::id_gray] ;
      xcolor.red    = 256 * 128;
      xcolor.green  = 256 * 128;
      xcolor.blue   = 256 * 128;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK GRAY.
      x_color[vpColor::id_darkGray] = 252;
      xcolor.pixel  = x_color[vpColor::id_darkGray] ;
      xcolor.red    = 256 * 64;
      xcolor.green  = 256 * 64;
      xcolor.blue   = 256 * 64;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT RED.
      x_color[vpColor::id_lightRed] = 251;
      xcolor.pixel  = x_color[vpColor::id_lightRed] ;
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 140;
      XStoreColor ( display, lut, &xcolor );

      // Color RED.
      x_color[vpColor::id_red] = 250;
      xcolor.pixel  = x_color[vpColor::id_red] ;
      xcolor.red    = 256 * 255;
      xcolor.green  = 0;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK RED.
      x_color[vpColor::id_darkRed] = 249;
      xcolor.pixel  = x_color[vpColor::id_darkRed] ;
      xcolor.red    = 256 * 128;
      xcolor.green  = 0;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT GREEN.
      x_color[vpColor::id_lightGreen] = 248;
      xcolor.pixel  = x_color[vpColor::id_lightGreen] ;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 256 * 140;
      XStoreColor ( display, lut, &xcolor );

      // Color GREEN.
      x_color[vpColor::id_green] = 247;
      xcolor.pixel  = x_color[vpColor::id_green];
      xcolor.red    = 0;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK GREEN.
      x_color[vpColor::id_darkGreen] = 246;
      xcolor.pixel  = x_color[vpColor::id_darkGreen] ;
      xcolor.red    = 0;
      xcolor.green  = 256 * 128;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT BLUE.
      x_color[vpColor::id_lightBlue] = 245;
      xcolor.pixel  = x_color[vpColor::id_lightBlue] ;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 255;
      XStoreColor ( display, lut, &xcolor );

      // Color BLUE.
      x_color[vpColor::id_blue] = 244;
      xcolor.pixel  = x_color[vpColor::id_blue];
      xcolor.red    = 0;
      xcolor.green  = 0;
      xcolor.blue   = 256 * 255;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK BLUE.
      x_color[vpColor::id_darkBlue] = 243;
      xcolor.pixel  = x_color[vpColor::id_darkBlue] ;
      xcolor.red    = 0;
      xcolor.green  = 0;
      xcolor.blue   = 256 * 128;
      XStoreColor ( display, lut, &xcolor );

      // Color YELLOW.
      x_color[vpColor::id_yellow] = 242;
      xcolor.pixel  = x_color[vpColor::id_yellow];
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );

      // Color ORANGE.
      x_color[vpColor::id_orange] = 241;
      xcolor.pixel  = x_color[vpColor::id_orange];
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 165;
      xcolor.blue   = 0;
      XStoreColor ( display, lut, &xcolor );

      // Color CYAN.
      x_color[vpColor::id_cyan] = 240;
      xcolor.pixel  = x_color[vpColor::id_cyan];
      xcolor.red    = 0;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 256 * 255;
      XStoreColor ( display, lut, &xcolor );
      
      // Color PURPLE.
      x_color[vpColor::id_purple] = 239;
      xcolor.pixel  = x_color[vpColor::id_purple];
      xcolor.red    = 256 * 128;
      xcolor.green  = 0;
      xcolor.blue   = 256 * 128;
      XStoreColor ( display, lut, &xcolor );

      break;

    case 16:
    case 24:
    {
      xcolor.flags = DoRed | DoGreen | DoBlue ;
      
      // Couleur BLACK.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 0;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_black] = xcolor.pixel;

      // Couleur WHITE.
      xcolor.pad   = 0;
      xcolor.red   = 256* 255;
      xcolor.green = 256* 255;
      xcolor.blue  = 256* 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_white] = xcolor.pixel;
      
      // Couleur LIGHT GRAY.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 192;
      xcolor.green  = 256 * 192;
      xcolor.blue   = 256 * 192;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightGray] = xcolor.pixel;

      // Couleur GRAY.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 128;
      xcolor.green  = 256 * 128;
      xcolor.blue   = 256 * 128;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_gray] = xcolor.pixel;
      
      // Couleur DARK GRAY.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 64;
      xcolor.green  = 256 * 64;
      xcolor.blue   = 256 * 64;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkGray] = xcolor.pixel;
      
      // Couleur LIGHT RED.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 255;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 140;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightRed] = xcolor.pixel;

      // Couleur RED.
      xcolor.pad   = 0;
      xcolor.red   = 256* 255;
      xcolor.green = 0;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_red] = xcolor.pixel;
      
      // Couleur DARK RED.
      xcolor.pad   = 0;
      xcolor.red   = 256* 128;
      xcolor.green = 0;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkRed] = xcolor.pixel;
      
      // Couleur LIGHT GREEN.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 255;
      xcolor.blue   = 256 * 140;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightGreen] = xcolor.pixel;

      // Couleur GREEN.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 256*255;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_green] = xcolor.pixel;
      
      // Couleur DARK GREEN.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 256* 128;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkGreen] = xcolor.pixel;
      
      // Couleur LIGHT Blue.
      xcolor.pad   = 0;
      xcolor.red    = 256 * 140;
      xcolor.green  = 256 * 140;
      xcolor.blue   = 256 * 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightBlue] = xcolor.pixel;

      // Couleur BLUE.
      xcolor.pad = 0;
      xcolor.red = 0;
      xcolor.green = 0;
      xcolor.blue  = 256* 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_blue] = xcolor.pixel;
      
      // Couleur DARK BLUE.
      xcolor.pad   = 0;
      xcolor.red   = 0;
      xcolor.green = 0;
      xcolor.blue  = 256* 128;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkBlue] = xcolor.pixel;

      // Couleur YELLOW.
      xcolor.pad = 0;
      xcolor.red = 256 * 255;
      xcolor.green = 256 * 255;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_yellow] = xcolor.pixel;

      // Couleur ORANGE.
      xcolor.pad = 0;
      xcolor.red = 256 * 255;
      xcolor.green = 256 * 165;
      xcolor.blue  = 0;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_orange] = xcolor.pixel;

      // Couleur CYAN.
      xcolor.pad = 0;
      xcolor.red = 0;
      xcolor.green = 256 * 255;
      xcolor.blue  = 256 * 255;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_cyan] = xcolor.pixel;
      
      // Couleur PURPLE.
      xcolor.pad = 0;
      xcolor.red = 256 * 128;
      xcolor.green = 0;
      xcolor.blue  = 256 * 128;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_purple] = xcolor.pixel;
      break;
    }
  }

  XSetStandardProperties ( display, window, title, title, None, 0, 0, &hints );
  XMapWindow ( display, window ) ;
  // Selection des evenements.
  XSelectInput ( display, window,
                 ExposureMask | 
		 ButtonPressMask | ButtonReleaseMask |
 		 KeyPressMask | KeyReleaseMask | 
		 StructureNotifyMask |
		 PointerMotionMask);

  // Creation du contexte graphique
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel ( display, screen );
  values.background = BlackPixel ( display, screen );
  context = XCreateGC ( display, window,
                        GCPlaneMask  | GCFillStyle | GCForeground | GCBackground,
                        &values );

  if ( context == NULL )
  {
    vpERROR_TRACE ( "Can't create graphics context." );
    throw ( vpDisplayException ( vpDisplayException::XWindowsError,
                                 "Can't create graphics context" ) ) ;
  }

  // Pixmap creation.
  pixmap = XCreatePixmap ( display, window, width, height, screen_depth );

  do
    XNextEvent ( display, &event );
  while ( event.xany.type != Expose );


  {
    Ximage = XCreateImage ( display, DefaultVisual ( display, screen ),
                            screen_depth, ZPixmap, 0, NULL,
                            I.getWidth() , I.getHeight(), XBitmapPad ( display ), 0 );


    Ximage->data = ( char * ) malloc ( I.getWidth() * I.getHeight()
                                       * Ximage->bits_per_pixel / 8 );
    ximage_data_init = true;

  }
  Xinitialise = true ;

  XSync ( display, true );
  setTitle ( title ) ;

  I.display = this ;
}


/*!
  Initialize the display size, position and title.

  \param width, height : Width and height of the window.
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.
*/
void vpDisplayX::init ( unsigned int width, unsigned int height,
                        int x, int y, const char *title )
{

  displayHasBeenInitialized = true ;


  /* setup X11 ------------------------------------------------------------- */
  this->width  = width;
  this->height = height;

  XSizeHints  hints;

  windowXPosition = x ;
  windowYPosition = y ;
  // Positionnement de la fenetre dans l'�cran.
  if ( ( windowXPosition < 0 ) || ( windowYPosition < 0 ) )
  {
    hints.flags = 0;
  }
  else
  {
    hints.flags = USPosition;
    hints.x = windowXPosition;
    hints.y = windowYPosition;
  }


  {
    if ( this->title != NULL )
    {
      delete [] this->title ;
      this->title = NULL ;
    }

    if ( title != NULL )
    {
      this->title = new char[strlen ( title ) + 1] ; // Modif Fabien le 19/04/02
      strcpy ( this->title, title ) ;
    }
  }


  if ( ( display = XOpenDisplay ( NULL ) ) == NULL )
  {
    vpERROR_TRACE ( "Can't connect display on server %s.\n", XDisplayName ( NULL ) );
    throw ( vpDisplayException ( vpDisplayException::connexionError,
                                 "Can't connect display on server." ) ) ;
  }

  screen       = DefaultScreen ( display );
  lut          = DefaultColormap ( display, screen );
  screen_depth = DefaultDepth ( display, screen );

  vpTRACE ( "Screen depth: %d\n", screen_depth );

  if ( ( window = XCreateSimpleWindow ( display, RootWindow ( display, screen ),
                                        windowXPosition, windowYPosition,
                                        width, height, 1,
                                        BlackPixel ( display, screen ),
                                        WhitePixel ( display, screen ) ) ) == 0 )
  {
    vpERROR_TRACE ( "Can't create window." );
    throw ( vpDisplayException ( vpDisplayException::cannotOpenWindowError,
                                 "Can't create window." ) ) ;
  }


  //
  // Create color table for 8 and 16 bits screen
  //
  if ( screen_depth == 8 )
  {
    lut = XCreateColormap ( display, window,
                            DefaultVisual ( display, screen ), AllocAll ) ;
    xcolor.flags = DoRed | DoGreen | DoBlue ;

    for ( unsigned int i = 0 ; i < 256 ; i++ )
    {
      xcolor.pixel = i ;
      xcolor.red = 256 * i;
      xcolor.green = 256 * i;
      xcolor.blue = 256 * i;
      XStoreColor ( display, lut, &xcolor );
    }

    XSetWindowColormap ( display, window, lut ) ;
    XInstallColormap ( display, lut ) ;
  }

  else if ( screen_depth == 16 )
  {
    for ( unsigned int i = 0; i < 256; i ++ )
    {
      xcolor.pad = 0;
      xcolor.red = xcolor.green = xcolor.blue = 256 * i;
      if ( XAllocColor ( display, lut, &xcolor ) == 0 )
      {
        vpERROR_TRACE ( "Can't allocate 256 colors. Only %d allocated.", i );
        throw ( vpDisplayException ( vpDisplayException::colorAllocError,
                                     "Can't allocate 256 colors." ) ) ;
      }
      colortable[i] = xcolor.pixel;
    }

    XSetWindowColormap ( display, window, lut ) ;
    XInstallColormap ( display, lut ) ;

  }

  vpColor pcolor; // predefined colors

  //
  // Create colors for overlay
  //
  switch ( screen_depth )
  {

    case 8:
      // Color BLACK: default set to 0

      // Color WHITE: default set to 255
      
      // Color LIGHT GRAY.
      pcolor = vpColor::lightGray;
      xcolor.pixel  = 254 ; // affected to 254
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
 
      // Color GRAY.
      pcolor = vpColor::gray;
      xcolor.pixel  = 253 ; // affected to 253
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK GRAY.
      pcolor = vpColor::darkGray;
      xcolor.pixel  = 252 ; // affected to 252
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT RED.
      pcolor = vpColor::lightRed;
      xcolor.pixel  = 251 ; // affected to 251
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
 
      // Color RED.
      pcolor = vpColor::red;
      xcolor.pixel  = 250 ; // affected to 250
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK RED.
      pcolor = vpColor::darkRed;
      xcolor.pixel  = 249 ; // affected to 249
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT GREEN.
      pcolor = vpColor::lightGreen;
      xcolor.pixel  = 248 ; // affected to 248
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );

      // Color GREEN.
      pcolor = vpColor::green;
      xcolor.pixel  = 247; // affected to 247
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK GREEN.
      pcolor = vpColor::darkGreen;
      xcolor.pixel  = 246 ; // affected to 246
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color LIGHT BLUE.
      pcolor = vpColor::lightBlue;
      xcolor.pixel  = 245 ; // affected to 245
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );

      // Color BLUE.
      pcolor = vpColor::blue;
      xcolor.pixel  = 244; // affected to 244
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color DARK BLUE.
      pcolor = vpColor::darkBlue;
      xcolor.pixel  = 243 ; // affected to 243
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );

      // Color YELLOW.
      pcolor = vpColor::yellow;
      xcolor.pixel  = 242; // affected to 242
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );

      // Color ORANGE.
      pcolor = vpColor::orange;
      xcolor.pixel  = 241; // affected to 241
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );

      // Color CYAN.
      pcolor = vpColor::cyan;
      xcolor.pixel  = 240; // affected to 240
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );
      
      // Color PURPLE.
      pcolor = vpColor::purple;
      xcolor.pixel  = 239; // affected to 239
      xcolor.red    = 256 * pcolor.R;
      xcolor.green  = 256 * pcolor.G;
      xcolor.blue   = 256 * pcolor.B;
      XStoreColor ( display, lut, &xcolor );

      break;

    case 16:
    case 24:
    {
      xcolor.flags = DoRed | DoGreen | DoBlue ;

      // Couleur BLACK.
      pcolor = vpColor::black;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_black] = xcolor.pixel;

      // Color WHITE.
      pcolor = vpColor::white;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_white] = xcolor.pixel;
      
      // Color LIGHT GRAY.
      pcolor = vpColor::lightGray;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightGray] = xcolor.pixel;

      // Color GRAY.
      pcolor = vpColor::gray;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_gray] = xcolor.pixel;
      
      // Color DARK GRAY.
      pcolor = vpColor::darkGray;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkGray] = xcolor.pixel;
      
      // Color LIGHT RED.
      pcolor = vpColor::lightRed;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightRed] = xcolor.pixel;

      // Color RED.
      pcolor = vpColor::red;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_red] = xcolor.pixel;
      
      // Color DARK RED.
      pcolor = vpColor::darkRed;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkRed] = xcolor.pixel;
      
      // Color LIGHT GREEN.
      pcolor = vpColor::lightGreen;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightGreen] = xcolor.pixel;

      // Color GREEN.
      pcolor = vpColor::green;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_green] = xcolor.pixel;
      
      // Color DARK GREEN.
      pcolor = vpColor::darkGreen;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkGreen] = xcolor.pixel;
      
      // Color LIGHT BLUE.
      pcolor = vpColor::lightBlue;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_lightBlue] = xcolor.pixel;

      // Color BLUE.
      pcolor = vpColor::blue;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_blue] = xcolor.pixel;
      
      // Color DARK BLUE.
      pcolor = vpColor::darkBlue;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_darkBlue] = xcolor.pixel;

      // Color YELLOW.     
      pcolor = vpColor::yellow;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_yellow] = xcolor.pixel;

      // Color ORANGE.
      pcolor = vpColor::orange;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_orange] = xcolor.pixel;

      // Color CYAN.
      pcolor = vpColor::cyan;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_cyan] = xcolor.pixel;
      
      // Color PURPLE.
      pcolor = vpColor::purple;
      xcolor.pad   = 0;
      xcolor.red   = 256 * pcolor.R;
      xcolor.green = 256 * pcolor.G;
      xcolor.blue  = 256 * pcolor.B;
      XAllocColor ( display, lut, &xcolor );
      x_color[vpColor::id_purple] = xcolor.pixel;
      break;
    }
  }


  XSetStandardProperties ( display, window, title, title, None, 0, 0, &hints );
  XMapWindow ( display, window ) ;
  // Selection des evenements.
  XSelectInput ( display, window,
                 ExposureMask | 
		 ButtonPressMask | ButtonReleaseMask | 
 		 KeyPressMask | KeyReleaseMask | 
		 StructureNotifyMask |
		 PointerMotionMask);

  /* Creation du contexte graphique */
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel ( display, screen );
  values.background = BlackPixel ( display, screen );
  context = XCreateGC ( display, window,
                        GCPlaneMask  | GCFillStyle | GCForeground | GCBackground,
                        &values );

  if ( context == NULL )
  {
    vpERROR_TRACE ( "Can't create graphics context." );
    throw ( vpDisplayException ( vpDisplayException::XWindowsError,
                                 "Can't create graphics context" ) ) ;
  }

  // Pixmap creation.
  pixmap = XCreatePixmap ( display, window, width, height, screen_depth );

  do
    XNextEvent ( display, &event );
  while ( event.xany.type != Expose );

  {
    Ximage = XCreateImage ( display, DefaultVisual ( display, screen ),
                            screen_depth, ZPixmap, 0, NULL,
                            width, height, XBitmapPad ( display ), 0 );

    Ximage->data = ( char * ) malloc ( width * height
                                       * Ximage->bits_per_pixel / 8 );
    ximage_data_init = true;
  }
  Xinitialise = true ;

  XSync ( display, true );
  setTitle ( title ) ;

}

/*!  

  Set the font used to display a text in overlay. The display is
  performed using displayCharString().

  \param font : The expected font name. The available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \note Under UNIX, to know all the available fonts, use the
  "xlsfonts" binary in a terminal. You can also use the "xfontsel" binary.

  \sa displayCharString()
*/
void vpDisplayX::setFont( const char* font )
{
  if ( Xinitialise )
  {
	if (font!=NULL)
	{
		try
		{
			Font stringfont;
			stringfont = XLoadFont (display, font) ; //"-adobe-times-bold-r-normal--18*");
			XSetFont (display, context, stringfont);
		}
		catch(...)
		{
			vpERROR_TRACE ( "Bad font " ) ;
			throw ( vpDisplayException ( vpDisplayException::notInitializedError,"Bad font" ) ) ;
		}
	}	
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
																"X not initialized" ) ) ;
  }
}

/*!
  Set the window title.
  \param title : Window title.
*/
void
vpDisplayX::setTitle ( const char *title )
{
  if ( Xinitialise )
  {
    XStoreName ( display, window, title );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Set the window position in the screen.

  \param winx, winy : Position of the upper-left window's border in the screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void vpDisplayX::setWindowPosition(int winx, int winy)
{
  if ( Xinitialise ) {
    XMoveWindow(display, window, winx, winy);
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImage ( const vpImage<unsigned char> &I )
{

  if ( Xinitialise )
  {
    switch ( screen_depth )
    {
      case 8:
      {
        unsigned char       *src_8  = NULL;
        unsigned char       *dst_8  = NULL;
        src_8 = ( unsigned char * ) I.bitmap;
        dst_8 = ( unsigned char * ) Ximage->data;
        // Correction de l'image de facon a liberer les niveaux de gris
        // ROUGE, VERT, BLEU, JAUNE
        {
          int i = 0;
          int size = width * height;
          unsigned char nivGris;
          unsigned char nivGrisMax = 255 - vpColor::id_unknown;

          while ( i < size )
          {
            nivGris = src_8[i] ;
            if ( nivGris > nivGrisMax )
              dst_8[i] = 255;
            else
              dst_8[i] = nivGris;
            i++ ;
          }
        }

        // Affichage de l'image dans la Pixmap.
        XPutImage ( display, pixmap, context, Ximage, 0, 0, 0, 0, width, height );
        XSetWindowBackgroundPixmap ( display, window, pixmap );
//        XClearWindow ( display, window );
//        XSync ( display,1 );
        break;
      }
      case 16:
      {
        unsigned short      *dst_16 = NULL;
        dst_16 = ( unsigned short* ) Ximage->data;

        for ( unsigned int i = 0; i < height ; i++ )
        {
          for ( unsigned int j=0 ; j < width; j++ )
          {
            * ( dst_16+ ( i*width+j ) ) = ( unsigned short ) colortable[I[i][j]] ;
          }
        }

        // Affichage de l'image dans la Pixmap.
        XPutImage ( display, pixmap, context, Ximage, 0, 0, 0, 0, width, height );
        XSetWindowBackgroundPixmap ( display, window, pixmap );
//        XClearWindow ( display, window );
//        XSync ( display,1 );
        break;
      }

      case 24:
      default:
      {
        unsigned char       *dst_32 = NULL;
        unsigned int size = width * height ;
        dst_32 = ( unsigned char* ) Ximage->data;
        unsigned char *bitmap = I.bitmap ;
        unsigned char *n = I.bitmap + size;
        //for (unsigned int i = 0; i < size; i++) // suppression de l'iterateur i
        while ( bitmap < n )
        {
          char val = * ( bitmap++ );
          * ( dst_32 ++ ) = val;  // Composante Rouge.
          * ( dst_32 ++ ) = val;  // Composante Verte.
          * ( dst_32 ++ ) = val;  // Composante Bleue.
          * ( dst_32 ++ ) = val;
        }

        // Affichage de l'image dans la Pixmap.
        XPutImage ( display, pixmap, context, Ximage, 0, 0, 0, 0, width, height );
        XSetWindowBackgroundPixmap ( display, window, pixmap );
//        XClearWindow ( display, window );
//        XSync ( display,1 );
        break;
      }
    }
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}
/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImage ( const vpImage<vpRGBa> &I )
{

  if ( Xinitialise )
  {

    switch ( screen_depth )
    {
      case 24:
      case 32:
      {
        /*
         * 32-bit source, 24/32-bit destination
         */

        unsigned char       *dst_32 = NULL;
        dst_32 = ( unsigned char* ) Ximage->data;
#ifdef BIGENDIAN
        // little indian/big indian
        for ( unsigned int i = 0; i < I.getWidth() * I.getHeight() ; i++ )
        {
          dst_32[i*4] = I.bitmap[i].A;
          dst_32[i*4 + 1] = I.bitmap[i].R;
          dst_32[i*4 + 2] = I.bitmap[i].G;
          dst_32[i*4 + 3] = I.bitmap[i].B;
        }
#else
        for ( unsigned int i = 0; i < I.getWidth() * I.getHeight() ; i++ )
        {
          dst_32[i*4] = I.bitmap[i].B;
          dst_32[i*4 + 1] = I.bitmap[i].G;
          dst_32[i*4 + 2] = I.bitmap[i].R;
          dst_32[i*4 + 3] = I.bitmap[i].A;
        }
#endif
        // Affichage de l'image dans la Pixmap.
        XPutImage ( display, pixmap, context, Ximage, 0, 0, 0, 0, width, height );
        XSetWindowBackgroundPixmap ( display, window, pixmap );
//        XClearWindow ( display, window );
//        XSync ( display,1 );
        break;

      }
      default:
        vpERROR_TRACE ( "Unsupported depth (%d bpp) for color display",
                        screen_depth ) ;
        throw ( vpDisplayException ( vpDisplayException::depthNotSupportedError,
                                     "Unsupported depth for color display" ) ) ;
    }
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display an image with a reference to the bitmap.

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Pointer to the image bitmap.

  \sa init(), closeDisplay()
*/  
void vpDisplayX::displayImage ( const unsigned char *I )
{
  unsigned char       *dst_32 = NULL;

  if ( Xinitialise )
  {

    dst_32 = ( unsigned char* ) Ximage->data;

    for ( unsigned int i = 0; i < width * height; i++ )
    {
      * ( dst_32 ++ ) = *I; // red component.
      * ( dst_32 ++ ) = *I; // green component.
      * ( dst_32 ++ ) = *I; // blue component.
      * ( dst_32 ++ ) = *I; // luminance component.
      I++;
    }

    // Affichage de l'image dans la Pixmap.
    XPutImage ( display, pixmap, context, Ximage, 0, 0, 0, 0, width, height );
    XSetWindowBackgroundPixmap ( display, window, pixmap );
//    XClearWindow ( display, window );
//    XSync ( display,1 );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!

  Close the window.

  \sa init()

*/
void vpDisplayX::closeDisplay()
{
  if ( Xinitialise )
  {
    if ( ximage_data_init == true )
      free ( Ximage->data );

    Ximage->data = NULL;
    XDestroyImage ( Ximage );

    XFreePixmap ( display, pixmap );

    XFreeGC ( display, context );
    XDestroyWindow ( display, window );
    XCloseDisplay ( display );

    Xinitialise = false;
    if ( title != NULL )
    {
      delete [] title ;
      title = NULL ;
    }

    if (x_color != NULL) {
      delete [] x_color;
      x_color = NULL;
    }

  }
  else
  {
    if ( title != NULL )
    {
      delete [] title ;
      title = NULL ;
    }
  }
}


/*!
  Flushes the X buffer.
  It's necessary to use this function to see the results of any drawing.

*/
void vpDisplayX::flushDisplay()
{
  if ( Xinitialise )
  {
    XClearWindow ( display, window );
    XFlush ( display );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}


/*!
  Set the window backgroud to \e color.
  \param color : Background color.
*/
void vpDisplayX::clearDisplay ( vpColor color )
{
  if ( Xinitialise )
  {

    if (color.id < vpColor::id_unknown)
      XSetWindowBackground ( display, window, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XClearWindow ( display, window );

    XFreePixmap ( display, pixmap );
    // Pixmap creation.
    pixmap = XCreatePixmap ( display, window, width, height, screen_depth );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplayX::displayArrow ( const vpImagePoint &ip1, 
				const vpImagePoint &ip2,
                                vpColor color,
                                unsigned int w, unsigned int h,
				unsigned int thickness)
{
  if ( Xinitialise )
  {
    try
    {
      double a = ip2.get_i() - ip1.get_i() ;
      double b = ip2.get_j() - ip1.get_j() ;
      double lg = sqrt ( vpMath::sqr ( a ) + vpMath::sqr ( b ) ) ;

      if ( ( a==0 ) && ( b==0 ) )
      {
        // DisplayCrossLarge(i1,j1,3,col) ;
      }
      else
      {
        a /= lg ;
        b /= lg ;

	vpImagePoint ip3;
        ip3.set_i(ip2.get_i() - w*a);
        ip3.set_j(ip2.get_j() - w*b);

	vpImagePoint ip4;
	ip4.set_i( ip3.get_i() - b*h );
	ip4.set_j( ip3.get_j() + a*h );

	displayLine ( ip2, ip4, color, thickness ) ;
        
	ip4.set_i( ip3.get_i() + b*h );
	ip4.set_j( ip3.get_j() - a*h );

	displayLine ( ip2, ip4, color, thickness ) ;
	displayLine ( ip1, ip2, color, thickness ) ;
      }
    }
    catch ( ... )
    {
      vpERROR_TRACE ( "Error caught" ) ;
      throw ;
    }
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param ip : Upper left image point location of the string in the display.
  \param text : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void vpDisplayX::displayCharString ( const vpImagePoint &ip,
                                     const char *text, 
				     vpColor color )
{
  if ( Xinitialise )
  {
    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XDrawString ( display, pixmap, context, 
		  (int)ip.get_u(), (int)ip.get_v(), 
		  text, strlen ( text ) );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display a circle.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the circle.
  \param thickness : Thickness of the circle. This parameter is only useful 
  when \e fill is set to false.
*/
void vpDisplayX::displayCircle ( const vpImagePoint &center,
				 unsigned int radius,
                                 vpColor color,
				 bool fill,
				 unsigned int thickness )
{
  if ( Xinitialise )
  {
    if ( thickness == 1 ) thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XSetLineAttributes ( display, context, thickness,
                         LineSolid, CapButt, JoinBevel );

    if ( fill == false )
      XDrawArc ( display, pixmap, context, 
		 vpMath::round( center.get_u()-radius ), 
		 vpMath::round( center.get_v()-radius ),
		 radius*2, radius*2, 0, 23040 ); /* 23040 = 360*64 */
    else
      XFillArc ( display, pixmap, context, 
		 vpMath::round( center.get_u()-radius ), 
		 vpMath::round( center.get_v()-radius ),
		 radius*2, radius*2, 0, 23040 ); /* 23040 = 360*64 */
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display a cross at the image point \e ip location.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplayX::displayCross ( const vpImagePoint &ip, 
                                unsigned int size, 
				vpColor color,
				unsigned int thickness)
{
  if ( Xinitialise )
  {
    try
    {
      double i = ip.get_i();
      double j = ip.get_j();
      vpImagePoint ip1, ip2;

      ip1.set_i( i-size/2 ); 
      ip1.set_j( j );
      ip2.set_i( i+size/2 ); 
      ip2.set_j( j );
      displayLine ( ip1, ip2, color, thickness ) ;

      ip1.set_i( i ); 
      ip1.set_j( j-size/2 );
      ip2.set_i( i ); 
      ip2.set_j( j+size/2 );

      displayLine ( ip1, ip2, color, thickness ) ;
    }
    catch ( ... )
    {
      vpERROR_TRACE ( "Error caught" ) ;
      throw ;
    }
  }

  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }

}
/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayX::displayDotLine ( const vpImagePoint &ip1, 
				  const vpImagePoint &ip2,
                                  vpColor color, 
				  unsigned int thickness )
{

  if ( Xinitialise )
  {
    if ( thickness == 1 ) thickness = 0;

    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XSetLineAttributes ( display, context, thickness,
                         LineOnOffDash, CapButt, JoinBevel );

    XDrawLine ( display, pixmap, context, 
		vpMath::round( ip1.get_u() ),
		vpMath::round( ip1.get_v() ),
		vpMath::round( ip2.get_u() ),
		vpMath::round( ip2.get_v() ) );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayX::displayLine ( const vpImagePoint &ip1, 
			       const vpImagePoint &ip2,
                               vpColor color, 
			       unsigned int thickness )
{
  if ( Xinitialise )
  {
    if ( thickness == 1 ) thickness = 0;

    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XSetLineAttributes ( display, context, thickness,
                         LineSolid, CapButt, JoinBevel );

    XDrawLine ( display, pixmap, context, 
		vpMath::round( ip1.get_u() ),
		vpMath::round( ip1.get_v() ),
		vpMath::round( ip2.get_u() ),
		vpMath::round( ip2.get_v() ) );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display a point at the image point \e ip location.
  \param ip : Point location.
  \param color : Point color.
*/
void vpDisplayX::displayPoint ( const vpImagePoint &ip,
                                vpColor color )
{
  if ( Xinitialise )
  {
    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XDrawPoint ( display, pixmap, context, 
		 vpMath::round( ip.get_u() ), 
		 vpMath::round( ip.get_v() ) );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!  
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param topLeft : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplayX::displayRectangle ( const vpImagePoint &topLeft,
                               unsigned int width, unsigned int height,
                               vpColor color, bool fill,
			       unsigned int thickness )
{
  if ( Xinitialise )
  {
    if ( thickness == 1 ) thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }
    XSetLineAttributes ( display, context, thickness,
                         LineSolid, CapButt, JoinBevel );
    if ( fill == false )
      XDrawRectangle ( display, pixmap, context, 
		       vpMath::round( topLeft.get_u() ),
		       vpMath::round( topLeft.get_v() ),
		       width-1, height-1 );
    else
      XFillRectangle ( display, pixmap, context, 
		       vpMath::round( topLeft.get_u() ),
		       vpMath::round( topLeft.get_v() ),
		       width, height );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!  
  Display a rectangle.

  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplayX::displayRectangle ( const vpImagePoint &topLeft,
                               const vpImagePoint &bottomRight,
                               vpColor color, bool fill,
			       unsigned int thickness )
{
  if ( Xinitialise )
  {
    if ( thickness == 1 ) thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XSetLineAttributes ( display, context, thickness,
                         LineSolid, CapButt, JoinBevel );

    int width  = vpMath::round( bottomRight.get_u() - topLeft.get_u() );
    int height = vpMath::round( bottomRight.get_v() - topLeft.get_v() );
    if ( fill == false )
      XDrawRectangle ( display, pixmap, context, 
		       vpMath::round( topLeft.get_u() ),
		       vpMath::round( topLeft.get_v() ),
		       width-1, height-1 );
    else
      XFillRectangle ( display, pixmap, context, 
		       vpMath::round( topLeft.get_u() ),
		       vpMath::round( topLeft.get_v() ),
		       width, height );
  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Display a rectangle.

  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void
vpDisplayX::displayRectangle ( const vpRect &rectangle,
                               vpColor color, bool fill,
			       unsigned int thickness )
{
  if ( Xinitialise )
  {
    if ( thickness == 1 ) thickness = 0;
    if (color.id < vpColor::id_unknown)
      XSetForeground ( display, context, x_color[color.id] );
    else {
      xcolor.pad   = 0;
      xcolor.red   = 256 * color.R;
      xcolor.green = 256 * color.G;
      xcolor.blue  = 256 * color.B;
      XAllocColor ( display, lut, &xcolor );
      XSetForeground ( display, context, xcolor.pixel );
    }

    XSetLineAttributes ( display, context, thickness,
                         LineSolid, CapButt, JoinBevel );

    if ( fill == false )
      XDrawRectangle ( display, pixmap, context,
                       vpMath::round( rectangle.getLeft() ), 
		       vpMath::round( rectangle.getTop() ),
		       vpMath::round( rectangle.getWidth()-1 ), 
		       vpMath::round( rectangle.getHeight()-1 ) );
    else
      XFillRectangle ( display, pixmap, context,
                       vpMath::round( rectangle.getLeft() ), 
		       vpMath::round( rectangle.getTop() ),
                       vpMath::round( rectangle.getWidth() ), 
		       vpMath::round( rectangle.getHeight() ) );

  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!

  Wait for a click from one of the mouse button.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return 
  - true if a button was clicked. This is always the case if blocking is set 
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool
vpDisplayX::getClick(bool blocking)
{

  bool ret = false;

  if ( Xinitialise ) {
    Window  rootwin, childwin ;
    int   root_x, root_y, win_x, win_y ;
    unsigned int  modifier ;

    // Event testing
    if(blocking){
      XCheckMaskEvent(display , ButtonPressMask, &event);
      XCheckMaskEvent(display , ButtonReleaseMask, &event);
      XMaskEvent ( display, ButtonPressMask ,&event );
      ret = true;
    }
    else{
      ret = XCheckMaskEvent(display , ButtonPressMask, &event);
    }
       
    if(ret){
      /* Recuperation de la coordonnee du pixel cliqu�. */
      if ( XQueryPointer ( display,
                           window,
                           &rootwin, &childwin,
                           &root_x, &root_y,
                           &win_x, &win_y,
                         &modifier ) ) {}
    }
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret;
}

/*!

  Wait for a click from one of the mouse button and get the position
  of the clicked image point.

  \param ip [out] : The coordinates of the clicked image point.

  \param blocking [in] : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return 
  - true if a button was clicked. This is always the case if blocking is set 
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool
vpDisplayX::getClick ( vpImagePoint &ip, bool blocking )
{

  bool ret = false;
  if ( Xinitialise ) {
    unsigned int u, v ;

    Window  rootwin, childwin ;
    int   root_x, root_y, win_x, win_y ;
    unsigned int  modifier ;
    // Event testing
    if(blocking){
      XCheckMaskEvent(display , ButtonPressMask, &event);
      XCheckMaskEvent(display , ButtonReleaseMask, &event);
      XMaskEvent ( display, ButtonPressMask ,&event );
      ret = true;
    }
    else{
      ret = XCheckMaskEvent(display , ButtonPressMask, &event);
    }
       
    if(ret){
      // Get mouse position
      if ( XQueryPointer ( display,
                           window,
                           &rootwin, &childwin,
                           &root_x, &root_y,
                           &win_x, &win_y,
                           &modifier ) ) {
        u = event.xbutton.x;
        v = event.xbutton.y;
        ip.set_u( u );
        ip.set_v( v );
      }
    }
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret ;
}

/*!

  Wait for a mouse button click and get the position of the clicked
  pixel. The button used to click is also set.
  
  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The button used to click.

  \param blocking [in] : 
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.
*/
bool
vpDisplayX::getClick ( vpImagePoint &ip,
                       vpMouseButton::vpMouseButtonType &button,
		       bool blocking )
{

  bool ret = false;
  if ( Xinitialise ) {
    unsigned int u,v ;

    Window  rootwin, childwin ;
    int   root_x, root_y, win_x, win_y ;
    unsigned int  modifier ;

    // Event testing
    if(blocking){
      XCheckMaskEvent(display , ButtonPressMask, &event);
      XCheckMaskEvent(display , ButtonReleaseMask, &event);
      XMaskEvent ( display, ButtonPressMask ,&event );
      ret = true;
    }
    else{
      ret = XCheckMaskEvent(display , ButtonPressMask, &event);
    }
       
    if(ret){
      // Get mouse position
      if ( XQueryPointer ( display,
                           window,
                           &rootwin, &childwin,
                           &root_x, &root_y,
                           &win_x, &win_y,
                           &modifier ) ) {
        u = event.xbutton.x;
        v = event.xbutton.y;
	ip.set_u( u );
        ip.set_v( v );
        switch ( event.xbutton.button ) {
          case Button1: button = vpMouseButton::button1; break;
          case Button2: button = vpMouseButton::button2; break;
          case Button3: button = vpMouseButton::button3; break;
        }
      }
    }   
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret ;
}

/*!

  Wait for a mouse button click release and get the position of the
  image point were the click release occurs.  The button used to click is
  also set. Same method as getClick(unsigned int&, unsigned int&,
  vpMouseButton::vpMouseButtonType &, bool).

  \param ip [out] : Position of the clicked image point.

  \param button [in] : Button used to click.

  \param blocking [in] : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return 
  - true if a button was clicked. This is always the case if blocking is set 
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.

  \sa getClick(vpImagePoint &, vpMouseButton::vpMouseButtonType &, bool)

*/
bool
vpDisplayX::getClickUp ( vpImagePoint &ip,
                         vpMouseButton::vpMouseButtonType &button,
			 bool blocking )
{

  bool ret = false;
  if ( Xinitialise ) {
    unsigned int u,v ;
    Window  rootwin, childwin ;
    int   root_x, root_y, win_x, win_y ;
    unsigned int  modifier ;

    // Event testing
    if(blocking){
      XCheckMaskEvent(display , ButtonPressMask, &event);
      XCheckMaskEvent(display , ButtonReleaseMask, &event);
      XMaskEvent ( display, ButtonReleaseMask ,&event );
      ret = true;
    }
    else{
      ret = XCheckMaskEvent(display , ButtonReleaseMask, &event);
    }
       
    if(ret){
      /* Recuperation de la coordonnee du pixel cliqu�. */
      if ( XQueryPointer ( display,
                           window,
                           &rootwin, &childwin,
                           &root_x, &root_y,
                           &win_x, &win_y,
                           &modifier ) ) {
        u = event.xbutton.x;
        v = event.xbutton.y;
	ip.set_u( u );
        ip.set_v( v );
        switch ( event.xbutton.button ) {
          case Button1: button = vpMouseButton::button1; break;
          case Button2: button = vpMouseButton::button2; break;
          case Button3: button = vpMouseButton::button3; break;
        }
      }
    }
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret ;
}

/*
  Gets the displayed image (including the overlay plane)
  and returns an RGBa image.

  \param I : Image to get.
*/
void vpDisplayX::getImage ( vpImage<vpRGBa> &I )
{

  if ( Xinitialise )
  {


    XImage *xi ;
    //xi= XGetImage ( display,window, 0,0, getWidth(), getHeight(),
    //                AllPlanes, ZPixmap ) ;

    XCopyArea (display,window, pixmap, context,
               0,0, getWidth(), getHeight(), 0, 0);

    xi= XGetImage ( display,pixmap, 0,0, getWidth(), getHeight(),
                    AllPlanes, ZPixmap ) ;

    try
    {
      I.resize ( getHeight(), getWidth() ) ;
    }
    catch ( ... )
    {
      vpERROR_TRACE ( "Error caught" ) ;
      throw ;
    }

    unsigned char       *src_32 = NULL;
    src_32 = ( unsigned char* ) xi->data;

#ifdef BIGENDIAN
    // little indian/big indian
    for ( unsigned int i = 0; i < I.getWidth() * I.getHeight() ; i++ )
    {
      I.bitmap[i].A = src_32[i*4] ;
      I.bitmap[i].R = src_32[i*4 + 1] ;
      I.bitmap[i].G = src_32[i*4 + 2] ;
      I.bitmap[i].B = src_32[i*4 + 3] ;
    }
#else
    for ( unsigned int i = 0; i < I.getWidth() * I.getHeight() ; i++ )
    {
      I.bitmap[i].B = src_32[i*4] ;
      I.bitmap[i].G = src_32[i*4 + 1] ;
      I.bitmap[i].R = src_32[i*4 + 2] ;
      I.bitmap[i].A = src_32[i*4 + 3] ;
    }
#endif


    XDestroyImage ( xi ) ;

  }
  else
  {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
}

/*!
  Gets the window depth (8, 16, 24, 32).
*/
unsigned int vpDisplayX::getScreenDepth()
{
  Display *_display;
  unsigned int  screen;
  unsigned int  depth;

  if ( ( _display = XOpenDisplay ( NULL ) ) == NULL )
  {
    vpERROR_TRACE ( "Can't connect display on server %s.",
                    XDisplayName ( NULL ) );
    throw ( vpDisplayException ( vpDisplayException::connexionError,
                                 "Can't connect display on server." ) ) ;
  }
  screen = DefaultScreen ( _display );
  depth  = DefaultDepth ( _display, screen );

  XCloseDisplay ( _display );

  return ( depth );
}

/*!
  Gets the window size.
  \param width, height : Size of the display.
 */
void vpDisplayX::getScreenSize ( unsigned int &width, unsigned int &height )
{
  Display *_display;
  unsigned int  screen;

  if ( ( _display = XOpenDisplay ( NULL ) ) == NULL )
  {
    vpERROR_TRACE ( "Can't connect display on server %s.",
                    XDisplayName ( NULL ) );
    throw ( vpDisplayException ( vpDisplayException::connexionError,
                                 "Can't connect display on server." ) ) ;
  }
  screen = DefaultScreen ( _display );
  width = DisplayWidth ( _display, screen );
  height = DisplayHeight ( _display, screen );

  XCloseDisplay ( _display );
}
/*!

  Get a keyboard event.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \return 
  - true if a key was pressed. This is always the case if blocking is set 
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.

*/
bool
vpDisplayX::getKeyboardEvent(bool blocking)
{

  bool ret = false;

  if ( Xinitialise ) {
    // Event testing
    if(blocking){
      XMaskEvent ( display, KeyPressMask ,&event );
      ret = true;
    }
    else{
      ret = XCheckMaskEvent(display , KeyPressMask, &event);
    }
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret;
}
/*!

  Get a keyboard event.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a key is
    pressed and then returns always true.
  - When set to false, returns true only if a key is
    pressed, otherwise returns false.

  \param string [out]: If possible, an ISO Latin-1 character
  corresponding to the keyboard key.

  \return 
  - true if a key was pressed. This is always the case if blocking is set 
    to \e true.
  - false if no key was pressed. This can occur if blocking is set
    to \e false.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.

*/
bool
vpDisplayX::getKeyboardEvent(char *string, bool blocking)
{

  bool ret = false;
  KeySym  keysym;
  int     count;
  XComposeStatus compose_status;
  char buffer;
  
  if ( Xinitialise ) {
    // Event testing
    if(blocking){
      XMaskEvent ( display, KeyPressMask ,&event );
      count = XLookupString ((XKeyEvent *)&event, &buffer, 1,
			     &keysym, &compose_status);
      //std::cout <<"count: " << count << " get \"" << buffer << "\"" << std::endl;
      sprintf(string, "%c", buffer);
      ret = true;
    }
    else{
      ret = XCheckMaskEvent(display , KeyPressMask, &event);
      if (ret) {
	count = XLookupString ((XKeyEvent *)&event, &buffer, 1,
			       &keysym, &compose_status);
	sprintf(string, "%c", buffer);
      }
    }
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret;
}
/*!

  Get the coordinates of the mouse pointer.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true if a pointer motion event was received, false otherwise.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.

*/
bool
vpDisplayX::getPointerMotionEvent ( vpImagePoint &ip)
{

  bool ret = false;
  if ( Xinitialise ) {
    unsigned int u, v ;

    Window  rootwin, childwin ;
    int   root_x, root_y, win_x, win_y ;
    unsigned int  modifier ;
    // Event testing
    ret = XCheckMaskEvent(display , PointerMotionMask, &event);
       
    if(ret){
      // Get mouse position
      if ( XQueryPointer ( display,
                           window,
                           &rootwin, &childwin,
                           &root_x, &root_y,
                           &win_x, &win_y,
                           &modifier ) ) {
        u = event.xbutton.x;
        v = event.xbutton.y;
	ip.set_u( u );
        ip.set_v( v );	
      }
    }
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret ;
}



bool
vpDisplayX::getPointerPosition ( vpImagePoint &ip)
{

  bool ret = false;
  if ( Xinitialise ) {
    unsigned int u, v ;

    Window  rootwin, childwin ;
    int   root_x, root_y, win_x, win_y ;
    unsigned int  modifier ;
    // Event testing
    ret = true;
       
    if(ret){
      // Get mouse position
      if ( XQueryPointer ( display,
                           window,
                           &rootwin, &childwin,
                           &root_x, &root_y,
                           &win_x, &win_y,
                           &modifier ) ) {
        u = win_x;
        v = win_y;
	ip.set_u( u );
        ip.set_v( v );	
      }
    }
  }
  else {
    vpERROR_TRACE ( "X not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "X not initialized" ) ) ;
  }
  return ret ;
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
