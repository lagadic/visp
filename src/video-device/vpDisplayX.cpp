/****************************************************************************
 *
 * $Id: vpDisplayX.cpp,v 1.16 2006-12-05 10:27:59 marchand Exp $
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

/*!
  \file vpDisplayX.cpp
  \brief Define the X11 console to display images
*/

/*!
  \class vpDisplayX

  \brief The  vpDisplayX allows to display image using the X11 library.


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

  \brief constructor : initialize a display to visualize a gray level image
  (8 bits).

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
vpDisplayX::vpDisplayX(vpImage<unsigned char> &I,
		       int _x,
		       int _y,
		       char *_title) : vpDisplay()
{
  init(I,_x,_y, _title) ;

}



/*!
  \brief constructor : initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled
*/
vpDisplayX::vpDisplayX(vpImage<vpRGBa> &I,
		     int _x,
		     int _y,
		     char *_title)
{
  title = NULL ;
  init(I,_x,_y, _title) ;
}

/*!
  \brief constructor

  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled
*/
vpDisplayX::vpDisplayX(int _x, int _y, char *_title)
{
  displayHasBeenInitialized = false ;
  windowXPosition = _x ;
  windowYPosition = _y ;

  title = NULL ;

  if (_title != NULL)
  {
    title = new char[strlen(_title) + 1] ; // Modif Fabien le 19/04/02
    strcpy(title,_title) ;
  }

  ximage_data_init = false;

}

/*!
  \brief basic constructor
  \warning must an init member of the vpDisplayGTK function to be useful
  \sa init()
*/
vpDisplayX::vpDisplayX()
{
  displayHasBeenInitialized =false ;
  windowXPosition = windowYPosition = -1 ;

  title = NULL ;
  if (title != NULL)
  {
    delete [] title ;
    title = NULL ;
  }
  title = new char[1] ;
  strcpy(title,"") ;

  Xinitialise = false ;
  ximage_data_init = false;
}

/*!
  \brief close the window
*/
vpDisplayX::~vpDisplayX()
{
  closeDisplay() ;
}

/*!
  \brief Initialized the display of a gray level image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
void
vpDisplayX::init(vpImage<unsigned char> &I, int _x, int _y, char *_title)
 {

  displayHasBeenInitialized =true ;


  XSizeHints	hints;
  windowXPosition = _x ;
  windowYPosition = _y ;
  {
    if (title != NULL)
    {
      //   vpTRACE(" ") ;
      delete [] title ;
      title = NULL ;
    }

    if (_title != NULL)
    {
      title = new char[strlen(_title) + 1] ;
      strcpy(title,_title) ;
    }
  }

  // Positionnement de la fenetre dans l'écran.
  if ( (windowXPosition < 0) || (windowYPosition < 0) ) {
    hints.flags = 0;
  }
  else {
    hints.flags = USPosition;
    hints.x = windowXPosition;
    hints.y = windowYPosition;
  }


  // setup X11 --------------------------------------------------
  ncols = I.getCols();
  nrows = I.getRows();
  display = XOpenDisplay (NULL);
  if (display == NULL)
  {
    vpERROR_TRACE("Can't connect display on server %s.\n", XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError,
			     "Can't connect display on server.")) ;
  }

  screen       = DefaultScreen   (display);
  lut          = DefaultColormap (display, screen);
  screen_depth = DefaultDepth    (display, screen);

  if ((window =
       XCreateSimpleWindow (display, RootWindow (display, screen),
			    windowXPosition, windowYPosition, ncols, nrows, 1,
			    BlackPixel (display, screen),
			    WhitePixel (display, screen))) == 0)
  {
    vpERROR_TRACE("Can't create window." );
    throw(vpDisplayException(vpDisplayException::cannotOpenWindowError,
			     "Can't create window.")) ;
  }

  //
  // Create color table for 8 and 16 bits screen
  //
  if (screen_depth == 8) {
    XColor colval ;

    lut = XCreateColormap(display, window,
			  DefaultVisual(display, screen), AllocAll) ;
    colval.flags = DoRed | DoGreen | DoBlue ;

    for(int i = 0 ; i < 256 ; i++) {
      colval.pixel = i ;
      colval.red = 256 * i;
      colval.green = 256 * i;
      colval.blue = 256 * i;
      XStoreColor(display, lut, &colval);
    }

    XSetWindowColormap(display, window, lut) ;
    XInstallColormap(display, lut) ;
  }

  else if (screen_depth == 16) {
    for (int i = 0; i < 256; i ++ ) {
      color.pad = 0;
      color.red = color.green = color.blue = 256 * i;
      if (XAllocColor (display, lut, &color) == 0) {
	vpERROR_TRACE("Can't allocate 256 colors. Only %d allocated.", i);
	throw(vpDisplayException(vpDisplayException::colorAllocError,
				 "Can't allocate 256 colors.")) ;
      }
      colortable[i] = color.pixel;
    }

    XSetWindowColormap(display, window, lut) ;
    XInstallColormap(display, lut) ;

  }

  //
  // Create colors for overlay
  //
  switch (screen_depth) {

  case 8:
    XColor colval ;

    // Couleur NOIR.
    x_color[vpColor::black] = 0;

    // Couleur BLANC.
    x_color[vpColor::white] = 255;

    // Couleur ROUGE.
    x_color[vpColor::red]= 254;
    colval.pixel  = x_color[vpColor::red] ;
    colval.red    = 256 * 255;
    colval.green  = 0;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);

    // Couleur VERT.
    x_color[vpColor::green] = 253;
    colval.pixel  = x_color[vpColor::green];
    colval.red    = 0;
    colval.green  = 256 * 255;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);

    // Couleur BLEU.
    x_color[vpColor::blue] = 252;
    colval.pixel  = x_color[vpColor::blue];
    colval.red    = 0;
    colval.green  = 0;
    colval.blue   = 256 * 255;
    XStoreColor(display, lut, &colval);

    // Couleur JAUNE.
    x_color[vpColor::yellow] = 251;
    colval.pixel  = x_color[vpColor::yellow];
    colval.red    = 256 * 255;
    colval.green  = 256 * 255;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);
    break;

  case 16:
  case 24:
    {
    color.flags = DoRed | DoGreen | DoBlue ;

    // Couleur NOIR.
    color.pad   = 0;
    color.red   = 0;
    color.green = 0;
    color.blue  = 0;
    XAllocColor (display, lut, &color);

    x_color[vpColor::black] = color.pixel;

    // Couleur BLANC.
    color.pad   = 0;
    color.red   = 256* 255;
    color.green = 256* 255;
    color.blue  = 256* 255;
    XAllocColor (display, lut, &color);
    x_color[vpColor::white] = color.pixel;

    // Couleur ROUGE.
    color.pad   = 0;
    color.red   = 256* 255;
    color.green = 0;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::red] = color.pixel;

    // Couleur VERT.
    color.pad   = 0;
    color.red   = 0;
    color.green = 256*255;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::green] = color.pixel;

    // Couleur BLEU.
    color.pad = 0;
    color.red = 0;
    color.green = 0;
    color.blue  = 256* 255;
    XAllocColor (display, lut, &color);
    x_color[vpColor::blue] = color.pixel;

    // Couleur JAUNE.
    color.pad = 0;
    color.red = 256 * 255;
    color.green = 256 * 255;
    color.blue  = 0;
    XAllocColor (display, lut, &color);

    x_color[vpColor::yellow] = color.pixel;
    break;
    }
  }


  XSetStandardProperties(display, window, title, title, None, 0, 0, &hints);
  XMapWindow(display, window) ;
  // Selection des evenements.
  XSelectInput(display, window,
	       ExposureMask | ButtonPressMask | ButtonReleaseMask);

  // Creation du contexte graphique
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel(display, screen);
  values.background = BlackPixel(display, screen);
  context = XCreateGC(display, window,
		      GCPlaneMask  | GCFillStyle | GCForeground | GCBackground,
		      &values);

  if (context == NULL)
  {
    vpERROR_TRACE("Can't create graphics context.");
    throw(vpDisplayException(vpDisplayException::XWindowsError,
			     "Can't create graphics context")) ;

  }

  do
    XNextEvent(display, &event);
  while (event.xany.type != Expose);

  {
    Ximage = XCreateImage (display, DefaultVisual (display, screen),
			 screen_depth, ZPixmap, 0, NULL,
			 I.getCols() , I.getRows(), XBitmapPad(display), 0);

    Ximage->data = (char *) malloc (I.getCols() * I.getRows() * Ximage->bits_per_pixel / 8);
    ximage_data_init = true;

  }
  Xinitialise = true ;
  flushTitle(title) ;
  XSync (display, 1);

  I.display = this ;
  I.initDisplay =  true ;

}

/*!
  \brief Initialized the display of a RGBa  image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
void
vpDisplayX::init(vpImage<vpRGBa> &I, int _x, int _y, char *_title)
{

  displayHasBeenInitialized =true ;

  XSizeHints	hints;
  windowXPosition = _x ;
  windowYPosition = _y ;

  {
    if (title != NULL)
    {
      delete [] title ;
      title = NULL ;
    }

    if (_title != NULL)
    {
      title = new char[strlen(_title) + 1] ; // Modif Fabien le 19/04/02
      strcpy(title,_title) ;
    }
  }

  // Positionnement de la fenetre dans l'écran.
  if ( (windowXPosition < 0) || (windowYPosition < 0) ) {
    hints.flags = 0;
  }
  else {
    hints.flags = USPosition;
    hints.x = windowXPosition;
    hints.y = windowYPosition;
  }


  // setup X11 --------------------------------------------------
  ncols = I.getCols();
  nrows = I.getRows();

  if ((display = XOpenDisplay (NULL)) == NULL) {
    vpERROR_TRACE("Can't connect display on server %s.\n", XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError,
			     "Can't connect display on server.")) ;
  }

  screen       = DefaultScreen   (display);
  lut          = DefaultColormap (display, screen);
  screen_depth = DefaultDepth    (display, screen);

  vpDEBUG_TRACE(1, "Screen depth: %d\n", screen_depth);

  if ((window = XCreateSimpleWindow (display, RootWindow (display, screen),
				     windowXPosition, windowYPosition, ncols, nrows, 1,
				     BlackPixel (display, screen),
				     WhitePixel (display, screen))) == 0)
  {
    vpERROR_TRACE("Can't create window." );
    throw(vpDisplayException(vpDisplayException::cannotOpenWindowError,
			     "Can't create window.")) ;
  }

  //
  // Create color table for 8 and 16 bits screen
  //
  if (screen_depth == 8) {
    XColor colval ;

    lut = XCreateColormap(display, window,
			  DefaultVisual(display, screen), AllocAll) ;
    colval.flags = DoRed | DoGreen | DoBlue ;

    for(int i = 0 ; i < 256 ; i++) {
      colval.pixel = i ;
      colval.red = 256 * i;
      colval.green = 256 * i;
      colval.blue = 256 * i;
      XStoreColor(display, lut, &colval);
    }

    XSetWindowColormap(display, window, lut) ;
    XInstallColormap(display, lut) ;
  }

  else if (screen_depth == 16) {
    for (int i = 0; i < 256; i ++ ) {
      color.pad = 0;
      color.red = color.green = color.blue = 256 * i;
      if (XAllocColor (display, lut, &color) == 0) {
	vpERROR_TRACE("Can't allocate 256 colors. Only %d allocated.", i);
	throw(vpDisplayException(vpDisplayException::colorAllocError,
				 "Can't allocate 256 colors.")) ;
      }
      colortable[i] = color.pixel;
    }

    XSetWindowColormap(display, window, lut) ;
    XInstallColormap(display, lut) ;

  }


  //
  // Create colors for overlay
  //
  switch (screen_depth) {

  case 8:
    XColor colval ;

    // Couleur NOIR.
    x_color[vpColor::black] = 0;

    // Couleur BLANC.
    x_color[vpColor::white] = 255;

    // Couleur ROUGE.
    x_color[vpColor::red]= 254;
    colval.pixel  = x_color[vpColor::red] ;
    colval.red    = 256 * 255;
    colval.green  = 0;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);

    // Couleur VERT.
    x_color[vpColor::green] = 253;
    colval.pixel  = x_color[vpColor::green];
    colval.red    = 0;
    colval.green  = 256 * 255;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);

    // Couleur BLEU.
    x_color[vpColor::blue] = 252;
    colval.pixel  = x_color[vpColor::blue];
    colval.red    = 0;
    colval.green  = 0;
    colval.blue   = 256 * 255;
    XStoreColor(display, lut, &colval);

    // Couleur JAUNE.
    x_color[vpColor::yellow] = 251;
    colval.pixel  = x_color[vpColor::yellow];
    colval.red    = 256 * 255;
    colval.green  = 256 * 255;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);
    break;

  case 16:
  case 24:
    {

    color.flags = DoRed | DoGreen | DoBlue ;

    // Couleur NOIR.
    color.pad   = 0;
    color.red   = 0;
    color.green = 0;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::black] = color.pixel;

    // Couleur BLANC.
    color.pad   = 0;
    color.red   = 256* 255;
    color.green = 256* 255;
    color.blue  = 256* 255;
    XAllocColor (display, lut, &color);
    x_color[vpColor::white] = color.pixel;

    // Couleur ROUGE.
    color.pad   = 0;
    color.red   = 256* 255;
    color.green = 0;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::red] = color.pixel;

    // Couleur VERT.
    color.pad   = 0;
    color.red   = 0;
    color.green = 256*255;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::green] = color.pixel;

    // Couleur BLEU.
    color.pad = 0;
    color.red = 0;
    color.green = 0;
    color.blue  = 256* 255;
    XAllocColor (display, lut, &color);
    x_color[vpColor::blue] = color.pixel;

    // Couleur JAUNE.
    color.pad = 0;
    color.red = 256 * 255;
    color.green = 256 * 255;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::yellow] = color.pixel;
    break;
    }
  }
  XSetStandardProperties(display, window, title, title, None, 0, 0, &hints);
  XMapWindow(display, window) ;
  // Selection des evenements.
  XSelectInput(display, window,
	       ExposureMask | ButtonPressMask | ButtonReleaseMask);

  // Creation du contexte graphique
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel(display, screen);
  values.background = BlackPixel(display, screen);
  context = XCreateGC(display, window,
		      GCPlaneMask  | GCFillStyle | GCForeground | GCBackground,
		      &values);

  if (context == NULL) {
   vpERROR_TRACE("Can't create graphics context.");
    throw(vpDisplayException(vpDisplayException::XWindowsError,
			     "Can't create graphics context")) ;
  }

  do
    XNextEvent(display, &event);
  while (event.xany.type != Expose);


  {
    Ximage = XCreateImage (display, DefaultVisual (display, screen),
			 screen_depth, ZPixmap, 0, NULL,
			 I.getCols() , I.getRows(), XBitmapPad(display), 0);


    Ximage->data = (char *) malloc (I.getCols() * I.getRows() * Ximage->bits_per_pixel / 8);
    ximage_data_init = true;

  }
  Xinitialise = true ;

  XSync (display, true);
  flushTitle(title) ;

  I.display = this ;
  I.initDisplay =  true ;
}


/*!
  \brief actual member used to Initialize the display of a
  gray level or RGBa  image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
void vpDisplayX::init(int cols, int rows, int _x, int _y, char *_title)
{

  displayHasBeenInitialized = true ;


  /* setup X11 ------------------------------------------------------------- */
  ncols = cols;
  nrows = rows;

  XSizeHints	hints;

  windowXPosition = _x ;
  windowYPosition = _y ;
  // Positionnement de la fenetre dans l'écran.
  if ( (windowXPosition < 0) || (windowYPosition < 0) ) {
    hints.flags = 0;
  }
  else {
    hints.flags = USPosition;
    hints.x = windowXPosition;
    hints.y = windowYPosition;
  }


  {
    if (title != NULL)
    {
      delete [] title ;
      title = NULL ;
    }

    if (_title != NULL)
    {
      title = new char[strlen(_title) + 1] ; // Modif Fabien le 19/04/02
      strcpy(title,_title) ;
    }
  }


  if ((display = XOpenDisplay (NULL)) == NULL) {
    vpERROR_TRACE("Can't connect display on server %s.\n", XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError,
			     "Can't connect display on server.")) ;
  }

  screen       = DefaultScreen   (display);
  lut          = DefaultColormap (display, screen);
  screen_depth = DefaultDepth    (display, screen);

  vpTRACE("Screen depth: %d\n", screen_depth);

  if ((window = XCreateSimpleWindow (display, RootWindow (display, screen),
				     windowXPosition, windowYPosition,
				     cols, rows, 1,
				     BlackPixel (display, screen),
				     WhitePixel (display, screen))) == 0)
  {
    vpERROR_TRACE("Can't create window." );
    throw(vpDisplayException(vpDisplayException::cannotOpenWindowError,
			     "Can't create window.")) ;
  }


  //
  // Create color table for 8 and 16 bits screen
  //
  if (screen_depth == 8) {
    XColor colval ;

    lut = XCreateColormap(display, window,
			  DefaultVisual(display, screen), AllocAll) ;
    colval.flags = DoRed | DoGreen | DoBlue ;

    for(int i = 0 ; i < 256 ; i++) {
      colval.pixel = i ;
      colval.red = 256 * i;
      colval.green = 256 * i;
      colval.blue = 256 * i;
      XStoreColor(display, lut, &colval);
    }

    XSetWindowColormap(display, window, lut) ;
    XInstallColormap(display, lut) ;
  }

  else if (screen_depth == 16) {
    for (int i = 0; i < 256; i ++ ) {
      color.pad = 0;
      color.red = color.green = color.blue = 256 * i;
      if (XAllocColor (display, lut, &color) == 0) {
	vpERROR_TRACE("Can't allocate 256 colors. Only %d allocated.", i);
	throw(vpDisplayException(vpDisplayException::colorAllocError,
				 "Can't allocate 256 colors.")) ;
      }
      colortable[i] = color.pixel;
    }

    XSetWindowColormap(display, window, lut) ;
    XInstallColormap(display, lut) ;

  }


  //
  // Create colors for overlay
  //
  switch (screen_depth) {

  case 8:
    XColor colval ;

    // Couleur NOIR.
    x_color[vpColor::black] = 0;

    // Couleur BLANC.
    x_color[vpColor::white] = 255;

    // Couleur ROUGE.
    x_color[vpColor::red]= 254;
    colval.pixel  = x_color[vpColor::red] ;
    colval.red    = 256 * 255;
    colval.green  = 0;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);

    // Couleur VERT.
    x_color[vpColor::green] = 253;
    colval.pixel  = x_color[vpColor::green];
    colval.red    = 0;
    colval.green  = 256 * 255;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);

    // Couleur BLEU.
    x_color[vpColor::blue] = 252;
    colval.pixel  = x_color[vpColor::blue];
    colval.red    = 0;
    colval.green  = 0;
    colval.blue   = 256 * 255;
    XStoreColor(display, lut, &colval);

    // Couleur JAUNE.
    x_color[vpColor::yellow] = 251;
    colval.pixel  = x_color[vpColor::yellow];
    colval.red    = 256 * 255;
    colval.green  = 256 * 255;
    colval.blue   = 0;
    XStoreColor(display, lut, &colval);
    break;

  case 16:
  case 24:
    {
    color.flags = DoRed | DoGreen | DoBlue ;

    // Couleur NOIR.
    color.pad   = 0;
    color.red   = 0;
    color.green = 0;
    color.blue  = 0;
    XAllocColor (display, lut, &color);

    x_color[vpColor::black] = color.pixel;

    // Couleur BLANC.
    color.pad   = 0;
    color.red   = 256* 255;
    color.green = 256* 255;
    color.blue  = 256* 255;
    XAllocColor (display, lut, &color);
    x_color[vpColor::white] = color.pixel;

    // Couleur ROUGE.
    color.pad   = 0;
    color.red   = 256* 255;
    color.green = 0;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::red] = color.pixel;

    // Couleur VERT.
    color.pad   = 0;
    color.red   = 0;
    color.green = 256*255;
    color.blue  = 0;
    XAllocColor (display, lut, &color);
    x_color[vpColor::green] = color.pixel;

    // Couleur BLEU.
    color.pad = 0;
    color.red = 0;
    color.green = 0;
    color.blue  = 256* 255;
    XAllocColor (display, lut, &color);
    x_color[vpColor::blue] = color.pixel;

    // Couleur JAUNE.
    color.pad = 0;
    color.red = 256 * 255;
    color.green = 256 * 255;
    color.blue  = 0;
    XAllocColor (display, lut, &color);

    x_color[vpColor::yellow] = color.pixel;
    break;
    }
  }

  XSetStandardProperties(display, window, title, title, None, 0, 0, &hints);
  XMapWindow(display, window) ;
  // Selection des evenements.
  XSelectInput(display, window,
	       ExposureMask | ButtonPressMask | ButtonReleaseMask);

  /* Creation du contexte graphique */
  values.plane_mask = AllPlanes;
  values.fill_style = FillSolid;
  values.foreground = WhitePixel(display, screen);
  values.background = BlackPixel(display, screen);
  context = XCreateGC(display, window,
		      GCPlaneMask  | GCFillStyle | GCForeground | GCBackground,
		      &values);

  if (context == NULL) {
    vpERROR_TRACE("Can't create graphics context.");
    throw(vpDisplayException(vpDisplayException::XWindowsError,
			     "Can't create graphics context")) ;
  }

  do
    XNextEvent(display, &event);
  while (event.xany.type != Expose);

  {
    Ximage = XCreateImage (display, DefaultVisual (display, screen),
			 screen_depth, ZPixmap, 0, NULL,
			 cols, rows, XBitmapPad(display), 0);

    Ximage->data = (char *) malloc (cols * rows * Ximage->bits_per_pixel / 8);
    ximage_data_init = true;
  }
  Xinitialise = true ;

  XSync (display, true);
  flushTitle(title) ;

}


/*!
  \brief display the gray level image (8bits)

  GTK has to be initialized

  \warning suppres the overlay drawing

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImage(vpImage<unsigned char> &I)
{

  if (Xinitialise)
  {
    switch (screen_depth) {
    case 8: {
      unsigned char       *src_8  = NULL;
      unsigned char       *dst_8  = NULL;
      src_8 = (unsigned char *) I.bitmap;
      dst_8 = (unsigned char *) Ximage->data;
      // Correction de l'image de facon a liberer les niveaux de gris
      // ROUGE, VERT, BLEU, JAUNE
      {
	int	i = 0;
	int	size = ncols * nrows;
	unsigned char	nivGris;
	unsigned char	nivGrisMax = 255 - vpColor::none;

	while(i < size){
	  nivGris = src_8[i] ;
	  if (nivGris > nivGrisMax)
	    dst_8[i] = 255;
	  else
	    dst_8[i] = nivGris;
	  i++ ;
	}
      }

      XPutImage (display, window, context, Ximage, 0, 0, 0, 0, ncols, nrows);
      break;
    }
    case 16: {
      unsigned short      *dst_16 = NULL;
      dst_16 = (unsigned short*)Ximage->data;

      for (int i = 0; i < nrows ; i++) {
	for (int j=0 ; j < ncols; j++){
	  *(dst_16+(i*ncols+j)) = (unsigned short)colortable[I[i][j]] ;
	}
      }

      XPutImage (display, window, context, Ximage, 0, 0, 0, 0, ncols, nrows);

      break;
    }

    case 24:
    default: {
      unsigned char       *dst_32 = NULL;
      int ncolsnrows = ncols * nrows ;
      dst_32 = (unsigned char*)Ximage->data;
      unsigned char *bitmap = I.bitmap ;
      unsigned char *n = I.bitmap + ncolsnrows;
      //for (int i = 0; i < ncolsnrows; i++) // suppression de l'iterateur i
      while (bitmap < n)  
	{
	  char val = *(bitmap++);
	  *(dst_32 ++) = val;	// Composante Rouge.
	  *(dst_32 ++) = val;	// Composante Verte.
	  *(dst_32 ++) = val;	// Composante Bleue.
	  *(dst_32 ++) = val;
	}

      // Creation de la Pixmap.
      pixmap = XCreatePixmap(display, window, ncols, nrows, screen_depth);
      // Affichage de l'image dans la Pixmap.
      XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, ncols, nrows);
      XSetWindowBackgroundPixmap(display, window, pixmap);

      XClearWindow(display, window);
      XFreePixmap(display, pixmap);
      break;
    }
    }
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}
/*!
  \brief display the RGBa level image (32bits)

  GTK has to be initialized

  \warning suppres the overlay drawing

  \sa init(), closeDisplay()
*/
void vpDisplayX::displayImage(vpImage<vpRGBa> &I)
{

  if (Xinitialise)
  {

    switch (screen_depth) {
    case 24:
    case 32:{
      /*
       * 32-bit source, 24/32-bit destination
       */

      unsigned char       *dst_32 = NULL;
      dst_32 = (unsigned char*)Ximage->data;
#ifdef APPLE
      // little indian/big indian
      for (int i = 0; i < I.getCols() * I.getRows() ; i++) {
	dst_32[i*4] = I.bitmap[i].A;
	dst_32[i*4 + 1] = I.bitmap[i].R;
	dst_32[i*4 + 2] = I.bitmap[i].G;
	dst_32[i*4 + 3] = I.bitmap[i].B;
      }
#else
      for (int i = 0; i < I.getCols() * I.getRows() ; i++) {
	dst_32[i*4] = I.bitmap[i].B;
	dst_32[i*4 + 1] = I.bitmap[i].G;
	dst_32[i*4 + 2] = I.bitmap[i].R;
	dst_32[i*4 + 3] = I.bitmap[i].A;
      }
#endif
      break;
    }
    default:
      vpERROR_TRACE("Unsupported depth (%d bpp) for color display",
		  screen_depth) ;
      throw(vpDisplayException(vpDisplayException::depthNotSupportedError,
			       "Unsupported depth for color display")) ;
    }

    XPutImage (display, window, context, Ximage, 0, 0, 0, 0,
	       I.getCols(), I.getRows());

  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}
/*
  \brief gets the displayed image (including the overlay plane)
  and returns an RGBa image
*/
void vpDisplayX::getImage(vpImage<vpRGBa> &I)
{

  if (Xinitialise)
  {


    XImage *xi ;
    xi= XGetImage(display,window, 0,0, getCols(), getRows(), AllPlanes, ZPixmap) ;
    try{
      I.resize(getRows(), getCols()) ;
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }

    unsigned char       *src_24 = NULL;
    src_24 = (unsigned char*)xi->data;
    for (int i = 0; i < I.getCols() * I.getRows() ; i++)
    {
      I.bitmap[i].B = src_24[i*4]  ;
      I.bitmap[i].G = src_24[i*4 + 1]  ;
      I.bitmap[i].R = src_24[i*4 + 2]  ;
    }
    XDestroyImage(xi) ;

  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}

/*!


*/
void vpDisplayX::displayImage(unsigned char *I)
{

  unsigned char       *dst_32 = NULL;



  if (Xinitialise)
  {

    dst_32 = (unsigned char*)Ximage->data;

    for (int i = 0; i < ncols * nrows; i++) {
      char val = I[i];
      *(dst_32 ++) = val;	// Composante Rouge.
      *(dst_32 ++) = val;	// Composante Vertee.
      *(dst_32 ++) = val;	// Composante Bleue.
      *(dst_32 ++) = val;	// Composante Bleue.
    }

    // Creation de la Pixmap.
    pixmap = XCreatePixmap(display, window, ncols, nrows, screen_depth);
    // Affichage de l'image dans la Pixmap.
    XPutImage(display, pixmap, context, Ximage, 0, 0, 0, 0, ncols, nrows);
    XSetWindowBackgroundPixmap(display, window, pixmap);

    XClearWindow(display, window);
    XFreePixmap(display, pixmap);
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}

/*!

  \brief close the window

  \sa init()

*/
void vpDisplayX::closeDisplay()
{
  if (Xinitialise)
  {
    if (ximage_data_init == true)
      free(Ximage->data);

    Ximage->data = NULL;
    XDestroyImage(Ximage);

    XFreeGC(display, context);
    XDestroyWindow(display, window);
    XCloseDisplay(display);

    Xinitialise = false;
    if (title != NULL)    {
      delete [] title ;
      title = NULL ;
    }

  }
  else
  {
    if (title != NULL)    {
      delete [] title ;
      title = NULL ;
    }
  }
}




/*!
  \brief flush the GTK buffer
  It's necessary to use this function to see the results of any drawing

*/
void vpDisplayX::flushDisplay()
{
  if (Xinitialise)
  {
    XFlush(display);
    XSync(display,1);
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}


/*!

*/
void vpDisplayX::clearDisplay(int c)
{
  if (Xinitialise)
  {
    XSetWindowBackground (display, window, x_color[c]);
    XClearWindow(display, window);
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}

/*!
  \brief display a point
  \param i,j (row,colum indexes)
  \param color (see vpColor)
*/
void vpDisplayX::displayPoint(int i, int j, int col)
{
  if (Xinitialise)
  {
    XSetForeground (display, context, x_color[col]);
    XDrawPoint(display, window,context, j, i) ;
  }
 else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}


/*!
  \brief display a line
  \param i1,j1 (row,colum indexes) initial coordinates
  \param i2,j2 (row,colum indexes) final coordinates
  \param color (see vpColor)
  \param e : line_width
*/
void vpDisplayX::displayLine(int i1, int j1, int i2, int j2, int col, int e)
{
  if (Xinitialise)
  {
    if (e == 1) e = 0;

    XSetForeground (display, context, x_color[col]);
    XSetLineAttributes (display, context, e,
			LineSolid, CapButt, JoinBevel);

    XDrawLine (display, window,context, j1, i1, j2, i2);
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}

/*!
  \brief display a dashed line
  \param i1,j1 (row,colum indexes) initial coordinates
  \param i2,j2 (row,colum indexes) final coordinates
  \param color (see vpColor)
  \param e : line_width
*/
void vpDisplayX::displayDotLine(int i1, int j1, int i2, int j2, int col, int e)
{

  if (Xinitialise)
  {
    if (e == 1) e = 0;

    XSetForeground (display, context, x_color[col]);
    XSetLineAttributes (display, context, e,
			LineOnOffDash, CapButt, JoinBevel);

    XDrawLine (display, window,context, j1, i1, j2, i2);
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}

/*!
  \brief display a cross
  \param i,j (row,colum indexes)
  \param size of the cross
  \param color (see vpColor)
*/
void vpDisplayX::displayCross(int i,int j, int size,int col)
{
  if (Xinitialise)
  {
    try{
      displayLine(i-size/2,j,i+size/2,j,col,1) ;
      displayLine(i ,j-size/2,i,j+size/2,col,1) ;
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }
  }

  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }

}


/*!
  \brief display a "large" cross
  \param i,j (row,colum indexes)
  \param size of the cross
  \param color (see vpColor)
*/
void vpDisplayX::displayCrossLarge(int i,int j, int size,int col)
{
  if (Xinitialise)
  {
    try{
      displayLine(i-size/2,j,i+size/2,j,col,3) ;
      displayLine(i ,j-size/2,i,j+size/2,col,3) ;
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;    //
  }
}


/*!
  \brief display an arrow
  \param i1,j1 (row,colum indexes) initial coordinates
  \param i2,j2 (row,colum indexes) final coordinates
  \param color (see vpColor)
  \param L,l : width and height of the arrow
*/
void vpDisplayX::displayArrow(int i1,int j1, int i2, int j2, int col, int L,int l)
{
  if (Xinitialise)
  {
    try{
      double a = i2 - i1 ;
      double b = j2 - j1 ;
      double lg = sqrt(vpMath::sqr(a)+vpMath::sqr(b)) ;

      if ((a==0)&&(b==0))
      {
	// DisplayCrossLarge(i1,j1,3,col) ;
      }
      else
      {
	a /= lg ;
	b /= lg ;

	double i3,j3  ;
	i3 = i2 - L*a ;
	j3 = j2 - L*b ;


	double i4,j4 ;

	double t = 0 ;
	while (t<=l)
	{
	  i4 = i3 - b*t ;
	  j4 = j3 + a*t ;

	  displayLine((int)i2,(int)j2,(int)i4,(int)j4,col) ;
	  t+=0.1 ;
	}
	t = 0 ;
	while (t>= -l)
	{
	  i4 = i3 - b*t ;
	  j4 = j3 + a*t ;

	  displayLine((int)i2,(int)j2,(int)i4,(int)j4,col) ;
	  t-=0.1 ;
	}
	displayLine(i1,j1,i2,j2,col) ;

      }
    }
    catch(...)
    {
      vpERROR_TRACE(" ") ;
      throw ;
    }
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}


/*!
  \brief display a rectangle
  \param i,j (row,colum indexes) up left corner
  \param width
  \param height
  \param color (see vpColor)
*/
void
vpDisplayX::displayRectangle(int i, int j, int width, int height, int col)
{
  if (Xinitialise)
  {
    XSetForeground (display, context, x_color[col]);
    XSetLineAttributes (display, context, 0,
			LineSolid, CapButt, JoinBevel);

    XDrawRectangle (display, window, context,  j, i, width, height);
  }
 else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}



/*!
  \brief display a string
  \param i,j (row,colum indexes)
  \param string
  \param color (see vpColor)
*/
void vpDisplayX::displayCharString(int i, int j, char *string, int col)
{
  if (Xinitialise)
  {
    XSetForeground (display, context, x_color[col]);
    XDrawString( display, window, context, j, i, string, strlen(string) );
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}

/*!
  \brief wait for and get the position of the click
  \param i,j (row,colum indexes)

*/

bool
vpDisplayX::getClick(int& i, int& j)
{

  bool ret = false;
  if (Xinitialise)
  {
    int x,y ;
    while (ret==false)
    {
      Window	rootwin, childwin ;
      int		root_x, root_y, win_x, win_y ;
      unsigned int	modifier ;
      // Test d'évènements.
      if ( XPending(display) )  {
	XNextEvent(display, &event);

	/* Detection de l'appui sur l'un des bouton de la souris. */
	switch(event.type) {

	case ButtonPress:
	  {
	    /* Recuperation de la coordonnee du pixel cliqué.	*/
	    if(XQueryPointer(display,
			     window,
			     &rootwin, &childwin,
			     &root_x, &root_y,
			     &win_x, &win_y,
			     &modifier)) {
	      x = event.xbutton.x;
	      y = event.xbutton.y;
	      i = y ;
	      j = x ;
	    }
	    ret = true ;
	    break;

	  } /* Fin case ButtonPress	*/
	} /* Fin switch type d'evenement.	*/
      }
    }
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
  return ret ;
}


/*!
  wait for a click
*/
void
vpDisplayX::getClick()
{

  bool ret = false;

  if (Xinitialise)
  {
    while (ret==false)
    {
      Window	rootwin, childwin ;
      int		root_x, root_y, win_x, win_y ;
      unsigned int	modifier ;

      // Test d'évènements.
      if ( XPending(display) )
      {

	XNextEvent(display, &event);

	/* Detection de l'appui sur l'un des bouton de la souris. */
	switch(event.type)
	{

	case ButtonPress:
	  {
	    /* Recuperation de la coordonnee du pixel cliqué.	*/
	    if(XQueryPointer(display,
			     window,
			     &rootwin, &childwin,
			     &root_x, &root_y,
			     &win_x, &win_y,
			     &modifier)){}
	    ret = true ;
	    break;
	  } /* Fin case ButtonPress	*/
	} /* Fin switch type d'evenement.	*/
      }
    }
    //   return(OK);
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
  return ;//ret;
}

/*!
  \brief wait for and get the position of the click of the button specified
  by "button"
  \param i,j (row,colum indexes)

*/
bool
vpDisplayX::getClick(int& i, int& j, int& button)
{

  if (Xinitialise)
  {
    int x,y ;
    Window	rootwin, childwin ;
    int		root_x, root_y, win_x, win_y ;
    unsigned int	modifier ;

    // Test d'évènements.
    if ( XPending(display) )  {

      XNextEvent(display, &event);

      /* Detection de l'appui sur l'un des bouton de la souris. */
      switch(event.type) {

      case ButtonPress: {
	/* Recuperation de la coordonnee du pixel cliqué.	*/
	if(XQueryPointer(display,
			 window,
			 &rootwin, &childwin,
			 &root_x, &root_y,
			 &win_x, &win_y,
			 &modifier)) {
	  x = event.xbutton.x;
	  y = event.xbutton.y;
	  i = y ;
	  j = x ;
	  switch(event.xbutton.button)
	  {
	  case Button1: button = vpDisplay::button1; break;
	  case Button2: button = vpDisplay::button2; break;
	  case Button3: button = vpDisplay::button3; break;
	  }
	}
	return true;
	break;

      } /* Fin case ButtonPress	*/
      } /* Fin switch type d'evenement.	*/
    }
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
  return  false;
}

/*!
  \brief wait for and get the position of the click release of the
  button specified by "button". Same method as getClick(int&, int&, int&).
  \param i,j (row,colum indexes)

  \sa getClick(int&, int&, int&)

*/
bool
vpDisplayX::getClickUp(int& i, int& j, int& button)
{
  return getClick(i, j, button);
}

/*!
  \brief get the window depth (8,16,24,32)

  usualy it 24 bits now...
*/
int vpDisplayX::getScreenDepth()
{
  Display	*_display;
  int		screen;
  int		depth;

  if ((_display = XOpenDisplay(NULL)) == NULL) {
    vpERROR_TRACE("Can't connect display on server %s.",
	    XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError,
			     "Can't connect display on server.")) ;
  }
  screen = DefaultScreen(_display);
  depth  = DefaultDepth(_display, screen);

  XCloseDisplay(_display);

  return (depth);
}

/*!
  \brief get the window size
 */
void vpDisplayX::getScreenSize(int *xsize, int *ysize)
{
  Display	*_display;
  int		screen;

  if ((_display = XOpenDisplay(NULL)) == NULL)
  {
    vpERROR_TRACE("Can't connect display on server %s.",
		XDisplayName(NULL));
    throw(vpDisplayException(vpDisplayException::connexionError,
			     "Can't connect display on server.")) ;
  }
  screen = DefaultScreen(_display);
  *xsize = DisplayWidth (_display, screen);
  *ysize = DisplayHeight(_display, screen);

  XCloseDisplay(_display);
}




/*!
  \brief set the window title
 */
void
vpDisplayX::flushTitle(const char *windowtitle)
{
  if (Xinitialise)
  {
    XStoreName (display, window, windowtitle);
  }
  else
  {
    vpERROR_TRACE("X not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "X not initialized")) ;
  }
}


/*!
  \brief Display a circle
  \param i,j : circle center position (row,column)
  \param r : radius
  \param color
*/
void vpDisplayX::displayCircle(int i, int j, int r, int c)
{
   if (Xinitialise)
   {
     XSetForeground (display, context, x_color[c]);
     XSetLineAttributes (display, context, 0,
                        LineSolid, CapButt, JoinBevel);

     XDrawArc (display, window, context,  j-r, i-r, r*2, r*2, 0, 360*64);
   }
  else
   {
     vpERROR_TRACE("X not initialized " ) ;
     throw(vpDisplayException(vpDisplayException::notInitializedError,
			      "X not initialized")) ;
   }
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
