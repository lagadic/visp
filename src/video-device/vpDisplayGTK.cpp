
/*                                                                -*-c++-*-
    Copyright (C) 1998  IRISA-INRIA Rennes Lagadic Project

    Contact:
       Eric Marchand
       IRISA-INRIA Rennes
       35042 Rennes Cedex
       France

    email: marchand@irisa.fr
    www  : http://www.irisa.fr/vista

    file vpDisplayGTK.cpp

    Main Auteur :
      Christophe Collewet


*/

/*!
  \file vpDisplayGTK.cpp
  \brief Define the GTK console to display images
*/

/*!
  \class vpDisplayGTK

  \brief The  vpDisplayGTK allows to display image using the GTK+ library
  version 1.2.

  \author Christophe Collewet (Christophe.Collewet@irisa.fr),
  imported in ViSP by Eric Marchand (Eric.Marchand@irisa.fr)
  Irisa / Inria Rennes

  The GTK+ 1.2 library has to be available on the system
*/

#include <visp/vpConfig.h>

#ifdef HAVE_GTK

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// Display stuff
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGTK.h>

//debug / exception
#include <visp/vpDebug.h>
#include <visp/vpDisplayException.h>

/*!

  \brief constructor : initialize a display to visualize a gray level image
  (8 bits).

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
vpDisplayGTK::vpDisplayGTK(vpImage<unsigned char> &I,
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
vpDisplayGTK::vpDisplayGTK(vpImage<vpRGBa> &I,
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
vpDisplayGTK::vpDisplayGTK(int _x, int _y, char *_title)
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



}
/*!
  \brief basic constructor
  \warning must an init member of the vpDisplayGTK function to be useful
  \sa init()
*/
vpDisplayGTK::vpDisplayGTK()
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

  GTKinitialized = false ;
}

/*!
  \brief close the window
*/
vpDisplayGTK::~vpDisplayGTK()
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
vpDisplayGTK::init(vpImage<unsigned char> &I,
		   int _x,
		   int _y,
		   char *_title)
 {

   if ((I.getRows() == 0) || (I.getCols()==0))
     {
       ERROR_TRACE("Image not initialized " ) ;
       throw(vpDisplayException(vpDisplayException::notInitializedError,
				"Image not initialized")) ;
     }
   init (I.getCols(), I.getRows(), _x, _y, _title) ;
   I.display = this ;
   I.initDisplay =  true ;
   GTKinitialized = true ;
 }

/*!
  \brief Initialized the display of a RGBa  image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
void
vpDisplayGTK::init(vpImage<vpRGBa> &I,
		   int _x,
		   int _y,
		   char *_title)
{
  if ((I.getRows() == 0) || (I.getCols()==0))
     {
       ERROR_TRACE("Image not initialized " ) ;
       throw(vpDisplayException(vpDisplayException::notInitializedError,
				"Image not initialized")) ;
     }

  init (I.getCols(), I.getRows(), _x, _y, _title) ;
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
void
vpDisplayGTK::init(int _ncol, int _nrow,
		   int _x, int _y,
		   char *_title)
 {

  displayHasBeenInitialized =true ;

  /* Initialisation of thegdk et gdk_rgb library */
  int *argc=NULL ;
  char **argv ;
  gdk_init(argc,&argv);
  gdk_rgb_init();

  ncol = _ncol ;
  nrow = _nrow ;

  GdkWindowAttr attr ;
  attr.y = _x;
  attr.x = _y ;
  attr.width = ncol ;
  attr.height = nrow ;
  attr.wclass =  GDK_INPUT_OUTPUT ;
  attr.window_type = GDK_WINDOW_TOPLEVEL ;

  attr.event_mask =
    GDK_BUTTON_PRESS_MASK |
    GDK_BUTTON_RELEASE_MASK ;

  /* Create the window*/
  window = gdk_window_new(NULL, &attr,0);
  gdk_window_show(window);


  /* Create background pixmap */
  background = gdk_pixmap_new(window,ncol,nrow,-1);

  /* Create graphic context */
  gc = gdk_gc_new(window);

  /* get the colormap  */
  GdkColormap		*colormap;
  colormap = gdk_window_get_colormap(window);


  col = new GdkColor *[10] ;

  /* Create color */
  gdk_color_parse("blue",&blue);
  gdk_colormap_alloc_color(colormap,&blue,FALSE,TRUE);
  col[vpColor::blue] = &blue ;

  gdk_color_parse("red",&red);
  gdk_colormap_alloc_color(colormap,&red,FALSE,TRUE);
  col[vpColor::red] = &red ;

  gdk_color_parse("green",&green);
  gdk_colormap_alloc_color(colormap,&green,FALSE,TRUE);
  col[vpColor::green] = &green ;

  gdk_color_parse("yellow",&yellow);
  gdk_colormap_alloc_color(colormap,&yellow,FALSE,TRUE);
  col[vpColor::yellow] = &yellow ;

  gdk_color_parse("cyan",&cyan);
  gdk_colormap_alloc_color(colormap,&cyan,FALSE,TRUE);
  col[vpColor::cyan] = &cyan ;

  gdk_color_parse("magenta",&magenta);
  gdk_colormap_alloc_color(colormap,&magenta,FALSE,TRUE);

  gdk_color_parse("goldenrod",&goldenrod);
  gdk_colormap_alloc_color(colormap,&goldenrod,FALSE,TRUE);

  gdk_color_parse("coral",&coral);
  gdk_colormap_alloc_color(colormap,&coral,FALSE,TRUE);

  gdk_color_parse("orange",&orange);
  gdk_colormap_alloc_color(colormap,&orange,FALSE,TRUE);

  gdk_color_parse("white",&white);
  gdk_colormap_alloc_color(colormap,&white,FALSE,TRUE);
  col[vpColor::white] = &white ;

  /* Chargement des polices */
  Police1 = gdk_font_load("-*-times-medium-r-normal-*-16-*-*-*-*-*-*-*");
  Police2 = gdk_font_load("-*-courier-bold-r-normal-*-*-140-*-*-*-*-*-*");

  title = NULL ;

  if (_title != NULL)
  {
    if (title != NULL)
      {
	delete [] title ;
	title = NULL ;
      }
    title = new char[strlen(_title) + 1] ;
    strcpy(title,_title) ;
  }

  GTKinitialized = true ;
  flushTitle(title) ;


}



/*!
  \brief display the gray level image (8bits)

  GTK has to be initialized

  \warning suppres the overlay drawing

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImage(vpImage<unsigned char> &I)
{

  if (GTKinitialized)
  {


    /* Copie de l'image dans le pixmap fond */
    gdk_draw_gray_image(background,
			gc,0, 0, ncol, nrow,
			GDK_RGB_DITHER_NONE,
			I.bitmap,
			ncol);

    /* Permet de fermer la fenêtre si besoin (cas des séquences d'images) */
    while (g_main_iteration(FALSE));

    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(window, background, FALSE);

    /* Affichage */
    gdk_window_clear(window);
    gdk_window_show(window);
    gdk_flush();
  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}


/*!
  \brief display the RGBa level image (32bits)

  GTK has to be initialized

  \warning suppres the overlay drawing

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImage(vpImage<vpRGBa> &I)
{

  if (GTKinitialized)
  {

     /* Copie de l'image dans le pixmap fond */
    gdk_draw_rgb_32_image(background,
			  gc,0, 0,  ncol, nrow,
			  GDK_RGB_DITHER_NONE,
			  (unsigned char *)I.bitmap,
			  4* ncol);

    /* Permet de fermer la fenêtre si besoin (cas des séquences d'images) */
    while (g_main_iteration(FALSE));

    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(window, background, FALSE);

    /* Affichage */
    flushDisplay() ;

  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}

/*
  \brief gets the displayed image (including the overlay plane)
  and returns an RGBa image
*/
void vpDisplayGTK::getImage(vpImage<vpRGBa> &I)
{


  // shoudl certainly be optimized.
  // marche pas
  if (GTKinitialized)
  {

    GdkImage	*ImageGtk;
    /*
    */

    ImageGtk = gdk_image_get(background,0,0,ncol,nrow);


    I.resize(nrow,ncol) ;
    guchar	*pos;
    guint32	pixel;
    gint	x,y;
    guchar	OctetRouge,OctetVert,OctetBleu,mask;
    mask = 0x000000FF;

    pos = (unsigned char *)I.bitmap;
    for(y=0;y<nrow;y++)
      {
	for(x=0;x<ncol;x++)
	  {
	    pixel = gdk_image_get_pixel(ImageGtk,x,y);
	    OctetBleu  = pixel & mask;
	    OctetVert  = (pixel>>8) & mask;
	    OctetRouge = (pixel>>16) & mask;
	    *pos++ = OctetRouge;
	    *pos++ = OctetVert;
	    *pos++ = OctetBleu;
	    *pos++ = 0;
	  }
      }


  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }

}

/*!
  \brief not implemented

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImage(unsigned char *I)
{
  TRACE(" not implemented ") ;
}

/*!

  \brief close the window

  \sa init()

*/
void vpDisplayGTK::closeDisplay()
{
  if (col != NULL)
    {
      delete [] col ; col = NULL ;
    }
  if (title != NULL)
    {
      delete [] title ;
      title = NULL ;
    }
}


/*!
  \brief flush the GTK buffer
  It's necessary to use this function to see the results of any drawing

*/
void vpDisplayGTK::flushDisplay()
{
  if (GTKinitialized)
  {
    gdk_window_clear(window);
    gdk_window_show(window);
    gdk_flush();

  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}


/*!
  \brief not implemented
*/
void vpDisplayGTK::clearDisplay(int c)
{
  TRACE("Not implemented") ;
}

/*!
  \brief display a point
  \param i,j (row,colum indexes)
  \param color (see vpColor)
*/
void vpDisplayGTK::displayPoint(int i, int j, int color)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    gdk_draw_point(background,gc,j,i) ;
  }
 else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}

/*!
  \brief display a line
  \param i1,j1 (row,colum indexes) initial coordinates
  \param i2,j2 (row,colum indexes) final coordinates
  \param color (see vpColor)
  \param e : line_width
*/
void
vpDisplayGTK::displayLine(int i1, int j1,
			  int i2, int j2,
			  int color,
			  int e)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    if (e>1)
      gdk_gc_set_line_attributes(gc,e,
				 GDK_LINE_SOLID,GDK_CAP_BUTT,
				 GDK_JOIN_BEVEL) ;
    gdk_draw_line(background,gc,j1,i1,j2,i2) ;
     if (e>1)
      gdk_gc_set_line_attributes(gc,0,
				 GDK_LINE_SOLID,GDK_CAP_BUTT,
				 GDK_JOIN_BEVEL) ;

  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}


/*!
  \brief display a dashed line
  \param i1,j1 (row,colum indexes) initial coordinates
  \param i2,j2 (row,colum indexes) final coordinates
  \param color (see vpColor)
  \param e : line_width
*/
void
vpDisplayGTK::displayDotLine(int i1, int j1,
			     int i2, int j2,
			     int color,
			     int e)
{

  if (GTKinitialized)
  {
    if (e == 1) e = 0;

    gdk_gc_set_foreground(gc,col[color]);
    gdk_gc_set_line_attributes(gc,e
			       ,GDK_LINE_ON_OFF_DASH,GDK_CAP_BUTT,
			       GDK_JOIN_BEVEL) ;
    gdk_draw_line(background,gc,j1,i1,j2,i2) ;
    gdk_gc_set_line_attributes(gc,0,
			       GDK_LINE_SOLID,GDK_CAP_BUTT,
			       GDK_JOIN_BEVEL) ;

  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}


/*!
  \brief display a cross
  \param i,j (row,colum indexes)
  \param size of the cross
  \param color (see vpColor)
*/
void
vpDisplayGTK::displayCross(int i,int j,
			   int size,
			   int col)
{
  if (GTKinitialized)
  {
    try{
      displayLine(i-size/2,j,i+size/2,j,col,1) ;
      displayLine(i ,j-size/2,i,j+size/2,col,1) ;
    }
    catch(...)
    {
      ERROR_TRACE(" ") ;
      throw ;
    }
  }

  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }

}


/*!
  \brief display a "large" cross
  \param i,j (row,colum indexes)
  \param size of the cross
  \param color (see vpColor)
*/
void vpDisplayGTK::displayCrossLarge(int i,int j, int size,int col)
{
  if (GTKinitialized)
  {
    try{
      displayLine(i-size/2,j,i+size/2,j,col,3) ;
      displayLine(i ,j-size/2,i,j+size/2,col,3) ;
    }
    catch(...)
    {
      ERROR_TRACE(" ") ;
      throw ;
    }
  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;    //
  }
}



/*!
  \brief display an arrow
  \param i1,j1 (row,colum indexes) initial coordinates
  \param i2,j2 (row,colum indexes) final coordinates
  \param color (see vpColor)
  \param L,l : width and height of the arrow
*/
void
vpDisplayGTK::displayArrow(int i1,int j1,
			   int i2, int j2,
			   int col,
			   int L,int l)
{
  if (GTKinitialized)
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
      ERROR_TRACE(" ") ;
      throw ;
    }
  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
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
vpDisplayGTK::displayRectangle(int i, int j,
			       int width, int height,
			       int color)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    gdk_draw_rectangle(background,gc,FALSE,j,i,width,height);
  }
 else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}

/*!
  \brief display a string
  \param i,j (row,colum indexes)
  \param string
  \param color (see vpColor)
*/
void vpDisplayGTK::displayCharString(int i, int j, char *string, int color)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    gdk_draw_string(background,Police2,gc,j,i,(const gchar *)string);

  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}

/*!
  wait for a click
*/
void
vpDisplayGTK::getClick()
{

  if (GTKinitialized)
  {

    flushDisplay() ;
    bool end = false ;
    GdkEvent *ev ;

    while (!end)
      {
	ev = gdk_event_get() ;
	if (ev)
	  {
	    switch(ev->type)
	      {
	      case GDK_BUTTON_PRESS :
		end = true ;
		break ;
	      default :;
	      }
	  }
      }
    gdk_event_free(ev) ;
    //   return(OK);
  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}

/*!
  \brief wait for and get the position of the click
  \param i,j (row,colum indexes)

*/
bool
vpDisplayGTK::getClick(int& i, int& j)
{

  if (GTKinitialized)
  {

    flushDisplay() ;
    bool end = false ;
    GdkEvent *ev ;

    while (!end)
      {
	ev = gdk_event_get() ;
	if (ev)
	  {
	    switch(ev->type)
	      {
	      case GDK_BUTTON_PRESS :
		i = (int)((GdkEventButton *)ev)->y ;
		j = (int)((GdkEventButton *)ev)->x ;
		end = true ;
		break ;
	      default :;
	      }
	  }
      }
    gdk_event_free(ev) ;
  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
    return false ;
}




/*!
  \brief wait for and get the position of the click of the button specified
  by "button"
  \param i,j (row,colum indexes)

*/
bool
vpDisplayGTK::getClick(int& i, int& j, int& button)
{

  if (GTKinitialized)
  {

    flushDisplay() ;
    bool end = false ;
    GdkEvent *ev ;

    while (!end)
      {
	ev = gdk_event_get() ;
	if (ev)
	  {
	    switch(ev->type)
	      {
	      case GDK_BUTTON_PRESS :
		if ((int)((GdkEventButton *)ev)->button==button)
		  {
		    i = (int)((GdkEventButton *)ev)->y ;
		    j = (int)((GdkEventButton *)ev)->x ;
		    end = true ;
		  }
		break ;
	      default :;
	      }
	  }
      }
    gdk_event_free(ev) ;
  }
 else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
    return  false;
}

/*!
  \brief wait for and get the position of the click release of the
  button specified by "button"
  \param i,j (row,colum indexes)

*/
bool
vpDisplayGTK::getClickUp(int& i, int& j, int& button)
{

  if (GTKinitialized)
  {

    flushDisplay() ;
    bool end = false ;
    GdkEvent *ev ;

    while (!end)
      {
	ev = gdk_event_get() ;
	if (ev)
	  {
	    switch(ev->type)
	      {
	      case GDK_BUTTON_RELEASE :
		if ((int)((GdkEventButton *)ev)->button==button)
		  {
		    i = (int)((GdkEventButton *)ev)->y ;
		    j = (int)((GdkEventButton *)ev)->x ;
		    end = true ;
		  }
		break ;
	      default :;
	      }
	  }
      }
    gdk_event_free(ev) ;
  }
 else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
  return  false ;
}

/*!
  \brief get the window depth (8,16,24,32)

  usualy it 24 bits now...
*/
int vpDisplayGTK::getScreenDepth()
{

  int		depth;

  depth = gdk_window_get_visual(window)->depth ;

  return (depth);
}

/*!
  \brief get the window size
 */
void vpDisplayGTK::getScreenSize(int *xsize, int *ysize)
{
  TRACE("Not implemented") ;
}





/*!
  \brief set the window title
 */
void
vpDisplayGTK::flushTitle(const char *windowtitle)
{
  if (GTKinitialized)
  {
    gdk_window_set_title(window,(char *)windowtitle);
  }
  else
  {
    ERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
			     "GTK not initialized")) ;
  }
}

/*!
  \brief Display a circle
  \param i,j : circle center position (row,column)
  \param r : radius
  \param color
*/
void vpDisplayGTK::displayCircle(int i, int j, int r, int color)
{
   if (GTKinitialized)
   {
     gdk_gc_set_foreground(gc,col[color]);
     gdk_draw_arc(background,gc,FALSE,j-r,i-r,2*r,2*r,360*64,360*64) ;
   }
  else
   {
     ERROR_TRACE("GTK not initialized " ) ;
     throw(vpDisplayException(vpDisplayException::notInitializedError,
			      "GTK not initialized")) ;
   }
}

#endif // HAVE_GTK

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
