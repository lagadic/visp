/****************************************************************************
 *
 * $Id: vpDisplayGTK.cpp,v 1.40 2008-06-13 13:37:37 asaunier Exp $
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
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file vpDisplayGTK.cpp
  \brief Define the GTK console to display images
*/

#include <visp/vpConfig.h>

#if ( defined(VISP_HAVE_GTK) )

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
                           const char *_title) : vpDisplay()
{
  col = NULL;
  title = NULL ;
  window = NULL ;
  init(I,_x,_y, _title) ;
}


/*!
  \brief constructor : initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled
*/
vpDisplayGTK::vpDisplayGTK(vpImage<vpRGBa> &I,
                           int _x,
                           int _y,
                           const char *_title)
{
  col = NULL;
  title = NULL ;
  window = NULL ;
  init(I,_x,_y, _title) ;
}



/*!
  \brief constructor

  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled
*/
vpDisplayGTK::vpDisplayGTK(int _x, int _y, const char *_title)
{
  windowXPosition = _x ;
  windowYPosition = _y ;

  col = NULL;
  title = NULL ;
  window = NULL ;

  if (_title != NULL)
  {
    title = new char[strlen(_title) + 1] ; // Modif Fabien le 19/04/02
    strcpy(title,_title) ;
  }

  GTKinitialized = false ;
}

/*!
  \brief basic constructor
  \warning must an init member of the vpDisplayGTK function to be useful
  \sa init()
*/
vpDisplayGTK::vpDisplayGTK()
{
  windowXPosition = windowYPosition = -1 ;

  col = NULL;
  title = NULL ;
  window = NULL ;
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
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled

*/
void
vpDisplayGTK::init(vpImage<unsigned char> &I,
                   int _x,
                   int _y,
                   const char *_title)
{

  if ((I.getHeight() == 0) || (I.getWidth()==0))
  {
    vpERROR_TRACE("Image not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "Image not initialized")) ;
  }
  init (I.getWidth(), I.getHeight(), _x, _y, _title) ;
  I.display = this ;
  GTKinitialized = true ;
}

/*!
  \brief Initialized the display of a RGBa  image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled

*/
void
vpDisplayGTK::init(vpImage<vpRGBa> &I,
                   int _x,
                   int _y,
                   const char *_title)
{
  if ((I.getHeight() == 0) || (I.getWidth()==0))
  {
    vpERROR_TRACE("Image not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "Image not initialized")) ;
  }

  init (I.getWidth(), I.getHeight(), _x, _y, _title) ;
  I.display = this ;
  GTKinitialized = true ;
}
/*!
  \brief actual member used to Initialize the display of a
  gray level or RGBa  image

  \param width, height : width, height of the window
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled

*/
void
vpDisplayGTK::init(unsigned int width, unsigned int height,
                   int _x, int _y,
                   const char *_title)
{
  /* Initialisation of thegdk et gdk_rgb library */
  int *argc=NULL ;
  char **argv ;
  gdk_init(argc,&argv);
  gdk_rgb_init();

  this->width  = width;
  this->height = height;

  GdkWindowAttr attr ;
  attr.x = _x;
  attr.y = _y ;
  attr.width = width ;
  attr.height = height ;
  attr.wclass =  GDK_INPUT_OUTPUT ;
  attr.window_type = GDK_WINDOW_TOPLEVEL ;

  attr.event_mask =
    GDK_BUTTON_PRESS_MASK |
    GDK_BUTTON_RELEASE_MASK;

  int attributes_mask = GDK_WA_X | GDK_WA_Y ;
  /* Create the window*/
  window = gdk_window_new(NULL, &attr, attributes_mask);
  gdk_window_show(window);


  /* Create background pixmap */
  background = gdk_pixmap_new(window,width,height,-1);

  /* Create graphic context */
  gc = gdk_gc_new(window);

  /* get the colormap  */
  GdkColormap  *colormap;
  colormap = gdk_window_get_colormap(window);


  col = new GdkColor *[vpColor::none] ;

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
  col[vpColor::orange] = &orange ;

  gdk_color_parse("white",&white);
  gdk_colormap_alloc_color(colormap,&white,FALSE,TRUE);
  col[vpColor::white] = &white ;

  gdk_color_parse("black",&black);
  gdk_colormap_alloc_color(colormap,&black,FALSE,TRUE);
  col[vpColor::black] = &black ;

  /* Chargement des polices */
  Police1 = gdk_font_load("-*-times-medium-r-normal-*-16-*-*-*-*-*-*-*");
  Police2 = gdk_font_load("-*-courier-bold-r-normal-*-*-140-*-*-*-*-*-*");

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
void vpDisplayGTK::displayImage(const vpImage<unsigned char> &I)
{

  if (GTKinitialized)
  {


    /* Copie de l'image dans le pixmap fond */
    gdk_draw_gray_image(background,
                        gc,0, 0, width, height,
                        GDK_RGB_DITHER_NONE,
                        I.bitmap,
                        width);

    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(window, background, FALSE);

    /* Affichage */
    //gdk_window_clear(window);
    //gdk_flush();
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
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
void vpDisplayGTK::displayImage(const vpImage<vpRGBa> &I)
{

  if (GTKinitialized)
  {

    /* Copie de l'image dans le pixmap fond */
    gdk_draw_rgb_32_image(background,
                          gc,0, 0,  width, height,
                          GDK_RGB_DITHER_NONE,
                          (unsigned char *)I.bitmap,
                          4* width);

    /* Permet de fermer la fenêtre si besoin (cas des séquences d'images) */
    //while (g_main_iteration(FALSE));

    /* Le pixmap background devient le fond de la zone de dessin */
    gdk_window_set_back_pixmap(window, background, FALSE);

    /* Affichage */
    //gdk_window_clear(window);
    //flushDisplay() ;

  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
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
  // doesn't work
  if (GTKinitialized)
  {

    GdkImage *ImageGtk;
    /*
     */

    ImageGtk = gdk_image_get(background,0,0,width,height);


    I.resize(height,width) ;
    guchar *pos;
    guint32 pixel;
    guint32 x,y;
    guchar OctetRouge,OctetVert,OctetBleu,mask;
    mask = 0x000000FF;

    pos = (unsigned char *)I.bitmap;
    for (y=0;y<height;y++)
    {
      for (x=0;x<width;x++)
      {
        pixel = gdk_image_get_pixel(ImageGtk,x,y);
        OctetBleu  = (guchar)pixel & mask;
        OctetVert  = (guchar)(pixel>>8) & mask;
        OctetRouge = (guchar)(pixel>>16) & mask;
        *pos++ = OctetRouge;
        *pos++ = OctetVert;
        *pos++ = OctetBleu;
        *pos++ = 0;
      }
    }


  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }

}

/*!
  \brief not implemented

  \sa init(), closeDisplay()
*/
void vpDisplayGTK::displayImage(const unsigned char * /* I */)
{
  vpTRACE(" not implemented ") ;
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

  if (window != NULL)
  {
    gdk_window_hide (window);
    gdk_window_destroy(window);
    window = NULL;
  }
  GTKinitialized= false;
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
    gdk_flush();
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
}


/*!
  \brief not implemented
*/
void vpDisplayGTK::clearDisplay(vpColor::vpColorType /* c */)
{
  vpTRACE("Not implemented") ;
}

/*!
  \brief display a point
  \param i,j (row,colum indexes)
  \param color (see vpColor)
*/
void vpDisplayGTK::displayPoint(int i, int j,
                                vpColor::vpColorType color)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    gdk_draw_point(background,gc,j,i) ;
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
}

/*!
  \brief display a line
  \param i1,j1 (row,colum indexes) initial coordinates
  \param i2,j2 (row,colum indexes) final coordinates
  \param color (see vpColor)
  \param e : line thick
*/
void
vpDisplayGTK::displayLine(int i1, int j1, int i2, int j2,
                          vpColor::vpColorType color,
                          unsigned int e)
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
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
}


/*!
  \brief display a dashed line
  \param i1,j1 : (row,colum indexes) initial coordinates
  \param i2,j2 : (row,colum indexes) final coordinates
  \param color : (see vpColor)
  \param e : line thick
*/
void
vpDisplayGTK::displayDotLine(int i1, int j1, int i2, int j2,
                             vpColor::vpColorType color,
                             unsigned int e)
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
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
}


/*!
  \brief display a cross
  \param i,j : (row,colum indexes)
  \param size : size of the cross
  \param col : color (see vpColor)
*/
void
vpDisplayGTK::displayCross(int i, int j,
                           unsigned int size,
                           vpColor::vpColorType col)
{
  if (GTKinitialized)
  {
    try{
      displayLine(i-size/2,j,i+size/2,j,col,1) ;
      displayLine(i ,j-size/2,i,j+size/2,col,1) ;
    }
    catch (...)
    {
      vpERROR_TRACE("Error caught") ;
      throw ;
    }
  }

  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }

}


/*!
  \brief display a "large" cross
  \param i,j : (row,colum indexes)
  \param size : size of the cross
  \param col : color (see vpColor)
*/
void vpDisplayGTK::displayCrossLarge(int i, int j,
                                     unsigned int size,
                                     vpColor::vpColorType col)
{
  if (GTKinitialized)
  {
    try{
      displayLine(i-size/2,j,i+size/2,j,col,3) ;
      displayLine(i ,j-size/2,i,j+size/2,col,3) ;
    }
    catch (...)
    {
      vpERROR_TRACE("Error caught") ;
      throw ;
    }
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;    //
  }
}



/*!
  \brief display an arrow
  \param i1,j1 : (row,colum indexes) initial coordinates
  \param i2,j2 : (row,colum indexes) final coordinates
  \param col : color (see vpColor)
  \param L,l : width and height of the arrow
*/
void
vpDisplayGTK::displayArrow(int i1, int j1, int i2, int j2,
                           vpColor::vpColorType col,
                           unsigned int L,unsigned int l)
{
  if (GTKinitialized)
  {
    try{
      int _l = l;
      double a = (int)i2 - (int)i1 ;
      double b = (int)j2 - (int)j1 ;
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

        //double t = 0 ;
        //while (t<=_l)
        {
          i4 = i3 - b*_l ;
          j4 = j3 + a*_l ;

          displayLine(i2, j2, (unsigned int) i4,
                      (unsigned int) j4, col) ;
          //t+=0.1 ;
        }
        //t = 0 ;
        //while (t>= -_l)
        {
          i4 = i3 + b*_l ;
          j4 = j3 - a*_l ;

          displayLine(i2, j2, (unsigned int) i4,
                      (unsigned int) j4, col) ;
          //t-=0.1 ;
        }
        displayLine(i1,j1,i2,j2,col) ;

      }
    }
    catch (...)
    {
      vpERROR_TRACE("Error caught") ;
      throw ;
    }
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
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
  \param fill : set as true to fill the rectangle.
  \param e : line thick
*/
void
vpDisplayGTK::displayRectangle(int i, int j,
                               unsigned int width, unsigned int height,
                               vpColor::vpColorType color, bool fill,
                               unsigned int e)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    if (e>1)
      gdk_gc_set_line_attributes(gc, e, GDK_LINE_SOLID, GDK_CAP_BUTT,
                                 GDK_JOIN_BEVEL) ;

    if (fill == false)
      gdk_draw_rectangle(background,gc,FALSE,j,i,width-1,height-1);
    else
      gdk_draw_rectangle(background,gc,TRUE,j,i,width,height);

    if (e>1)
      gdk_gc_set_line_attributes(gc, 0, GDK_LINE_SOLID, GDK_CAP_BUTT,
                                 GDK_JOIN_BEVEL) ;
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
}

/*!
  \brief display a rectangle
  \param rect : Rectangle characteristics.
  \param color : Color (see vpColor)
  \param fill : set as true to fill the rectangle.
  \param e : line thick
*/
void
vpDisplayGTK::displayRectangle(const vpRect &rect,
                               vpColor::vpColorType color, bool fill,
                               unsigned int e)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    if (e>1)
      gdk_gc_set_line_attributes(gc, e, GDK_LINE_SOLID, GDK_CAP_BUTT,
                                 GDK_JOIN_BEVEL) ;

    if (fill == false)
      gdk_draw_rectangle(background,gc,FALSE,
                         (int)rect.getLeft(), (int)rect.getTop(),
                         (int)rect.getWidth()-1, (int)rect.getHeight()-1);
    else
      gdk_draw_rectangle(background,gc,TRUE,
                         (int)rect.getLeft(), (int)rect.getTop(),
                         (int)rect.getWidth(), (int)rect.getHeight());

    if (e>1)
      gdk_gc_set_line_attributes(gc, 0, GDK_LINE_SOLID, GDK_CAP_BUTT,
                                 GDK_JOIN_BEVEL) ;
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
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
void vpDisplayGTK::displayCharString(int i, int j,
                                     const char *string, vpColor::vpColorType color)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    gdk_draw_string(background,Police2,gc,j,i,(const gchar *)string);

  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
}

/*!
  Wait for a click from one of the mouse button.

  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool
vpDisplayGTK::getClick(bool blocking)
{
  bool ret = false;

  if (GTKinitialized) {

//    flushDisplay() ;
    GdkEvent *ev = NULL;
    do {
      while ((ev = gdk_event_get())!=NULL){
        if (ev->any.window == window && ev->type == GDK_BUTTON_PRESS){
          ret = true ;
        }
        gdk_event_free(ev) ;
      }
      if (blocking){
        flushDisplay();
        vpTime::wait(10);
      }
    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
  return ret;
}

/*!
  Wait for a mouse button click and get the position of the clicked pixel.

  \param i,j : Position of the clicked pixel (row, colum indexes).

  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.

*/
bool
vpDisplayGTK::getClick(unsigned int& i, unsigned int& j, bool blocking)
{
  bool ret = false;

  if (GTKinitialized) {

    GdkEvent *ev = NULL;
    do {
      while ((ev = gdk_event_get())!=NULL){
        if (ev->any.window == window && ev->type == GDK_BUTTON_PRESS) {
          i = (unsigned int)((GdkEventButton *)ev)->y ;
          j = (unsigned int)((GdkEventButton *)ev)->x ;
          ret = true ;
        }
        gdk_event_free(ev) ;
      }
      if (blocking){
        flushDisplay();
        vpTime::wait(10);
      }
    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
  return ret ;
}


/*!

  Wait for a mouse button click and get the position of the clicked
  pixel. The button used to click is also set.

  \param i,j : Position of the clicked pixel (row, colum indexes).

  \param button : Button used to click.

  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.

*/
bool
vpDisplayGTK::getClick(unsigned int& i, unsigned int& j,
                       vpMouseButton::vpMouseButtonType& button,
                       bool blocking)
{
  bool ret = false;

  if (GTKinitialized) {
    GdkEvent *ev = NULL;
    do {
      while ((ev = gdk_event_get())){
        if (ev->any.window == window && ev->type == GDK_BUTTON_PRESS){
          i = (unsigned int)((GdkEventButton *)ev)->y ;
          j = (unsigned int)((GdkEventButton *)ev)->x ;

          switch ((int)((GdkEventButton *)ev)->button) {
            case 1:
              button = vpMouseButton::button1; break;
            case 2:
              button = vpMouseButton::button2; break;
            case 3:
              button = vpMouseButton::button3; break;
          }
          ret = true ;
        }
        gdk_event_free(ev) ;
      }
      if (blocking){
        flushDisplay();
        vpTime::wait(10);
      }

    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
  return ret;
}

/*!

  Wait for a mouse button click release and get the position of the
  pixel were the click release occurs.  The button used to click is
  also set.

  \param i,j : Position of the clicked pixel (row, colum indexes).

  \param button : Button used to click.

  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return
  - true if a button was clicked. This is always the case if blocking is set
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.

*/
bool
vpDisplayGTK::getClickUp(unsigned int& i, unsigned int& j,
                         vpMouseButton::vpMouseButtonType& button,
                         bool blocking)
{
  bool ret = false;

  if ( GTKinitialized ) {

    flushDisplay() ;
    GdkEvent *ev = NULL;
    do {
      while ((ev = gdk_event_get())!=NULL){
        if ( ev->any.window == window  && ev->type == GDK_BUTTON_RELEASE) {
          i = ( unsigned int ) ( ( GdkEventButton * ) ev )->y ;
          j = ( unsigned int ) ( ( GdkEventButton * ) ev )->x ;

          switch ( ( int ) ( ( GdkEventButton * ) ev )->button ) {
            case 1:
              button = vpMouseButton::button1; break;
            case 2:
              button = vpMouseButton::button2; break;
            case 3:
              button = vpMouseButton::button3; break;
          }
          ret = true ;
        }
        gdk_event_free(ev) ;
      }
      if (blocking){
        flushDisplay();
        vpTime::wait(10);
      }

    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE ( "GTK not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "GTK not initialized" ) ) ;
  }
  return ret;
}

/*!
  \brief get the window depth (8,16,24,32)

  usualy it 24 bits now...
*/
unsigned int vpDisplayGTK::getScreenDepth()
{

  unsigned int depth;

  depth = gdk_window_get_visual(window)->depth ;

  return (depth);
}

/*!
  \brief get the window size

  \warning Not implemented
*/
void vpDisplayGTK::getScreenSize(unsigned int &width, unsigned int &height)
{
  vpTRACE("Not implemented") ;
  width = 0;
  height = 0;
}





/*!
  \brief set the window title
*/
void
vpDisplayGTK::flushTitle(const char *windowtitle)
{
  if (GTKinitialized)
  {
    if (windowtitle != NULL)
      gdk_window_set_title(window,(char *)windowtitle);
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
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
void vpDisplayGTK::displayCircle(int i, int j,
                                 unsigned int r,
                                 vpColor::vpColorType color)
{
  if (GTKinitialized)
  {
    gdk_gc_set_foreground(gc,col[color]);
    gdk_draw_arc(background,gc,FALSE,j-r,i-r,2*r,2*r,360*64,360*64) ;
  }
  else
  {
    vpERROR_TRACE("GTK not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "GTK not initialized")) ;
  }
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
