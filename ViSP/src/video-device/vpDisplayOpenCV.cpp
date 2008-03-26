/****************************************************************************
 *
 * $Id: vpDisplayOpenCV.cpp,v 1.1 2008-03-26 09:09:28 asaunier Exp $
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
  \file vpDisplayOpenCV.cpp
  \brief Define the OpenCV console to display images
*/

#include <visp/vpConfig.h>

#if ( defined(VISP_HAVE_OPENCV) )

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// Display stuff
#include <visp/vpDisplay.h>
#include <visp/vpDisplayOpenCV.h>

//debug / exception
#include <visp/vpDebug.h>
#include <visp/vpDisplayException.h>

int vpDisplayOpenCV::count = 1;
/*!

\brief constructor : initialize a display to visualize a gray level image
(8 bits).

\param I : image to be displayed (not that image has to be initialized)
\param _x, _y The window is set at position x,y (column index, row index).
\param _title  window  titled

*/
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<unsigned char> &I,
                                 int _x,
                                 int _y,
                                 char *_title) : vpDisplay()
{
  col = NULL;
  title = NULL ;
  window = 0 ;
  background = NULL;
  font = NULL;
  init(I,_x,_y, _title) ;
}


/*!
  \brief constructor : initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled
*/
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<vpRGBa> &I,
                                 int _x,
                                 int _y,
                                 char *_title)
{
  col = NULL;
  title = NULL ;
  window = 0 ;
  background = NULL;
  font = NULL;
  init(I,_x,_y, _title) ;
}



/*!
  \brief constructor

  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled
*/
vpDisplayOpenCV::vpDisplayOpenCV(int _x, int _y, char *_title)
{
  col = NULL;
  title = NULL ;
  window = 0 ;
  background = NULL;
  if (_title != NULL)
  {
    title = new char[strlen(_title) + 1] ; // Modif Fabien le 19/04/02
    strcpy(title,_title) ;
  }
  font = NULL;
  OpenCVinitialized = false ;
}

/*!
  \brief basic constructor
  \warning must an init member of the vpDisplayOpenCV function to be useful
  \sa init()
*/
vpDisplayOpenCV::vpDisplayOpenCV()
{
  windowXPosition = windowYPosition = -1 ;

  col = NULL;
  title = NULL ;
  window = 0 ;
  background = NULL;
  if (title != NULL)
  {
    delete [] title ;
    title = NULL ;
  }
  font = NULL;
  OpenCVinitialized = false ;
}

/*!
  \brief close the window
*/
vpDisplayOpenCV::~vpDisplayOpenCV()
{
  closeDisplay() ;
  cvReleaseImage(&background);
}

/*!
  \brief Initialized the display of a gray level image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled

*/
void
vpDisplayOpenCV::init(vpImage<unsigned char> &I,
                      int _x,
                      int _y,
                      char *_title)
{

  if ((I.getHeight() == 0) || (I.getWidth()==0))
  {
    vpERROR_TRACE("Image not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "Image not initialized")) ;
  }
  init (I.getWidth(), I.getHeight(), _x, _y, _title) ;
  I.display = this ;
  OpenCVinitialized = true ;
}

/*!
  \brief Initialized the display of a RGBa  image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled

*/
void
vpDisplayOpenCV::init(vpImage<vpRGBa> &I,
                      int _x,
                      int _y,
                      char *_title)
{
  if ((I.getHeight() == 0) || (I.getWidth()==0))
  {
    vpERROR_TRACE("Image not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "Image not initialized")) ;
  }

  init (I.getWidth(), I.getHeight(), _x, _y, _title) ;
  I.display = this ;
  OpenCVinitialized = true ;
}
/*!
  \brief actual member used to Initialize the display of a
  gray level or RGBa  image

  \param width, height : width, height of the window
  \param _x, _y : The window is set at position x,y (column index, row index).
  \param _title : window  titled

*/
void
vpDisplayOpenCV::init(unsigned int width, unsigned int height,
                      int _x, int _y,
                      char *_title)
{
  this->width  = width;
  this->height = height;
  this->windowXPosition = _x;
  this->windowYPosition = _y;
  int flags = CV_WINDOW_AUTOSIZE;
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
  else{
    if (title != NULL)
    {
      delete [] title ;
      title = NULL ;
    }
    title = new char[50] ;
    sprintf(title,"Unnamed ViSP display <%02d>",count) ;
  }
  count++;
  /* Create the window*/
  window = cvNamedWindow( title, flags );
  cvMoveWindow( title, _x, _y );
  lbuttondown = false;
  mbuttondown = false;
  rbuttondown = false;
  lbuttonup = false;
  mbuttonup = false;
  rbuttonup = false;
  cvSetMouseCallback( title,on_mouse, this );
  /* Create background pixmap */
//   background = cvCreateImage(cvSize((int)width,(int)height),IPL_DEPTH_8U,3);
//
//   cvShowImage( _title,background);

  col = new CvScalar[vpColor::none] ;

  /* Create color */
  col[vpColor::blue] = CV_RGB(0,0,255) ;
  col[vpColor::red] = CV_RGB(255,0,0) ;
  col[vpColor::green] = CV_RGB(0,255,0) ;
  col[vpColor::yellow] = CV_RGB(255,255,0) ;
  col[vpColor::cyan] = CV_RGB(0,255,255) ;
  col[vpColor::orange] = CV_RGB(255,128,0) ;
  col[vpColor::white] = CV_RGB(255,255,255) ;
  col[vpColor::black] = CV_RGB(0,0,0) ;

  font = new CvFont;
  cvInitFont( font, CV_FONT_HERSHEY_PLAIN, 1.0f,1.0f);

  CvSize fontSize;
  int baseline;
  cvGetTextSize( "A", font, &fontSize, &baseline );
  fontHeight = fontSize.height + baseline;
  OpenCVinitialized = true ;
}



/*!
  \brief display the gray level image (8bits)

  OpenCV has to be initialized

  \warning suppres the overlay drawing

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImage(const vpImage<unsigned char> &I)
{

  if (OpenCVinitialized)
  {
    vpImage<vpRGBa> Ic;
    vpImageConvert::convert(I,Ic);
    vpImageConvert::convert(Ic,background);
    /* Copie de l'image dans le pixmap fond */
    width = I.getWidth();
    height = I.getHeight();
    /* Le pixmap background devient le fond de la zone de dessin */

    /* Affichage */
    //gdk_window_clear(window);
    //gdk_flush();
  }
  else
  {
    vpERROR_TRACE("openCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}


/*!
  \brief display the RGBa level image (32bits)

  OpenCV has to be initialized

  \warning suppres the overlay drawing

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImage(const vpImage<vpRGBa> &I)
{

  if (OpenCVinitialized)
  {
    /* Copie de l'image dans le pixmap fond */

    vpImageConvert::convert(I,background);
    /* Copie de l'image dans le pixmap fond */
    width = I.getWidth();
    height = I.getHeight();

  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}

/*
  \brief gets the displayed image (including the overlay plane)
  and returns an RGBa image
*/
void vpDisplayOpenCV::getImage(vpImage<vpRGBa> &I)
{
  vpImageConvert::convert(background,I);
  // shoudl certainly be optimized.
}

/*!
  \brief not implemented

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImage(const unsigned char * /* I */)
{
  vpTRACE(" not implemented ") ;
}

/*!

\brief close the window

\sa init()

*/
void vpDisplayOpenCV::closeDisplay()
{
  if (col != NULL)
  {
    delete [] col ; col = NULL ;
  }
  if (font != NULL)
  {
    delete font ;
    font = NULL ;
  }

  if (window != 0)
  {
    cvDestroyWindow( title );
    count--;
    window = 0;
  }
  if (title != NULL)
  {
    delete [] title ;
    title = NULL ;
  }

  OpenCVinitialized= false;
}


/*!
  \brief flush the OpenCV buffer
  It's necessary to use this function to see the results of any drawing

*/
void vpDisplayOpenCV::flushDisplay()
{
  if (OpenCVinitialized)
  {
    cvShowImage(title, background );
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}


/*!
  \brief not implemented
*/
void vpDisplayOpenCV::clearDisplay(vpColor::vpColorType /* c */)
{
  vpTRACE("Not implemented") ;
}

/*!
  \brief display a point
  \param i,j (row,colum indexes)
  \param color (see vpColor)
*/
void vpDisplayOpenCV::displayPoint(int i, int j,
                                   vpColor::vpColorType color)
{
  if (OpenCVinitialized)
  {
    ((uchar*)(background->imageData + background->widthStep*i))[j*3] = (uchar)col[color].val[0];
    ((uchar*)(background->imageData + background->widthStep*i))[j*3+1] = (uchar)col[color].val[1];
    ((uchar*)(background->imageData + background->widthStep*i))[j*3+2] = (uchar)col[color].val[2];
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::displayLine(int i1, int j1, int i2, int j2,
                             vpColor::vpColorType color,
                             unsigned int e)
{
  if (OpenCVinitialized)
  {
    cvLine( background, cvPoint(j1,i1), cvPoint(j2,i2), col[color],
            (int)e);
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::displayDotLine(int i1, int j1, int i2, int j2,
                                vpColor::vpColorType color,
                                unsigned int e)
{

  if (OpenCVinitialized)
  {
    vpTRACE("Dot lines are not yet implemented");
    cvLine( background, cvPoint(j1,i1), cvPoint(j2,i2), col[color],
            (int)e);
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}


/*!
  \brief display a cross
  \param i,j : (row,colum indexes)
  \param size : size of the cross
  \param col : color (see vpColor)
*/
void
vpDisplayOpenCV::displayCross(int i, int j,
                              unsigned int size,
                              vpColor::vpColorType col)
{
  if (OpenCVinitialized)
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
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }

}


/*!
  \brief display a "large" cross
  \param i,j : (row,colum indexes)
  \param size : size of the cross
  \param col : color (see vpColor)
*/
void vpDisplayOpenCV::displayCrossLarge(int i, int j,
                                        unsigned int size,
                                        vpColor::vpColorType col)
{
  if (OpenCVinitialized)
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
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;    //
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
vpDisplayOpenCV::displayArrow(int i1, int j1, int i2, int j2,
                              vpColor::vpColorType col,
                              unsigned int L,unsigned int l)
{
  if (OpenCVinitialized)
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
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::displayRectangle(int i, int j,
                                  unsigned int width, unsigned int height,
                                  vpColor::vpColorType color, bool fill,
                                  unsigned int e)
{
  if (OpenCVinitialized)
  {
    if (fill == false)
      cvRectangle( background,
                   cvPoint(j,i),
                   cvPoint(j+width,i+height),
                   col[color],(int)e);
    else
      cvRectangle( background,
                   cvPoint(j,i),
                   cvPoint(j+height,i+width),
                   col[color],CV_FILLED);
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::displayRectangle(const vpRect &rect,
                                  vpColor::vpColorType color, bool fill,
                                  unsigned int e)
{
  if (OpenCVinitialized)
  {
    if (fill == false)
      cvRectangle( background,
                   cvPoint((int)rect.getLeft(),(int)rect.getBottom()),
                   cvPoint((int)rect.getRight(),(int)rect.getTop()),
                   col[color],(int)e);
    else
      cvRectangle( background,
                   cvPoint((int)rect.getLeft(),(int)rect.getBottom()),
                   cvPoint((int)rect.getRight(),(int)rect.getTop()),
                   col[color],CV_FILLED);
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}

/*!
  \brief display a string
  \param i,j (row,colum indexes)
  \param string
  \param color (see vpColor)
*/
void vpDisplayOpenCV::displayCharString(int i, int j,
                                        char *string, vpColor::vpColorType color)
{
  if (OpenCVinitialized)
  {
    cvPutText( background, string, cvPoint(j,i+fontHeight) , font, col[color] );
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::getClick(bool blocking)
{
  bool ret = false;
  if (OpenCVinitialized) {
    flushDisplay() ;
    if (blocking){
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown){
        ret = true ;
        lbuttondown = false;
      }
      if (mbuttondown){
        ret = true ;
        mbuttondown = false;
      }
      if (rbuttondown){
        ret = true ;
        rbuttondown = false;
      }
      if (blocking) cvWaitKey(10);
    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::getClick(unsigned int& i, unsigned int& j, bool blocking)
{
  bool ret = false;

  if (OpenCVinitialized) {
    flushDisplay() ;
    if (blocking){
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown){
        ret = true ;
        j = (unsigned int)x_lbuttondown;
        i = (unsigned int)y_lbuttondown;

        lbuttondown = false;
      }
      if (mbuttondown){
        ret = true ;
        j = (unsigned int)x_mbuttondown;
        i = (unsigned int)y_mbuttondown;

        mbuttondown = false;
      }
      if (rbuttondown){
        ret = true ;
        j = (unsigned int)x_rbuttondown;
        i = (unsigned int)y_rbuttondown;

        rbuttondown = false;
      }
      if (blocking) cvWaitKey(10);
    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::getClick(unsigned int& i, unsigned int& j,
                          vpMouseButton::vpMouseButtonType& button,
                          bool blocking)
{
  bool ret = false;

  if (OpenCVinitialized) {
    flushDisplay() ;
    if (blocking){
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown){
        ret = true ;
        j = (unsigned int)x_lbuttondown;
        i = (unsigned int)y_lbuttondown;
        button = vpMouseButton::button1;
        lbuttondown = false;
      }
      if (mbuttondown){
        ret = true ;
        j = (unsigned int)x_mbuttondown;
        i = (unsigned int)y_mbuttondown;
        button = vpMouseButton::button2;
        mbuttondown = false;
      }
      if (rbuttondown){
        ret = true ;
        j = (unsigned int)x_rbuttondown;
        i = (unsigned int)y_rbuttondown;
        button = vpMouseButton::button3;
        rbuttondown = false;
      }
      if (blocking) cvWaitKey(10);
    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::getClickUp(unsigned int& i, unsigned int& j,
                            vpMouseButton::vpMouseButtonType& button,
                            bool blocking)
{
  bool ret = false;
  if (OpenCVinitialized) {
    flushDisplay() ;
    if (blocking){
      lbuttonup = false;
      mbuttonup = false;
      rbuttonup = false;
    }
    do {
      if (lbuttonup){
        ret = true ;
        j = (unsigned int)x_lbuttonup;
        i = (unsigned int)y_lbuttonup;
        button = vpMouseButton::button1;
        lbuttonup = false;
      }
      if (mbuttonup){
        ret = true ;
        j = (unsigned int)x_mbuttonup;
        i = (unsigned int)y_mbuttonup;
        button = vpMouseButton::button2;
        mbuttonup = false;
      }
      if (rbuttonup){
        ret = true ;
        j = (unsigned int)x_rbuttonup;
        i = (unsigned int)y_rbuttonup;
        button = vpMouseButton::button3;
        rbuttonup = false;
      }
      if (blocking) cvWaitKey(10);
    } while ( ret == false && blocking == true);
  }
  else {
    vpERROR_TRACE ( "OpenCV not initialized " ) ;
    throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                 "OpenCV not initialized" ) ) ;
  }
  return ret;
}

/*!
  \brief set the window title
*/
void
vpDisplayOpenCV::flushTitle(const char *windowtitle)
{
  if (OpenCVinitialized)
  {
    vpTRACE("Not implemented");
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}

/*!
  \brief Display a circle
  \param i,j : circle center position (row,column)
  \param r : radius
  \param color
*/
void vpDisplayOpenCV::displayCircle(int i, int j,
                                    unsigned int r,
                                    vpColor::vpColorType color)
{
  if (OpenCVinitialized)
  {
    cvCircle( background, cvPoint(j,i), (int)r, col[color]);
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}

void vpDisplayOpenCV::on_mouse( int event, int x, int y, int flags, void* display )
{
  vpDisplayOpenCV* disp = (vpDisplayOpenCV*)display;
  switch ( event )
  {
    case CV_EVENT_LBUTTONDOWN:
    {
      disp->lbuttondown = true;
      disp->x_lbuttondown = x;
      disp->y_lbuttondown = y;
      break;
    }
    case CV_EVENT_MBUTTONDOWN:
    {
      disp->mbuttondown = true;
      disp->x_mbuttondown = x;
      disp->y_mbuttondown = y;
      break;
    }
    case CV_EVENT_RBUTTONDOWN:
    {
      disp->rbuttondown = true;
      disp->x_rbuttondown = x;
      disp->y_rbuttondown = y;
      break;
    }
    case CV_EVENT_LBUTTONUP:
    {
      disp->lbuttonup = true;
      disp->x_lbuttonup = x;
      disp->y_lbuttonup = y;
      break;
    }
    case CV_EVENT_MBUTTONUP:
    {
      disp->mbuttonup = true;
      disp->x_mbuttonup = x;
      disp->y_mbuttonup = y;
      break;
    }
    case CV_EVENT_RBUTTONUP:
    {
      disp->rbuttonup = true;
      disp->x_rbuttonup = x;
      disp->y_rbuttonup = y;
      break;
    }

    default :
      break;
  }
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
