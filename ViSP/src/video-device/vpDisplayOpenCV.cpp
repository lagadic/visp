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
 * Fabien Spindler
 *
 *****************************************************************************/


/*!
  \file vpDisplayOpenCV.cpp
  \brief Define the OpenCV console to display images.
*/

#include <visp/vpConfig.h>

#if ( defined(VISP_HAVE_OPENCV) )

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

// Display stuff
#include <visp/vpDisplay.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpMath.h>
#include <visp/vpImageTools.h>

//debug / exception
#include <visp/vpDebug.h>
#include <visp/vpDisplayException.h>

int vpDisplayOpenCV::count = 1;
/*!

  Constructor. Initialize a display to visualize a gray level image
  (8 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<unsigned char> &I,
                                 int x,
                                 int y,
                                 const char *title) : vpDisplay()
{
  col = NULL;
  title = NULL ;
  window = 0 ;
  background = NULL;
  font = NULL;
  init(I, x, y, title) ;
}


/*!
  Constructor. Initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.
*/
vpDisplayOpenCV::vpDisplayOpenCV(vpImage<vpRGBa> &I,
                                 int x,
                                 int y,
                                 const char *title)
{
  col = NULL;
  title = NULL ;
  window = 0 ;
  background = NULL;
  font = NULL;
  init(I, x, y, title) ;
}

/*!

  Constructor that just initialize the display position in the screen
  and the display title.

  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

  To initialize the display size, you need to call init().

  \code
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImage.h>

int main()
{
  vpDisplayOpenCV d(100, 200, "My display");
  vpImage<unsigned char> I(240, 384);
  d.init(I);
}
  \endcode
*/
vpDisplayOpenCV::vpDisplayOpenCV ( int x, int y, const char *title )
{
  col = NULL;
  title = NULL ;
  window = 0 ;
  background = NULL;
  font = NULL;
  init(0, 0, x, y, title) ;
}

/*!
  Basic constructor.

  To initialize the window position, title and size you may call
  init(vpImage<unsigned char> &, int, int, const char *) or
  init(vpImage<vpRGBa> &, int, int, const char *).

  \code
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpImage.h>

int main()
{
  vpDisplayOpenCV d;
  vpImage<unsigned char> I(240, 384);
  d.init(I, 100, 200, "My display");
}
  \endcode
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
  Destructor.
*/
vpDisplayOpenCV::~vpDisplayOpenCV()
{
  closeDisplay() ;
  cvReleaseImage(&background);
}

/*!
  Initialize the display (size, position and title) of a gray level image.

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void
vpDisplayOpenCV::init(vpImage<unsigned char> &I,
                      int x,
                      int y,
                      const char *title)
{

  if ((I.getHeight() == 0) || (I.getWidth()==0))
  {
    vpERROR_TRACE("Image not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "Image not initialized")) ;
  }
  init (I.getWidth(), I.getHeight(), x, y, title) ;
  I.display = this ;
  OpenCVinitialized = true ;
}

/*!  
  Initialize the display (size, position and title) of a color
  image in RGBa format.

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void
vpDisplayOpenCV::init(vpImage<vpRGBa> &I,
                      int x,
                      int y,
                      const char *title)
{
  if ((I.getHeight() == 0) || (I.getWidth()==0))
  {
    vpERROR_TRACE("Image not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "Image not initialized")) ;
  }

  init (I.getWidth(), I.getHeight(), x, y, title) ;
  I.display = this ;
  OpenCVinitialized = true ;
}

/*!
  Initialize the display size, position and title.

  \param width, height : Width and height of the window.
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void
vpDisplayOpenCV::init(unsigned int width, unsigned int height,
                      int x, int y,
                      const char *title)
{
  this->width  = width;
  this->height = height;
  this->windowXPosition = x;
  this->windowYPosition = y;
  int flags = CV_WINDOW_AUTOSIZE;
  if (title != NULL)
  {
    if (this->title != NULL)
    {
      delete [] this->title ;
      this->title = NULL ;
    }
    this->title = new char[strlen(title) + 1] ;
    strcpy(this->title, title) ;
  }
  else{
    if (this->title != NULL)
    {
      delete [] this->title ;
      this->title = NULL ;
    }
    this->title = new char[50] ;
    sprintf(this->title,"Unnamed ViSP display <%02d>",count) ;
  }
  count++;
  /* Create the window*/
  window = cvNamedWindow( this->title, flags );
  cvMoveWindow( this->title, x, y );
  move = false;
  lbuttondown = false;
  mbuttondown = false;
  rbuttondown = false;
  lbuttonup = false;
  mbuttonup = false;
  rbuttonup = false;
  cvSetMouseCallback( this->title, on_mouse, this );
  /* Create background pixmap */
//   background = cvCreateImage(cvSize((int)width,(int)height),IPL_DEPTH_8U,3);
//
//   cvShowImage( this->title,background);

  col = new CvScalar[vpColor::id_unknown] ;

  /* Create color */
  vpColor pcolor; // Predefined colors
  pcolor = vpColor::lightBlue;
  col[vpColor::id_lightBlue]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::blue;
  col[vpColor::id_blue]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::darkBlue;
  col[vpColor::id_darkBlue]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::lightRed;
  col[vpColor::id_lightRed]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::red;
  col[vpColor::id_red]    = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::darkRed;
  col[vpColor::id_darkRed]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::lightGreen;
  col[vpColor::id_lightGreen]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::green;
  col[vpColor::id_green]  = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::darkGreen;
  col[vpColor::id_darkGreen]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::yellow;
  col[vpColor::id_yellow] = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::cyan;
  col[vpColor::id_cyan]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::orange;
  col[vpColor::id_orange] = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::purple;
  col[vpColor::id_purple] = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::white;
  col[vpColor::id_white]  = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::black;
  col[vpColor::id_black]  = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::lightGray;
  col[vpColor::id_lightGray]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::gray;
  col[vpColor::id_gray]  = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;
  pcolor = vpColor::darkGray;
  col[vpColor::id_darkGray]   = CV_RGB(pcolor.R, pcolor.G, pcolor.B) ;

  font = new CvFont;
  cvInitFont( font, CV_FONT_HERSHEY_PLAIN, 1.0f,1.0f);

  CvSize fontSize;
  int baseline;
  cvGetTextSize( "A", font, &fontSize, &baseline );
  fontHeight = fontSize.height + baseline;
  OpenCVinitialized = true ;
}


/*!
  \warning This method is not yet implemented.

  Set the font used to display a text in overlay. The display is
  performed using displayCharString().

  \param font : The expected font name. The available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \note Under UNIX, to know all the available fonts, use the
  "xlsfonts" binary in a terminal. You can also use the "xfontsel" binary.

  \sa displayCharString()
*/
void
vpDisplayOpenCV::setFont(const char * /* font */)
{
  vpERROR_TRACE("Not yet implemented" ) ;
}

/*!
  Set the window title.

  \warning This method is not implemented yet.

  \param title : Window title.
 */
void
vpDisplayOpenCV::setTitle(const char * /* title */)
{

  vpTRACE("Not implemented");
#if 0
  if (OpenCVinitialized)
  {
    if (this->title != NULL) {
      delete [] this->title ;
      this->title = NULL ;
    }
    //    fprintf(stdout, "len: %d\n",  strlen(windowtitle)); fflush(stdout);
    this->title = new char[strlen(windowtitle) + 1] ;
    strcpy(this->title, windowtitle) ;
    //cvMoveWindow( this->title, windowXPosition,  windowYPosition);
    cvMoveWindow(windowtitle , windowXPosition,  windowYPosition);

  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
#endif
}


/*!
  Set the window position in the screen.

  \param winx, winy : Position of the upper-left window's border in the screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void vpDisplayOpenCV::setWindowPosition(int winx, int winy)
{
  if (OpenCVinitialized) {
    this->windowXPosition = winx;
    this->windowYPosition = winy;
    cvMoveWindow( this->title, winx, winy );
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}
/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning suppres the overlay drawing

  \param I : Image to display.

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
  Display a selection of the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param I : Image to display.
  
  \param iP : Top left corner of the region of interest
  
  \param width : Width of the region of interest
  
  \param height : Height of the region of interest

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImageROI ( const vpImage<unsigned char> &I,const vpImagePoint iP, const unsigned int width, const unsigned int height )
{
  if (OpenCVinitialized)
  { 
    vpImage<unsigned char> Itemp;
    vpImageTools::createSubImage(I,iP.get_i(),iP.get_j(),height,width,Itemp);
    vpImage<vpRGBa> Ic;
    vpImageConvert::convert(Itemp,Ic);
    
    CvSize size = cvSize(this->width,this->height);
    int depth = 8;
    int channels = 3;
    if (background != NULL){
      if(background->nChannels != channels || background->depth != depth
         || background->height != (int) I.getHeight() || background->width != (int) I.getWidth()){
        if(background->nChannels != 0) cvReleaseImage(&background);
        background = cvCreateImage( size, depth, channels );
      }
    }
    else background = cvCreateImage( size, depth, channels );
    
    IplImage* Ip = NULL;
    vpImageConvert::convert(Ic, Ip);
    
    unsigned char * input = (unsigned char*)Ip->imageData;
    unsigned char * output = (unsigned char*)background->imageData;
    
    unsigned int iwidth = Ic.getWidth();

    input = input;
    output = output + (int)(iP.get_i()*3*this->width+ iP.get_j()*3);
    
    unsigned int i = 0;
    while (i < height)
    {
      unsigned int j = 0;
      while (j < width)
      {
	*(output+3*j) = *(input+j*3);
	*(output+3*j+1) = *(input+j*3+1);
	*(output+3*j+2) = *(input+j*3+2);
	j++;
      }
      input = input + 3*iwidth;
      output = output + 3*this->width;
      i++;
    }

    cvReleaseImage(&Ip);
  }
  else
  {
    vpERROR_TRACE("openCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}


/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning suppres the overlay drawing

  \param I : Image to display.

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

/*!
  Display a selection of the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing in the region of interest.

  \param I : Image to display.
  
  \param iP : Top left corner of the region of interest
  
  \param width : Width of the region of interest
  
  \param height : Height of the region of interest

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImageROI ( const vpImage<vpRGBa> &I,const vpImagePoint iP, const unsigned int width, const unsigned int height )
{
  if (OpenCVinitialized)
  { 
    vpImage<vpRGBa> Ic;
    vpImageTools::createSubImage(I,iP.get_i(),iP.get_j(),height,width,Ic);
    
    CvSize size = cvSize(this->width,this->height);
    int depth = 8;
    int channels = 3;
    if (background != NULL){
      if(background->nChannels != channels || background->depth != depth
         || background->height != (int) I.getHeight() || background->width != (int) I.getWidth()){
        if(background->nChannels != 0) cvReleaseImage(&background);
        background = cvCreateImage( size, depth, channels );
      }
    }
    else background = cvCreateImage( size, depth, channels );
    
    IplImage* Ip = NULL;
    vpImageConvert::convert(Ic, Ip);
    
    unsigned char * input = (unsigned char*)Ip->imageData;
    unsigned char * output = (unsigned char*)background->imageData;
    
    unsigned int iwidth = Ic.getWidth();

    input = input;
    output = output + (int)(iP.get_i()*3*this->width+ iP.get_j()*3);
    
    unsigned int i = 0;
    while (i < height)
    {
      unsigned int j = 0;
      while (j < width)
      {
	*(output+3*j) = *(input+j*3);
	*(output+3*j+1) = *(input+j*3+1);
	*(output+3*j+2) = *(input+j*3+2);
	j++;
      }
      input = input + 3*iwidth;
      output = output + 3*this->width;
      i++;
    }

    cvReleaseImage(&Ip);
  }
  else
  {
    vpERROR_TRACE("openCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}


/*!
  \warning ot implemented yet

  \sa init(), closeDisplay()
*/
void vpDisplayOpenCV::displayImage(const unsigned char * /* I */)
{
  vpTRACE(" not implemented ") ;
}

/*!

  Close the window.

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
  Flushes the OpenCV buffer.
  It's necessary to use this function to see the results of any drawing.

*/
void vpDisplayOpenCV::flushDisplay()
{
  if (OpenCVinitialized)
  {
    cvShowImage(title, background );
    cvWaitKey(5);
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}

/*!
  Flushes the OpenCV buffer.
  It's necessary to use this function to see the results of any drawing.

*/
void vpDisplayOpenCV::flushDisplayROI(const vpImagePoint /*iP*/, const unsigned int /*width*/, const unsigned int /*height*/)
{
  if (OpenCVinitialized)
  {
    cvShowImage(title, background );
    cvWaitKey(5);
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
}


/*!
  \warning Not implemented yet.
*/
void vpDisplayOpenCV::clearDisplay(const vpColor & /* color */)
{
  vpTRACE("Not implemented") ;
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplayOpenCV::displayArrow ( const vpImagePoint &ip1, 
				     const vpImagePoint &ip2,
				     const vpColor &color,
				     unsigned int w, unsigned int h,
				     unsigned int thickness)
{
  if (OpenCVinitialized)
  {
    try{
      double a = ip2.get_i() - ip1.get_i() ;
      double b = ip2.get_j() - ip1.get_j() ;
      double lg = sqrt(vpMath::sqr(a)+vpMath::sqr(b)) ;

      if ((a==0)&&(b==0))
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
  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param ip : Upper left image point location of the string in the display.
  \param text : String to display in overlay.
  \param color : String color.

  \sa setFont()
*/
void vpDisplayOpenCV::displayCharString( const vpImagePoint &ip,
                                     const char *text, 
				     const vpColor &color )
{
  if (OpenCVinitialized)
  {
    if (color.id < vpColor::id_unknown) {
      cvPutText( background, text, 
		 cvPoint( vpMath::round( ip.get_u() ),
			  vpMath::round( ip.get_v()+fontHeight ) ), 
		 font, col[color.id] );
    }
    else {
      cvcolor = CV_RGB(color.R, color.G, color.B) ;
      cvPutText( background, text, 
		 cvPoint( vpMath::round( ip.get_u() ),
			  vpMath::round( ip.get_v()+fontHeight ) ), 
		 font, cvcolor );
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
  Display a circle.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the circle.
  \param thickness : Thickness of the circle. This parameter is only useful 
  when \e fill is set to false.
*/
void vpDisplayOpenCV::displayCircle(const vpImagePoint &center,
				    unsigned int radius,
				    const vpColor &color,
				    bool  fill ,
				    unsigned int thickness)
{
  if (OpenCVinitialized)
  {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
	cvCircle( background, 
		  cvPoint( vpMath::round( center.get_u() ), 
			   vpMath::round( center.get_v() ) ), 
		  (int)radius, col[color.id], (int)thickness);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvCircle( background, 
		  cvPoint( vpMath::round( center.get_u() ), 
			   vpMath::round( center.get_v() ) ), 
		  (int)radius, cvcolor, (int)thickness);
      }
    }
    else {
      if (color.id < vpColor::id_unknown) {
	cvCircle( background, 
		  cvPoint( vpMath::round( center.get_u() ), 
			   vpMath::round( center.get_v() ) ), 
		  (int)radius, col[color.id], CV_FILLED);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvCircle( background, 
		  cvPoint( vpMath::round( center.get_u() ), 
			   vpMath::round( center.get_v() ) ), 
		  (int)radius, cvcolor, CV_FILLED);
      }
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
  Display a cross at the image point \e ip location.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void
vpDisplayOpenCV::displayCross(const vpImagePoint &ip,
                              unsigned int size,
                              const vpColor &color,
			      unsigned int thickness)
{
  if (OpenCVinitialized)
  {
    vpImagePoint top,bottom,left,right;
    top.set_i(ip.get_i()-size/2);
    top.set_j(ip.get_j());
    bottom.set_i(ip.get_i()+size/2);
    bottom.set_j(ip.get_j());
    left.set_i(ip.get_i());
    left.set_j(ip.get_j()-size/2);
    right.set_i(ip.get_i());
    right.set_j(ip.get_j()+size/2);
    try{
      displayLine(top, bottom, color, thickness) ;
      displayLine(left, right, color, thickness) ;
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
  Display a dashed line from image point \e ip1 to image point \e ip2.

  \warning Dot lines are not yet implemented in OpenCV. We display a
  normal line instead.

  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void
vpDisplayOpenCV::displayDotLine(const vpImagePoint &ip1, 
				const vpImagePoint &ip2,
				const vpColor &color, 
				unsigned int thickness)
{

  if (OpenCVinitialized)
  {
    vpTRACE("Dot lines are not yet implemented");
    if (color.id < vpColor::id_unknown) {
      cvLine( background, 
	      cvPoint( vpMath::round( ip1.get_u() ), 
		       vpMath::round( ip1.get_v() ) ), 
	      cvPoint( vpMath::round( ip2.get_u() ),
		       vpMath::round( ip2.get_v() ) ), 
	      col[color.id], (int) thickness);
    }
    else {
      cvcolor = CV_RGB(color.R, color.G, color.B) ;
      cvLine( background, 
	      cvPoint( vpMath::round( ip1.get_u() ), 
		       vpMath::round( ip1.get_v() ) ), 
	      cvPoint( vpMath::round( ip2.get_u() ),
		       vpMath::round( ip2.get_v() ) ), 
	      cvcolor, (int) thickness);
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
  Display a line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void
vpDisplayOpenCV::displayLine(const vpImagePoint &ip1, 
			     const vpImagePoint &ip2,
			     const vpColor &color, 
			     unsigned int thickness)
{
  if (OpenCVinitialized)
  {
    if (color.id < vpColor::id_unknown) {
      cvLine( background, 
	      cvPoint( vpMath::round( ip1.get_u() ), 
		       vpMath::round( ip1.get_v() ) ), 
	      cvPoint( vpMath::round( ip2.get_u() ),
		       vpMath::round( ip2.get_v() ) ), 
	      col[color.id], (int) thickness);
    }
    else {
      cvcolor = CV_RGB(color.R, color.G, color.B) ;
      cvLine( background, 
	      cvPoint( vpMath::round( ip1.get_u() ), 
		       vpMath::round( ip1.get_v() ) ), 
	      cvPoint( vpMath::round( ip2.get_u() ),
		       vpMath::round( ip2.get_v() ) ), 
	      cvcolor, (int) thickness);
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
  Display a point at the image point \e ip location.
  \param ip : Point location.
  \param color : Point color.
*/
void vpDisplayOpenCV::displayPoint(const vpImagePoint &ip,
                                   const vpColor &color)
{
  if (OpenCVinitialized)
  {
    displayLine(ip,ip,color,1);
//     if (color.id < vpColor::id_unknown) {
//       ((uchar*)(background->imageData 
// 		+ background->widthStep*vpMath::round( ip.get_i() )))
// 	[vpMath::round( ip.get_j()*3 )] = (uchar)col[color.id].val[0];
// 
//       ((uchar*)(background->imageData 
// 		+ background->widthStep*vpMath::round( ip.get_i() )))
// 	[vpMath::round( ip.get_j()*3+1 )] = (uchar)col[color.id].val[1];
//       
//       ((uchar*)(background->imageData 
// 		+ background->widthStep*vpMath::round( ip.get_i() )))
// 	[vpMath::round( ip.get_j()*3+2 )] = (uchar)col[color.id].val[2];
//     }
//     else {
//       cvcolor = CV_RGB(color.R, color.G, color.B) ;
//       ((uchar*)(background->imageData 
// 		+ background->widthStep*vpMath::round( ip.get_i() )))
// 	[vpMath::round( ip.get_j()*3 )] = (uchar)cvcolor.val[0];
// 
//       ((uchar*)(background->imageData 
// 		+ background->widthStep*vpMath::round( ip.get_i() )))
// 	[vpMath::round( ip.get_j()*3+1 )] = (uchar)cvcolor.val[1];
//       
//       ((uchar*)(background->imageData 
// 		+ background->widthStep*vpMath::round( ip.get_i() )))
// 	[vpMath::round( ip.get_j()*3+2 )] = (uchar)cvcolor.val[2];
// 
//     }
  }
  else
  {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
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
vpDisplayOpenCV::displayRectangle(const vpImagePoint &topLeft,
                                  unsigned int width, unsigned int height,
                                  const vpColor &color, bool fill,
                                  unsigned int thickness)
{
  if (OpenCVinitialized)
  {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ),
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( topLeft.get_u()+width ),
			      vpMath::round( topLeft.get_v()+height ) ),
		     col[color.id], (int)thickness);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ),
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( topLeft.get_u()+width ),
			      vpMath::round( topLeft.get_v()+height ) ),
		     cvcolor, (int)thickness);
      }
    }
    else {
      if (color.id < vpColor::id_unknown) {
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ),
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( topLeft.get_u()+width ),
			      vpMath::round( topLeft.get_v()+height ) ),
		     col[color.id], CV_FILLED);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ),
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( topLeft.get_u()+width ),
			      vpMath::round( topLeft.get_v()+height ) ),
		     cvcolor, CV_FILLED);

      }
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
vpDisplayOpenCV::displayRectangle ( const vpImagePoint &topLeft,
				    const vpImagePoint &bottomRight,
				    const vpColor &color, bool fill,
				    unsigned int thickness )
{
  if (OpenCVinitialized)
  {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ), 
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( bottomRight.get_u() ), 
			      vpMath::round( bottomRight.get_v() ) ),
		     col[color.id], (int)thickness);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ), 
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( bottomRight.get_u() ), 
			      vpMath::round( bottomRight.get_v() ) ),
		     cvcolor, (int)thickness);
      }
    }
    else {
      if (color.id < vpColor::id_unknown) {
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ), 
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( bottomRight.get_u() ), 
			      vpMath::round( bottomRight.get_v() ) ),
		     col[color.id], CV_FILLED);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvRectangle( background,
		     cvPoint( vpMath::round( topLeft.get_u() ), 
			      vpMath::round( topLeft.get_v() ) ),
		     cvPoint( vpMath::round( bottomRight.get_u() ), 
			      vpMath::round( bottomRight.get_v() ) ),
		     cvcolor, CV_FILLED);

      }
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
  Display a rectangle.

  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void
vpDisplayOpenCV::displayRectangle(const vpRect &rectangle,
                                  const vpColor &color, bool fill,
                                  unsigned int thickness)
{
  if (OpenCVinitialized)
  {
    if (fill == false) {
      if (color.id < vpColor::id_unknown) {
	cvRectangle( background,
		     cvPoint( vpMath::round( rectangle.getLeft() ),
			      vpMath::round( rectangle.getBottom() ) ),
		     cvPoint( vpMath::round( rectangle.getRight() ),
			      vpMath::round( rectangle.getTop() ) ),
		     col[color.id], (int)thickness);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvRectangle( background,
		     cvPoint( vpMath::round( rectangle.getLeft() ),
			      vpMath::round( rectangle.getBottom() ) ),
		     cvPoint( vpMath::round( rectangle.getRight() ),
			      vpMath::round( rectangle.getTop() ) ),
		     cvcolor, (int)thickness);

      }
    }
    else {
      if (color.id < vpColor::id_unknown) {
	cvRectangle( background,
		     cvPoint( vpMath::round( rectangle.getLeft() ),
			      vpMath::round( rectangle.getBottom() ) ),
		     cvPoint( vpMath::round( rectangle.getRight() ),
			      vpMath::round( rectangle.getTop() ) ),
		     col[color.id], CV_FILLED);
      }
      else {
	cvcolor = CV_RGB(color.R, color.G, color.B) ;
	cvRectangle( background,
		     cvPoint( vpMath::round( rectangle.getLeft() ),
			      vpMath::round( rectangle.getBottom() ) ),
		     cvPoint( vpMath::round( rectangle.getRight() ),
			      vpMath::round( rectangle.getTop() ) ),
		     cvcolor, CV_FILLED);
      }
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
vpDisplayOpenCV::getClick(vpImagePoint &ip, bool blocking)
{
  bool ret = false;

  if (OpenCVinitialized) {
    flushDisplay() ;

    double u, v;

    if (blocking){
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown){
        ret = true ;
        u = (unsigned int)x_lbuttondown;
        v = (unsigned int)y_lbuttondown;
	ip.set_u( u );
	ip.set_v( v );
        lbuttondown = false;
      }
      if (mbuttondown){
        ret = true ;
        u = (unsigned int)x_mbuttondown;
        v = (unsigned int)y_mbuttondown;
	ip.set_u( u );
	ip.set_v( v );
        mbuttondown = false;
      }
      if (rbuttondown){
        ret = true ;
        u = (unsigned int)x_rbuttondown;
        v = (unsigned int)y_rbuttondown;
	ip.set_u( u );
	ip.set_v( v );
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
vpDisplayOpenCV::getClick(vpImagePoint &ip,
                          vpMouseButton::vpMouseButtonType& button,
                          bool blocking)
{
  bool ret = false;

  if (OpenCVinitialized) {
    //flushDisplay() ;
    double u, v;
    if (blocking){
      lbuttondown = false;
      mbuttondown = false;
      rbuttondown = false;
    }
    do {
      if (lbuttondown){
        ret = true ;
        u = (unsigned int)x_lbuttondown;
        v = (unsigned int)y_lbuttondown;
	ip.set_u( u );
	ip.set_v( v );
        button = vpMouseButton::button1;
        lbuttondown = false;
      }
      if (mbuttondown){
        ret = true ;
        u = (unsigned int)x_mbuttondown;
        v = (unsigned int)y_mbuttondown;
	ip.set_u( u );
	ip.set_v( v );
        button = vpMouseButton::button2;
        mbuttondown = false;
      }
      if (rbuttondown){
        ret = true ;
        u = (unsigned int)x_rbuttondown;
        v = (unsigned int)y_rbuttondown;
	ip.set_u( u );
	ip.set_v( v );
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
vpDisplayOpenCV::getClickUp(vpImagePoint &ip,
                            vpMouseButton::vpMouseButtonType& button,
                            bool blocking)
{
  bool ret = false;
  if (OpenCVinitialized) {
    //flushDisplay() ;
    double u, v;
    if (blocking){
      lbuttonup = false;
      mbuttonup = false;
      rbuttonup = false;
    }
    do {
      if (lbuttonup){
        ret = true ;
        u = (unsigned int)x_lbuttonup;
        v = (unsigned int)y_lbuttonup;
	ip.set_u( u );
	ip.set_v( v );
        button = vpMouseButton::button1;
        lbuttonup = false;
      }
      if (mbuttonup){
        ret = true ;
        u = (unsigned int)x_mbuttonup;
        v = (unsigned int)y_mbuttonup;
	ip.set_u( u );
	ip.set_v( v );
        button = vpMouseButton::button2;
        mbuttonup = false;
      }
      if (rbuttonup){
        ret = true ;
        u = (unsigned int)x_rbuttonup;
        v = (unsigned int)y_rbuttonup;
	ip.set_u( u );
	ip.set_v( v );
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

/*
  \brief gets the displayed image (including the overlay plane)
  and returns an RGBa image
*/
void vpDisplayOpenCV::getImage(vpImage<vpRGBa> &I)
{
  vpImageConvert::convert(background,I);
  // shoudl certainly be optimized.
}

void vpDisplayOpenCV::on_mouse( int event, int x, int y, int /*flags*/, void* display )
{
  vpDisplayOpenCV* disp = (vpDisplayOpenCV*)display;
  switch ( event )
  {
    case CV_EVENT_MOUSEMOVE:
    {
      disp->move = true;
      disp->x_move = x;
      disp->y_move = y;
      break;
    }
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
*/
bool
vpDisplayOpenCV::getKeyboardEvent(bool blocking)
{
  int key_pressed;
  int delay;
  if (OpenCVinitialized) {
    flushDisplay() ;
    if (blocking) 
      delay = 0;
    else 
      delay = 10;

    key_pressed = cvWaitKey(delay);
    if (key_pressed == -1)
      return false;
    return true;
  }
  else {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
  return false;
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
*/
bool
vpDisplayOpenCV::getKeyboardEvent(char *string, bool blocking)
{
  int key_pressed;
  int delay;
  if (OpenCVinitialized) {
    flushDisplay() ;
    if (blocking) 
      delay = 0;
    else 
      delay = 10;

    key_pressed = cvWaitKey(delay);
    if (key_pressed == -1)
      return false;
    else {
      //std::cout << "Key pressed: \"" << key_pressed << "\"" << std::endl;
      sprintf(string, "%c", key_pressed);
    }
    return true;
  }
  else {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
  return false;
}

/*!
  Get the coordinates of the mouse pointer.

  \warning Not implemented yet.
  
  \param ip [out] : The coordinates of the mouse pointer.
  
  \return true if a pointer motion event was received, false otherwise.
  
  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.
*/
bool 
vpDisplayOpenCV::getPointerMotionEvent (vpImagePoint &ip )
{
  bool ret = false;

  if (OpenCVinitialized) {
    //flushDisplay() ;
    double u, v;
    if (move){
      ret = true ;
      u = (unsigned int)x_move;
      v = (unsigned int)y_move;
      ip.set_u( u );
      ip.set_v( v );
      move = false;
    }
  }

  else {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
  return ret;
}

/*!
  Get the coordinates of the mouse pointer.

  \param ip [out] : The coordinates of the mouse pointer.

  \return true.

  \exception vpDisplayException::notInitializedError : If the display
  was not initialized.
*/
bool
vpDisplayOpenCV::getPointerPosition ( vpImagePoint &ip)
{
  if (OpenCVinitialized) {
    //vpTRACE("Not implemented yet");
    bool moved = getPointerMotionEvent(ip);
    if (!moved)
    {
      double u, v;
      u = (unsigned int)x_move;
      v = (unsigned int)y_move;
      ip.set_u( u );
      ip.set_v( v );
    }
    return false;
  }
  else {
    vpERROR_TRACE("OpenCV not initialized " ) ;
    throw(vpDisplayException(vpDisplayException::notInitializedError,
                             "OpenCV not initialized")) ;
  }
  return false;
}

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
