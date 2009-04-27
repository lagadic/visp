/****************************************************************************
 *
 * $Id: vpDisplay.cpp,v 1.36 2008-12-17 10:51:12 fspindle Exp $
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
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp/vpDisplay.h>
#include <visp/vpDisplayException.h>

#include <visp/vpPoint.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpMath.h>


/*!
  \file vpDisplay.cpp
  \brief  generic class for image display

  Sample code :
  \code

    #include <visp/vpDebug.h>
    #include <visp/vpConfig.h>

    #include <visp/vpImage.h>
    #include <visp/vpImageIo.h>
    #include <visp/vpDisplayX.h>
    #include <visp/vpDisplayGTK.h>
    #include <visp/vpDisplayGDI.h>
    #include <visp/vpDisplayD3D.h>

    vpImage<vpRGBa> Ic; // A color image

    //Read an image on a disk
    vpImageIo::readPPM(Ic, "image.ppm");


    #if defined VISP_HAVE_GTK
      vpDisplayGTK display;
    #elif defined VISP_HAVE_X11
      vpDisplayX display;
    #elif defined VISP_HAVE_GDI
      vpDisplayGDI display;
    #elif defined VISP_HAVE_D3D
      vpDisplayD3D display;
    #endif

    // We open a window using either X11 or GTK or GDI.
    // Its size is automatically defined by the image (I) size
    display.init(I, 100, 100,"Display...") ;

    // Display the image
    // The image class has a member that specify a pointer toward
    // the display that has been initialized in the display declaration
    // therefore is is no longuer necessary to make a reference to the
    // display variable.
    vpDisplay::display(I) ;

    // Display in overlay a red cross at position 10,10 in the
    // image. The lines are 10 pixels long
    vpDisplay::displayCross(I, 100,10, 20, vpColor::red) ;
    // ...

    // The display is flushed; without this line, nothing will appear in
    // the display.
    vpDisplay::flush(I) ;

  \endcode

*/

vpDisplay::vpDisplay()
{
  title = NULL ;
  displayHasBeenInitialized = false ;
}

/*!
  Set the windows title.
  \param I : Image associated to the display window.
  \param windowtitle : Window title.
*/
void
vpDisplay::setTitle ( const vpImage<unsigned char> &I, 
		      const char *windowtitle )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setTitle ( windowtitle ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Set the window position in the screen.

  \param I : Image associated to the display window.
  \param winx, winy : Position of the upper-left window's border in the screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void
vpDisplay::setWindowPosition ( const vpImage<unsigned char> &I, 
			       int winx, int winy )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setWindowPosition ( winx, winy ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Set the font of a text printed in the display overlay. To print a
  text you may use displayCharString().

  \param I : Image associated to the display window.
  \param fontname : The expected font name.

  \note Under UNIX, the available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \sa displayCharString()
*/
void
vpDisplay::setFont ( const vpImage<unsigned char> &I, 
		      const char *fontname )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setFont ( fontname ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a 8bits image in the display window
*/
void
vpDisplay::display ( const vpImage<unsigned char> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayImage ( I ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}



/*!
  \brief get the window pixmap and put it in vpRGBa image
*/
void
vpDisplay::getImage ( const vpImage<unsigned  char> &Isrc,
                      vpImage<vpRGBa> &Idest )
{

  try
  {
    if ( Isrc.display != NULL )
    {
      ( Isrc.display )->getImage ( Idest ) ;
    }
    else
    {
      vpERROR_TRACE ( "Display not initialized" ) ;
      throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                   "Display not initialized" ) ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a point at coordinates (i,j) in the display window
*/
void vpDisplay::displayPoint ( const vpImage<unsigned char> &I,
                               int i, int j,
                               vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayPoint ( i,j,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (i,j) in the display window
*/
void vpDisplay::displayCross ( const vpImage<unsigned char> &I,
                               int i, int j,
                               unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCross ( i,j,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (i,j) in the display window
*/
void
vpDisplay::displayCrossLarge ( const vpImage<unsigned char> &I,
                               int i, int j,
                               unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCrossLarge ( i,j,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (i,j) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle ( const vpImage<unsigned char> &I,
                           int i, int j, unsigned int r,
                           vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCircle ( i,j,r,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
*/
void vpDisplay::displayLine ( const vpImage<unsigned char> &I,
                              int i1, int j1, int i2, int j2,
                              vpColor::vpColorType col, unsigned int e )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayLine ( i1,j1,i2,j2,col,e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!  Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine ( const vpImage<unsigned char> &I,
                                 int i1, int j1, int i2, int j2,
                                 vpColor::vpColorType col, unsigned int e2 )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayDotLine ( i1,j1,i2,j2,col,e2 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

void
vpDisplay::displayFrame ( const vpImage<unsigned char> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, vpColor::vpColorType col)
{
  // used by display
  vpPoint o; o.setWorldCoordinates ( 0.0,0.0,0.0 ) ;
  vpPoint x; x.setWorldCoordinates ( size,0.0,0.0 ) ;
  vpPoint y; y.setWorldCoordinates ( 0.0,size,0.0 ) ;
  vpPoint z; z.setWorldCoordinates ( 0.0,0.0,size ) ;

  o.track ( cMo ) ;
  x.track ( cMo ) ;
  y.track ( cMo ) ;
  z.track ( cMo ) ;

  double ox=0, oy=0, x1=0, y1=0 ;

  if ( col == vpColor::none )
  {
    vpMeterPixelConversion::convertPoint ( cam,o.p[0],o.p[1],ox,oy) ;

    vpMeterPixelConversion::convertPoint ( cam,x.p[0],x.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              vpColor::red ) ;

    vpMeterPixelConversion::convertPoint ( cam,y.p[0],y.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              vpColor::green ) ;

    vpMeterPixelConversion::convertPoint ( cam,z.p[0],z.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              vpColor::blue ) ;
  }
  else
  {
    vpMeterPixelConversion::convertPoint ( cam,o.p[0],o.p[1],ox,oy) ;

    vpMeterPixelConversion::convertPoint ( cam,x.p[0],x.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              col ) ;

    vpMeterPixelConversion::convertPoint ( cam,y.p[0],y.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              col ) ;

    vpMeterPixelConversion::convertPoint ( cam,z.p[0],z.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              col ) ;
  }
}


void
vpDisplay::displayFrame ( const vpImage<vpRGBa> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, vpColor::vpColorType col)
{
  // used by display
  vpPoint o; o.setWorldCoordinates ( 0.0,0.0,0.0 ) ;
  vpPoint x; x.setWorldCoordinates ( size,0.0,0.0 ) ;
  vpPoint y; y.setWorldCoordinates ( 0.0,size,0.0 ) ;
  vpPoint z; z.setWorldCoordinates ( 0.0,0.0,size ) ;

  o.track ( cMo ) ;
  x.track ( cMo ) ;
  y.track ( cMo ) ;
  z.track ( cMo ) ;

  double ox=0, oy=0, x1=0, y1=0 ;

  if ( col == vpColor::none )
  {
    vpMeterPixelConversion::convertPoint ( cam,o.p[0],o.p[1],ox,oy) ;

    vpMeterPixelConversion::convertPoint ( cam,x.p[0],x.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              vpColor::red ) ;

    vpMeterPixelConversion::convertPoint ( cam,y.p[0],y.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              vpColor::green ) ;

    vpMeterPixelConversion::convertPoint ( cam,z.p[0],z.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              vpColor::blue ) ;
  }
  else
  {
    vpMeterPixelConversion::convertPoint ( cam,o.p[0],o.p[1],ox,oy) ;

    vpMeterPixelConversion::convertPoint ( cam,x.p[0],x.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              col ) ;

    vpMeterPixelConversion::convertPoint ( cam,y.p[0],y.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              col ) ;

    vpMeterPixelConversion::convertPoint ( cam,z.p[0],z.p[1],x1,y1) ;
    vpDisplay::displayArrow ( I,
                              vpMath::round ( oy ), vpMath::round ( ox ),
                              vpMath::round ( y1 ), vpMath::round ( x1 ),
                              col ) ;
  }
}

/*! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  window
*/
void
vpDisplay::displayArrow ( const vpImage<unsigned char> &I,
                          int i1, int j1, int i2, int j2,
                          vpColor::vpColorType col,
                          unsigned int L,unsigned int l )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayArrow ( i1,j1,i2,j2,col,L,l ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

Display a rectangle in the display window.  The rectangle upper left corner
has coordinates (i,j). The size of the rectangle is fixed by \e width and \e
height.

\param I : Image associated to the display.
\param i : Row number of the rectangle upper corner
\param j : Column number of the rectangle upper corner
\param width : Width of the rectangle.
\param height : Height of the rectangle.
\param col : Color of the rectangle.
\param fill : set as true to fill the rectangle.
\param e : Line thickness

*/
void
vpDisplay::displayRectangle ( const vpImage<unsigned char> &I,
                              int i, int j,
			      unsigned int width, unsigned int height,
                              vpColor::vpColorType col, bool fill,
			                        unsigned int e)
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( i,j,width,height,col, fill, e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!

Display a rectangle in the display window.

\param I : Image associated to the display.
\param rect : The rectangle characteristics
\param col  : Color of the rectangle.
\param fill : set as true to fill the rectangle.
\param e : Line thickness
*/
void
vpDisplay::displayRectangle ( const vpImage<unsigned char> &I,
                              const vpRect &rect,
                              vpColor::vpColorType col, bool fill,
			                        unsigned int e )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( rect, col , fill, e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!

Display a rectangle in the display window.  The rectangle is defined by its center, its orientation (angle) and its size. The rectangle center has coordinates (i,j). 
The size of the rectangle is fixed by \e width and \e height.

\param I : Image associated to the display.
\param i : Row number of the rectangle center
\param j : Column number of the rectangle center
\param angle : Angle of the width side with horizontal
\param width : Width of the rectangle.
\param height : Height of the rectangle.
\param col : Color of the rectangle.
\param e : Line thickness

*/
void
vpDisplay::displayRectangle(const vpImage<unsigned char> &I,
			    unsigned int i, unsigned int j, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col,unsigned int e)
{
	try
    {
		if (I.display != NULL)
		{
			//A, B, C, D, corners of the rectangle clockwise
			int ua, va, ub, vb, uc, vc, ud, vd;
			float cosinus = cos(angle);
			float sinus = sin(angle);
			ua = (int)(j + 0.5*width*cosinus + 0.5*height*sinus);
			va = (int)(i + 0.5*width*sinus - 0.5*height*cosinus);
			ub = (int)(j + 0.5*width*cosinus - 0.5*height*sinus);
			vb = (int)(i + 0.5*width*sinus + 0.5*height*cosinus);
			uc = (int)(j - 0.5*width*cosinus - 0.5*height*sinus);
			vc = (int)(i - 0.5*width*sinus + 0.5*height*cosinus);
			ud = (int)(j - 0.5*width*cosinus + 0.5*height*sinus);
			vd = (int)(i - 0.5*width*sinus - 0.5*height*cosinus);

			( I.display )->displayLine_uv(I, ua, va, ub, vb,col, e);
			( I.display )->displayLine_uv(I, ua, va, ud, vd,col, e);
			( I.display )->displayLine_uv(I, uc, vc, ub, vb,col, e);
			( I.display )->displayLine_uv(I, uc, vc, ud, vd,col, e);
		}
    }
	catch(...)
    {
		vpERROR_TRACE("Error caught in displayRectangle") ;
		throw ;
    }
}

/*!

Display a rectangle in the display window.  The rectangle is defined by its center, its orientation (angle) and its size. The rectangle center has coordinates (i,j). 
The size of the rectangle is fixed by \e width and \e height.

\param I : Image associated to the display.
\param i : Row number of the rectangle center
\param j : Column number of the rectangle center
\param angle : Angle of the width side with horizontal
\param width : Width of the rectangle.
\param height : Height of the rectangle.
\param col : Color of the rectangle.
\param e : Line thickness

*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I,
			    unsigned int i, unsigned int j, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col,unsigned int e)
{
	try
	{
		if (I.display != NULL)
		{
			//A, B, C, D, corners of the rectangle
			int ua, va, ub, vb, uc, vc, ud, vd;
			float cosinus = cos(angle);
			float sinus = sin(angle);
			
			ua = (int)(j + 0.5*width*cosinus + 0.5*height*sinus);
			va = (int)(i + 0.5*width*sinus - 0.5*height*cosinus);
			ub = (int)(j + 0.5*width*cosinus - 0.5*height*sinus);
			vb = (int)(i + 0.5*width*sinus + 0.5*height*cosinus);
			uc = (int)(j - 0.5*width*cosinus - 0.5*height*sinus);
			vc = (int)(i - 0.5*width*sinus + 0.5*height*cosinus);
			ud = (int)(j - 0.5*width*cosinus + 0.5*height*sinus);
			vd = (int)(i - 0.5*width*sinus - 0.5*height*cosinus);
	
			( I.display )->displayLine_uv(I, ua, va, ub, vb,col, e);
			( I.display )->displayLine_uv(I, ua, va, ud, vd,col, e);
			( I.display )->displayLine_uv(I, uc, vc, ub, vb,col, e);
			( I.display )->displayLine_uv(I, uc, vc, ud, vd,col, e);
		}
    }
	catch(...)
	{
		vpERROR_TRACE("Error caught in displayRectangle") ;
		throw ;
	}
}

/*!

Display a rectangle in the display window.  The rectangle is defined by its center, its orientation (angle) and its size. The rectangle center
has coordinates (u, v) in the display window. The size of the rectangle is fixed by \e width and \e
height.
\param I : Image associated to the display.
\param u : Column number of the rectangle center
\param v : Row number of the rectangle center
\param angle : Angle of the longest side with horizontal
\param width : Width of the rectangle.
\param height : Height of the rectangle.
\param col : Color of the rectangle.
\param e : Line thickness
*/
void
vpDisplay::displayRectangle_uv(const vpImage<unsigned char> &I,
			    unsigned int u, unsigned int v, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col,unsigned int e)
{
	try
	{
		if ( I.display != NULL )
		{
		( I.display )->displayRectangle ( I, v, u, angle, width, height, col, e ) ;
		}
	}
	catch ( ... )
	{
		vpERROR_TRACE ( "Error caught in displayRectangle_uv" ) ;
		throw ;
	}
}


/*!

Display a rectangle in the display window.  The rectangle is defined by its center, its orientation (angle) and its size. The rectangle center
has coordinates (u, v) in the display window. The size of the rectangle is fixed by \e width and \e
height.
\param I : Image associated to the display.
\param u : Column number of the rectangle center
\param v : Row number of the rectangle center
\param angle : Angle of the longest side with horizontal
\param width : Width of the rectangle.
\param height : Height of the rectangle.
\param col : Color of the rectangle.
\param e : Line thickness
*/
void
vpDisplay::displayRectangle_uv(const vpImage<vpRGBa> &I,
			    unsigned int u, unsigned int v, float angle,
			    unsigned int width, unsigned int height,
			    vpColor::vpColorType col,unsigned int e)
{
	try
	{
		if ( I.display != NULL )
		{
		( I.display )->displayRectangle ( I, v, u, angle, width, height, col, e ) ;
		}
	}
	catch ( ... )
	{
		vpERROR_TRACE ( "Error caught in displayRectangle_uv" ) ;
		throw ;
	}
}

/*! 

  Display a string at coordinates (i,j) in the display
  window overlay.

  To select the font used to print this string, use setFont().

  \param I : Image associated to the display.
  \param i, j : Upper left location (row, column) of the string in the display.
  \param s : String to print in overlay.
  \param c : Color of the text

  \sa setFont()
*/
void
vpDisplay::displayCharString ( const vpImage<unsigned char> &I,
                               int i, int j,const char *s,
                               vpColor::vpColorType c )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( i,j,s,c ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  flushes the output buffer and then waits until all
  requests have been received and processed by the server.

  \warning In vpDisplayX class this function is particular and must be called
  to show the overlay. Because it's time spending, use it parcimoniously.
*/
void vpDisplay::flush ( const vpImage<unsigned char> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->flushDisplay() ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Close the display attached to I.
*/
void vpDisplay::close ( const vpImage<unsigned char> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->closeDisplay() ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  Set the windows title.
  \param I : Image associated to the display window.
  \param windowtitle : Window title.
*/
void
vpDisplay::setTitle ( const vpImage<vpRGBa> &I, const char *windowtitle )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setTitle ( windowtitle ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Set the window position in the screen.

  \param I : Image associated to the display window.
  \param winx, winy : Position of the upper-left window's border in the screen.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void
vpDisplay::setWindowPosition ( const vpImage<vpRGBa> &I, int winx, int winy )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setWindowPosition ( winx, winy ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Set the font of a text printed in the display overlay. To print a
  text you may use displayCharString().

  \param I : Image associated to the display window.
  \param fontname : The expected font name.

  \note Under UNIX, the available fonts are given by
  the "xlsfonts" binary. To choose a font you can also use the
  "xfontsel" binary.

  \sa displayCharString()
*/
void
vpDisplay::setFont ( const vpImage<vpRGBa> &I, const char *fontname )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setFont ( fontname ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  Display a 32bits image in the display window
*/
void
vpDisplay::display ( const vpImage<vpRGBa> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayImage ( I ) ;
    }
    else
    {
      vpERROR_TRACE ( "Display not initialized" ) ;
      throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                   "Display not initialized" ) ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  \brief get the window pixmap and put it in vpRGBa image
*/
void
vpDisplay::getImage ( const vpImage<vpRGBa> &Isrc, vpImage<vpRGBa> &Idest )
{

  try
  {
    if ( Isrc.display != NULL )
    {
      ( Isrc.display )->getImage ( Idest ) ;
    }
    else
    {
      vpERROR_TRACE ( "Display not initialized" ) ;
      throw ( vpDisplayException ( vpDisplayException::notInitializedError,
                                   "Display not initialized" ) ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a point at coordinates (i,j) in the display window
*/


void vpDisplay::displayPoint ( const vpImage<vpRGBa> &I,
                               int i, int j,
                               vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayPoint ( i,j,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (i,j) in the display window
*/
void vpDisplay::displayCross ( const vpImage<vpRGBa> &I,
                               int i, int j,
                               unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCross ( i,j,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (i,j) in the display window
*/
void
vpDisplay::displayCrossLarge ( const vpImage<vpRGBa> &I,
                               int i, int j,
                               unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCrossLarge ( i,j,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (i,j) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle ( const vpImage<vpRGBa> &I,
                           int i, int j, unsigned int r,
                           vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCircle ( i,j,r,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
*/
void vpDisplay::displayLine ( const vpImage<vpRGBa> &I,
                              int i1, int j1,
                              int i2, int j2,
                              vpColor::vpColorType col, unsigned int e )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayLine ( i1,j1,i2,j2,col,e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!  Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine ( const vpImage<vpRGBa> &I,
                                 int i1, int j1,
                                 int i2, int j2,
                                 vpColor::vpColorType col, unsigned int e2 )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayDotLine ( i1,j1,i2,j2,col,e2 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  window
*/
void
vpDisplay::displayArrow ( const vpImage<vpRGBa> &I,
                          int i1, int j1, int i2, int j2,
                          vpColor::vpColorType col,
                          unsigned int L,unsigned int l )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayArrow ( i1,j1,i2,j2,col,L,l ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display a string at coordinates (i,j) in the display
  window overlay.

  To select the font used to print this string, use setFont().

  \param I : Image associated to the display.
  \param i, j : Upper left location (row, column) of the string in the display.
  \param s : String to print in overlay.
  \param c : Color of the text

  \sa setFont()
*/
void
vpDisplay::displayCharString ( const vpImage<vpRGBa> &I,
                               int i, int j, const char *s,
                               vpColor::vpColorType c )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( i,j,s,c ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  flushes the output buffer and then waits until all
  requests have been received and processed by the server
*/
void vpDisplay::flush ( const vpImage<vpRGBa> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->flushDisplay() ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Close the display attached to I.
*/
void vpDisplay::close ( const vpImage<vpRGBa> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->closeDisplay() ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


//--------------
// uv

/*!
  Display a point at coordinates (u,v) in the display window
*/
void vpDisplay::displayPoint_uv ( const vpImage<unsigned char> &I,
                                  int u, int v,
                                  vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayPoint ( v,u,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (u,v) in the display window
*/
void vpDisplay::displayCross_uv ( const vpImage<unsigned char> &I,
                                  int u, int v,
                                  unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCross ( v,u,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (u,v) in the display window
*/
void
vpDisplay::displayCrossLarge_uv ( const vpImage<unsigned char> &I,
                                  int u, int v,
                                  unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCrossLarge ( v,u,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (u,v) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle_uv ( const vpImage<unsigned char> &I,
                              int u, int v, unsigned int r,
                              vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCircle ( v,u,r,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a line from coordinates (u1,v1) to (u2,v2) in the display window.
*/
void vpDisplay::displayLine_uv ( const vpImage<unsigned char> &I,
                                 int u1, int v1,
                                 int u2, int v2,
                                 vpColor::vpColorType col, unsigned int e )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayLine ( v1,u1,v2,u2,col,e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!  Display a dotted line from coordinates (u1,v1) to (u2,v2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine_uv ( const vpImage<unsigned char> &I,
                                    int u1, int v1, int u2, int v2,
                                    vpColor::vpColorType col, unsigned int e2 )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayDotLine ( v1,u1,v2,u2,col,e2 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*! Display an arrow from coordinates (u1,v1) to (u2,v2) in the display
  window
*/
void
vpDisplay::displayArrow_uv ( const vpImage<unsigned char> &I,
                             int u1, int v1, int u2, int v2,
                             vpColor::vpColorType col,
                             unsigned int L,unsigned int l )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayArrow ( v1,u1,v2,u2,col,L,l ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

Display a rectangle in the display window associated to Image \e I.
The rectangle upper left corner has coordinates (u,v). The size of the
rectangle is fixed by \e width and \e height.

\param I : Image. 
\param u : Column number of the rectangle upper corner
\param v : Row number of the rectangle upper corner
\param width : Width of the rectangle.
\param height : Height of the rectangle.
\param col : Color of the rectangle.
\param fill : set as true to fill the rectangle.
\param e : Line thickness

*/
void
vpDisplay::displayRectangle_uv ( const vpImage<unsigned char> &I,
                                 int u, int v,
                                 unsigned int width, unsigned int height,
                                 vpColor::vpColorType col, bool fill,
				 unsigned int e )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( v,u,width,height,col,fill, e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!

  Display a string at coordinates (i,j) in the display
  window overlay.

  To select the font used to print this string, use setFont().

  \param I : Image associated to the display.
  \param u, v : Upper left location (column, row) of the string in the display.
  \param s : String to print in overlay.
  \param c : Color of the text

  \sa setFont()

*/
void
vpDisplay::displayCharString_uv ( const vpImage<unsigned char> &I,
                                  int u, int v, const char *s,
                                  vpColor::vpColorType c )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( v,u,s,c ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a point at coordinates (u,v) in the display window
*/


void vpDisplay::displayPoint_uv ( const vpImage<vpRGBa> &I,
                                  int u, int v,
                                  vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayPoint ( v,u,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (u,v) in the display window
*/
void vpDisplay::displayCross_uv ( const vpImage<vpRGBa> &I,
                                  int u, int v,
                                  unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCross ( v,u,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (u,v) in the display window
*/
void
vpDisplay::displayCrossLarge_uv ( const vpImage<vpRGBa> &I,
                                  int u, int v,
                                  unsigned int size,vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCrossLarge ( v,u,size,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (u,v) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle_uv ( const vpImage<vpRGBa> &I,
                              int u, int v, unsigned int r,
                              vpColor::vpColorType col )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCircle ( v,u,r,col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a line from coordinates (u1,v1) to (u2,v2) in the display window.
*/
void vpDisplay::displayLine_uv ( const vpImage<vpRGBa> &I,
                                 int u1, int v1, int u2, int v2,
                                 vpColor::vpColorType col, unsigned int e )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayLine ( v1,u1,v2,u2,col,e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!  Display a dotted line from coordinates  (u1,v1) to (u2,v2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine_uv ( const vpImage<vpRGBa> &I,
                                    int u1, int v1, int u2, int v2,
                                    vpColor::vpColorType col, unsigned int e2 )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayDotLine ( v1,u1,v2,u2,col,e2 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*! Display an arrow from coordinates  (u1,v1) to (u2,v2) in the display
  window
*/
void
vpDisplay::displayArrow_uv ( const vpImage<vpRGBa> &I,
                             int u1, int v1, int u2, int v2,
                             vpColor::vpColorType col,
                             unsigned int L,unsigned int l )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayArrow ( v1,u1,v2,u2,col,L,l ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display a string at coordinates (i,j) in the display
  window overlay.

  To select the font used to print this string, use setFont().

  \param I : Image associated to the display.
  \param u, v : Upper left location (column, row) of the string in the display.
  \param s : String to print in overlay.
  \param c : Color of the text

  \sa setFont()

*/
void
vpDisplay::displayCharString_uv ( const vpImage<vpRGBa> &I,
                                  int u, int v, const char *s,
                                  vpColor::vpColorType c )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( v,u,s,c ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Wait for a click from one of the mouse button.

  \param I [in] : The displayed image.

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
bool  vpDisplay::getClick ( const vpImage<unsigned char> &I, bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClick(blocking) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!

  Wait for a click from one of the mouse button and get the position
  of the clicked image point.
  
  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param blocking [in] : Blocking behavior.
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.

*/
bool vpDisplay::getClick ( const vpImage<unsigned char> &I,
			   vpImagePoint &ip, bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClick ( ip, blocking ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  Wait for a mouse button click and get the position of the clicked
  image point. The button used to click is also set.
  
  \param I [in] : The displayed image.

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
bool  vpDisplay::getClick ( const vpImage<unsigned char> &I,
                            vpImagePoint &ip, 
                            vpMouseButton::vpMouseButtonType& button,
			    bool blocking)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClick ( ip, button, blocking ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  Wait for a mouse button click release and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The clicked button.

  \param blocking [in] : 
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise. If a
  button is released, the location of the mouse pointer is updated in
  \e ip.
   
*/
bool
vpDisplay::getClickUp ( const vpImage<unsigned char> &I,
                        vpImagePoint &ip,
                        vpMouseButton::vpMouseButtonType& button,
			bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClickUp ( ip, button, blocking ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  Wait for a click.

  \param I [in] : The displayed image.

  \param blocking [in] : 
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. 
*/
bool  vpDisplay::getClick ( const vpImage<vpRGBa> &I, bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClick(blocking) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false;
}


/*!
  Return true when a mouse button is pressed.
  
  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param blocking [in] : 
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.

*/
bool  vpDisplay::getClick ( const vpImage<vpRGBa> &I,
                            vpImagePoint &ip, bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClick ( ip, blocking ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  Return true when a mouse button is pressed.
  
  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The clicked button.

  \param blocking [in] : 
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise. If a
  button is pressed, the location of the mouse pointer is updated in
  \e ip.
  
*/
bool  vpDisplay::getClick ( const vpImage<vpRGBa> &I,
                            vpImagePoint &ip,
                            vpMouseButton::vpMouseButtonType& button,
			    bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClick ( ip, button, blocking ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  Return true when a mouse button is released.
  
  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the clicked image point.

  \param button [out] : The clicked button.

  \param blocking [in] : 
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise. If a
  button is released, the location of the mouse pointer is updated in
  \e ip.
   
*/
bool
vpDisplay::getClickUp ( const vpImage<vpRGBa> &I,
                        vpImagePoint &ip,
                        vpMouseButton::vpMouseButtonType& button,
			bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getClickUp ( ip, button, blocking ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}



/****************************************************************

           Deprecated functions

*****************************************************************/

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  Display the windows title.
  \deprecated Use setTitle() instead.
*/
void
vpDisplay::displayTitle ( const vpImage<unsigned char> &I,
                          const char *windowtitle )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setTitle ( windowtitle ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display the windows title.
  \deprecated Use setTitle() instead.
*/
void
vpDisplay::displayTitle ( const vpImage<vpRGBa> &I, const char *windowtitle )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->setTitle ( windowtitle ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  return true way a button is pressed
*/
bool  vpDisplay::getClick ( const vpImage<unsigned char> &I,
                            unsigned int& i, unsigned int& j, bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, blocking);

      i = (unsigned int) ip.get_i();
      j = (unsigned int) ip.get_j();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
*/
bool  vpDisplay::getClick ( const vpImage<unsigned char> &I,
                            unsigned int& i, unsigned int& j,
                            vpMouseButton::vpMouseButtonType& button,
			    bool blocking)
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, button, blocking);

      i = (unsigned int) ip.get_i();
      j = (unsigned int) ip.get_j();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}


/*!
  return true way a button is pressed
*/
bool  vpDisplay::getClick ( const vpImage<vpRGBa> &I,
                            unsigned int& i, unsigned int& j, bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, blocking);

      i = (unsigned int) ip.get_i();
      j = (unsigned int) ip.get_j();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
*/
bool  vpDisplay::getClick ( const vpImage<vpRGBa> &I,
                            unsigned int& i, unsigned int& j,
                            vpMouseButton::vpMouseButtonType& button,
			                      bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, button, blocking);

      i = (unsigned int) ip.get_i();
      j = (unsigned int) ip.get_j();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}


/*!
  return true way  button is released
*/
bool
vpDisplay::getClickUp ( const vpImage<unsigned char> &I,
                        unsigned int& i, unsigned int& j,
                        vpMouseButton::vpMouseButtonType& button,
			                  bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClickUp ( ip, button, blocking);

      i = (unsigned int) ip.get_i();
      j = (unsigned int) ip.get_j();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  return true way  button is released
*/
bool
vpDisplay::getClickUp ( const vpImage<vpRGBa> &I,
                        unsigned int& i, unsigned int& j,
                        vpMouseButton::vpMouseButtonType& button,
			                  bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClickUp ( ip, button, blocking);

      i = (unsigned int) ip.get_i();
      j = (unsigned int) ip.get_j();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  return true way a button is pressed
*/
bool  vpDisplay::getClick_uv ( const vpImage<unsigned char> &I,
                               unsigned int& u, unsigned int& v,
			       bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, blocking);

      u = (unsigned int) ip.get_u();
      v = (unsigned int) ip.get_v();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
*/
bool  vpDisplay::getClick_uv ( const vpImage<unsigned char> &I,
                               unsigned int& u, unsigned int& v,
                               vpMouseButton::vpMouseButtonType& button,
			                         bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, button, blocking);

      u = (unsigned int) ip.get_u();
      v = (unsigned int) ip.get_v();
    
      return ( ret ) ;
     }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}



/*!
  return true way a button is pressed
*/
bool  vpDisplay::getClick_uv ( const vpImage<vpRGBa> &I,
                               unsigned int& u, unsigned int& v,
			                         bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, blocking);

      u = (unsigned int) ip.get_u();
      v = (unsigned int) ip.get_v();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
*/
bool  vpDisplay::getClick_uv ( const vpImage<vpRGBa> &I,
                               unsigned int& u, unsigned int& v,
                               vpMouseButton::vpMouseButtonType& button,
			       bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClick ( ip, button, blocking);

      u = (unsigned int) ip.get_u();
      v = (unsigned int) ip.get_v();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}


/*!
  return true way  button is released
*/
bool
vpDisplay::getClickUp_uv ( const vpImage<unsigned char> &I,
                           unsigned int& u, unsigned int& v,
                           vpMouseButton::vpMouseButtonType& button,
			                     bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClickUp ( ip, button, blocking);

      u = (unsigned int) ip.get_u();
      v = (unsigned int) ip.get_v();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}

/*!
  return true way  button is released
*/
bool
vpDisplay::getClickUp_uv ( const vpImage<vpRGBa> &I,
                           unsigned int& u, unsigned int& v,
                           vpMouseButton::vpMouseButtonType& button,
			   bool blocking )
{
  try
  {
    if ( I.display != NULL )
    {
      bool ret;
      vpImagePoint ip;

      ret = (I.display )->getClickUp ( ip, button, blocking);

      u = (unsigned int) ip.get_u();
      v = (unsigned int) ip.get_v();
    
      return ( ret ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
  return false ;
}
#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
