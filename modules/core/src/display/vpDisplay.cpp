/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Image display.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <limits>

#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpDisplayException.h>
#include <visp3/core/vpImageConvert.h>

#include <visp3/core/vpPoint.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpMath.h>


/*!
  \file vpDisplay.cpp
  \brief Generic class for image display.
*/

/*!
  Default constructor.
*/
vpDisplay::vpDisplay()
  : displayHasBeenInitialized(false), windowXPosition(0), windowYPosition(0), width(0), height(0), title_() {}

/*!
  Copy constructor.
*/
vpDisplay::vpDisplay(const vpDisplay &d)
  : displayHasBeenInitialized(false), windowXPosition(0), windowYPosition(0), width(0), height(0), title_()
{
  displayHasBeenInitialized = d.displayHasBeenInitialized;
  windowXPosition = d.windowXPosition;
  windowYPosition = d.windowYPosition;

  width  = d.width;
  height = d.height;
}

/*!
  Destructor that desallocates memory.
*/
vpDisplay::~vpDisplay()
{
  displayHasBeenInitialized = false ;
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
  Set the windows title.
  \note This functionality is not implemented when vpDisplayOpenCV is used.

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
  Set the window background.

  \param I : Image associated to the display window.
  \param color: Background color.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void 
vpDisplay::setBackground(const vpImage<unsigned char> &I, const vpColor &color)
{
 try
  {
    if ( I.display != NULL )
    {
      ( I.display )->clearDisplay ( color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), close()
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


void
vpDisplay::displayROI(const vpImage<unsigned char> &I, const vpRect &roi)
{
  vpImagePoint topLeft;
  double top = floor(roi.getTop());
  double left = floor(roi.getLeft());
  double roiheight = floor(roi.getHeight());
  double roiwidth = floor(roi.getWidth());
  double iheight = (double)(I.getHeight());
  double iwidth = (double)(I.getWidth());
  
  if (top < 0 || top > iheight || left < 0 || left > iwidth || top+roiheight > iheight || left+roiwidth > iwidth)
  {
    vpERROR_TRACE ( "Region of interest outside of the image" ) ;
    throw ( vpException ( vpException::dimensionError,"Region of interest outside of the image" ) ) ;
  }
  
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayImageROI ( I , vpImagePoint(top,left), (unsigned int)roiwidth,(unsigned int)roiheight ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}



/*!
  Get the window pixmap and put it in vpRGBa image.

  The code below shows how to use this method.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>

int main()
{
  vpImage<unsigned char> I(240, 320); // Create a black grey level image
  vpImage<vpRGBa> Ioverlay;

  vpDisplay *d;

  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Updates the color image with the original loaded image and the overlay
  vpDisplay::getImage(I, Ioverlay) ;

  // Write the color image on the disk
  std::string ofilename("overlay.ppm");
  vpImageIo::write(Ioverlay, ofilename) ;

  // Wait for a click in the display window
  vpDisplay::getClick(I);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
  \endcode
  
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
      vpImageConvert::convert(Isrc, Idest);
//      vpERROR_TRACE ( "Display not initialized" ) ;
//      throw ( vpDisplayException ( vpDisplayException::notInitializedError,
//                                   "Display not initialized" ) ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!

  Display the projection of an object frame represented by 3 arrows in
  the image.

  \param I : The image associated to the display.

  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.

  \param cam : Camera intrinsic parameters.

  \param size : Size of the object frame.

  \param color : Color used to display the frame in the image.
  
  \param thickness : the thickness of the line.

  \param offset : Offset in pixels applied to the frame origin location in the image.
  
*/
void
vpDisplay::displayFrame ( const vpImage<unsigned char> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, const vpColor &color,
                          unsigned int thickness,
                          vpImagePoint offset)
{
  // used by display
  vpPoint o( 0.0,  0.0,  0.0);
  vpPoint x(size,  0.0,  0.0);
  vpPoint y( 0.0, size,  0.0);
  vpPoint z( 0.0,  0.0, size);

  o.track ( cMo ) ;
  x.track ( cMo ) ;
  y.track ( cMo ) ;
  z.track ( cMo ) ;

  vpImagePoint ipo, ip1;

  if ( color == vpColor::none )
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::red, 4*thickness, 2*thickness, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::green, 4*thickness, 2*thickness, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::blue, 4*thickness, 2*thickness, thickness) ;
  }
  else
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness) ;

  }
}


/*!

  Display the projection of an object frame represented by 3 arrows in
  the image.

  \param I : The image associated to the display.

  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.

  \param cam : Camera intrinsic parameters.

  \param size : Size of the object frame.

  \param color : Color used to display the frame in the image.
  
  \param thickness : the thickness of the line
  
  \param offset : Offset in pixels applied to the frame origin location in the image.
*/
void
vpDisplay::displayFrame ( const vpImage<vpRGBa> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, const vpColor &color,
                          unsigned int thickness,
                          vpImagePoint offset)
{
  // used by display
  vpPoint o( 0.0,  0.0,  0.0);
  vpPoint x(size,  0.0,  0.0);
  vpPoint y( 0.0, size,  0.0);
  vpPoint z( 0.0,  0.0, size);

  o.track ( cMo ) ;
  x.track ( cMo ) ;
  y.track ( cMo ) ;
  z.track ( cMo ) ;

  vpImagePoint ipo, ip1;
  if ( color == vpColor::none )
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::red, 4, 2, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, vpColor::green, 4, 2, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I,ipo+offset, ip1+offset, vpColor::blue, 4, 2, thickness) ;
  }
  else
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo+offset, ip1+offset, color, 4*thickness, 2*thickness, thickness) ;

  }
}

/*!

  Display the projection of an object camera represented by a cone in
  the image.

  \param I : The image associated to the display.

  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.

  \param cam : Camera intrinsic parameters.

  \param size : Size of the object camera.

  \param color : Color used to display the camera in the image.

  \param thickness : Thickness of the graphics drawing.
  
*/
void
vpDisplay::displayCamera ( const vpImage<unsigned char> &I,
                           const vpHomogeneousMatrix &cMo,
                           const vpCameraParameters &cam,
                           double size, const vpColor &color,
                           unsigned int thickness)
{
  // used by display
  double halfSize = size/2.0;
  vpPoint pt[5];
  pt[0].setWorldCoordinates ( -halfSize,-halfSize,0.0 );
  pt[1].setWorldCoordinates ( halfSize,-halfSize,0.0 );
  pt[2].setWorldCoordinates ( halfSize,halfSize,0.0 );
  pt[3].setWorldCoordinates ( -halfSize,halfSize,0.0 );
  pt[4].setWorldCoordinates ( 0.0,0.0,-size );
  
  for (int i = 0; i < 5; i++)
    pt[i].track ( cMo ) ;

  vpImagePoint ip, ip_1, ip0;
  vpMeterPixelConversion::convertPoint ( cam, pt[4].p[0], pt[4].p[1], ip0);
  
  for (int i = 0; i < 4; i++)
  {
    vpMeterPixelConversion::convertPoint ( cam, pt[i].p[0], pt[i].p[1], ip_1);
    vpMeterPixelConversion::convertPoint ( cam, pt[(i+1)%4].p[0], pt[(i+1)%4].p[1], ip);
    vpDisplay::displayLine ( I, ip_1, ip, color, thickness);
    vpDisplay::displayLine ( I, ip0, ip_1, color, thickness);
  }
}


/*!

  Display the projection of an object camera represented by a cone in
  the image.

  \param I : The image associated to the display.

  \param cMo : Homogeneous matrix that gives the transformation
  between the camera frame and the object frame to project in the
  image.

  \param cam : Camera intrinsic parameters.

  \param size : Size of the object camera.

  \param color : Color used to display the camera in the image.

  \param thickness : Thickness of the graphics drawing.
  
*/
void
vpDisplay::displayCamera( const vpImage<vpRGBa> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, const vpColor &color,
                          unsigned int thickness)
{
  // used by display
  double halfSize = size/2.0;
  vpPoint pt[5];
  pt[0].setWorldCoordinates ( -halfSize,-halfSize,0.0 );
  pt[1].setWorldCoordinates ( halfSize,-halfSize,0.0 );
  pt[2].setWorldCoordinates ( halfSize,halfSize,0.0 );
  pt[3].setWorldCoordinates ( -halfSize,halfSize,0.0 );
  pt[4].setWorldCoordinates ( 0.0,0.0,-size );
  
  for (int i = 0; i < 5; i++)
    pt[i].track ( cMo ) ;

  vpImagePoint ip, ip_1, ip0;
  vpMeterPixelConversion::convertPoint ( cam, pt[4].p[0], pt[4].p[1], ip0);
  
  for (int i = 0; i < 4; i++)
  {
    vpMeterPixelConversion::convertPoint ( cam, pt[i].p[0], pt[i].p[1], ip_1);
    vpMeterPixelConversion::convertPoint ( cam, pt[(i+1)%4].p[0], pt[(i+1)%4].p[1], ip);
    vpDisplay::displayLine ( I, ip_1, ip, color, thickness);
    vpDisplay::displayLine ( I, ip0, ip_1, color, thickness);
  }
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void
vpDisplay::displayArrow ( const vpImage<unsigned char> &I,
                          const vpImagePoint &ip1, const vpImagePoint &ip2,
                          const vpColor &color,
                          unsigned int w,unsigned int h,
			  unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayArrow ( ip1, ip2, color, w, h, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/

void
vpDisplay::displayArrow ( const vpImage<vpRGBa> &I,
                          const vpImagePoint &ip1, const vpImagePoint &ip2,
                          const vpColor &color,
                          unsigned int w,unsigned int h,
			  unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayArrow ( ip1, ip2, color, w, h, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*! 
  Display an arrow from image point (i1,j1) to  image point (i2,j2).

  \param I : The image associated to the display.
  \param i1,j1 : Initial image point.
  \param i2,j2 : Final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void
vpDisplay::displayArrow ( const vpImage<unsigned char> &I,
                          int i1, int j1, int i2, int j2,
                          const vpColor &color,
			  unsigned int w, unsigned int h,
			  unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_i(i1);
      ip1.set_j(j1);
      ip2.set_i(i2);
      ip2.set_j(j2);
      ( I.display )->displayArrow ( ip1, ip2, color, w, h, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display an arrow from image point (i1,j1) to  image point (i2,j2).

  \param I : The image associated to the display.
  \param i1,j1 : Initial image point.
  \param i2,j2 : Final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.

*/
void
vpDisplay::displayArrow ( const vpImage<vpRGBa> &I,
                          int i1, int j1, int i2, int j2,
                          const vpColor &color,
			  unsigned int w, unsigned int h,
			  unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_i(i1);
      ip1.set_j(j1);
      ip2.set_i(i2);
      ip2.set_j(j2);

      ( I.display )->displayArrow ( ip1, ip2, color, w, h, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*! 

  Display a string at the image point \e ip location.
  Use rather displayText() that does the same.
    
  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont(), displayText()

*/
void
vpDisplay::displayCharString ( const vpImage<unsigned char> &I,
                               const vpImagePoint &ip, const char *string,
			       const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( ip, string, color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*! 

  Display a string at the image point \e ip location.
  Use rather displayText() that does the same.

  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont(), displayText()

*/
void
vpDisplay::displayCharString ( const vpImage<vpRGBa> &I,
                               const vpImagePoint &ip, const char *string,
			       const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( ip, string, color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*! 

  Display a string at the image point (i,j) location.
  Use rather displayText() that does the same.

  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont(), displayText()

*/
void
vpDisplay::displayCharString ( const vpImage<unsigned char> &I,
                               int i, int j, const char *string,
			       const vpColor &color)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );

      ( I.display )->displayCharString ( ip, string, color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display a string at the image point (i,j) location.
  Use rather displayText() that does the same.

  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont(), displayText()

*/
void
vpDisplay::displayCharString ( const vpImage<vpRGBa> &I,
                               int i, int j, const char *string,
                               const vpColor &color)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayCharString ( ip, string, color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()

*/
void
vpDisplay::displayText ( const vpImage<unsigned char> &I,
                         const vpImagePoint &ip, const std::string &s,
                         const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( ip, s.c_str(), color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display a string at the image point \e ip location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()

*/
void
vpDisplay::displayText ( const vpImage<vpRGBa> &I,
                         const vpImagePoint &ip, const std::string &s,
                         const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCharString ( ip, s.c_str(), color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display a string at the image point (i,j) location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()

*/
void
vpDisplay::displayText ( const vpImage<unsigned char> &I,
                         int i, int j, const std::string &s,
                         const vpColor &color)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );

      ( I.display )->displayCharString ( ip, s.c_str(), color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  Display a string at the image point (i,j) location.

  To select the font used to display the string, use setFont().

  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param s : String to display in overlay.
  \param color : String color.

  \sa setFont()

*/
void
vpDisplay::displayText ( const vpImage<vpRGBa> &I,
                         int i, int j, const std::string &s,
                         const vpColor &color)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayCharString ( ip, s.c_str(), color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a circle.
  \param I : The image associated to the display.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the circle. This parameter is only useful 
  when \e fill is set to false.
*/
void
vpDisplay::displayCircle ( const vpImage<unsigned char> &I,
                           const vpImagePoint &center, unsigned int radius,
			   const vpColor &color,
			   bool fill,
			   unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCircle ( center, radius, color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a circle.
  \param I : The image associated to the display.
  \param center : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the circle. This parameter is only useful 
  when \e fill is set to false.
*/
void
vpDisplay::displayCircle ( const vpImage<vpRGBa> &I,
                           const vpImagePoint &center, unsigned int radius,
			   const vpColor &color,
			   bool fill,
			   unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCircle ( center, radius, color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!

  Display a circle.
  \param I : The image associated to the display.
  \param i,j : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the circle. This parameter is only useful 
  when \e fill is set to false.

*/
void
vpDisplay::displayCircle ( const vpImage<unsigned char> &I,
                           int i, int j,  unsigned int radius,
			   const vpColor &color,
			   bool fill,
			   unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );

      ( I.display )->displayCircle ( ip, radius, color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!

  Display a circle.
  \param I : The image associated to the display.
  \param i,j : Circle center position.
  \param radius : Circle radius.
  \param color : Circle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the circle. This parameter is only useful 
  when \e fill is set to false.

*/
void
vpDisplay::displayCircle ( const vpImage<vpRGBa> &I,
                           int i, int j, unsigned int radius,
			   const vpColor &color,
			   bool fill,
			   unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayCircle ( ip, radius, color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a cross at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross ( const vpImage<unsigned char> &I,
                               const vpImagePoint &ip, unsigned int size,
			       const vpColor &color, 
			       unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCross ( ip, size, color, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a cross at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross ( const vpImage<vpRGBa> &I,
                               const vpImagePoint &ip, unsigned int size,
			       const vpColor &color, 
			       unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayCross ( ip, size, color, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a cross at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross ( const vpImage<unsigned char> &I,
                               int i, int j,
                               unsigned int size, const vpColor &color, 
			       unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );

      ( I.display )->displayCross ( ip, size, color, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a cross at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplay::displayCross ( const vpImage<vpRGBa> &I,
                               int i, int j,
			       unsigned int size, const vpColor &color, 
			       unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayCross ( ip, size, color, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine ( const vpImage<unsigned char> &I,
                                 const vpImagePoint &ip1, 
				 const vpImagePoint &ip2,
				 const vpColor &color, 
				 unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayDotLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine ( const vpImage<vpRGBa> &I,
                                 const vpImagePoint &ip1, 
				 const vpImagePoint &ip2,
				 const vpColor &color, 
				 unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayDotLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a dashed line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Dashed line thickness.

*/
void vpDisplay::displayDotLine ( const vpImage<unsigned char> &I,
                                 int i1, int j1, int i2, int j2,
                                 const vpColor &color, 
				 unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_i( i1 );
      ip1.set_j( j1 );
      ip2.set_i( i2 );
      ip2.set_j( j2 );
      ( I.display )->displayDotLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!  
  Display a dashed line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Dashed line thickness.
*/
void vpDisplay::displayDotLine ( const vpImage<vpRGBa> &I,
                                 int i1, int j1,
                                 int i2, int j2,
                                 const vpColor &color, 
				 unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_i( i1 );
      ip1.set_j( j1 );
      ip2.set_i( i2 );
      ip2.set_j( j2 );
      ( I.display )->displayDotLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine ( const vpImage<unsigned char> &I,
                              const vpImagePoint &ip1, 
			      const vpImagePoint &ip2,
			      const vpColor &color, 
			      unsigned int thickness )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Line thickness.

*/
void vpDisplay::displayLine ( const vpImage<unsigned char> &I,
                              int i1, int j1, int i2, int j2,
			      const vpColor &color, 
			      unsigned int thickness )
{

  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_i( i1 );
      ip1.set_j( j1 );
      ip2.set_i( i2 );
      ip2.set_j( j2 );
      ( I.display )->displayLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!

  Display a line from image point (i1,j1) to image point (i2,j2).
  \param I : The image associated to the display.
  \param i1,j1: Initial image point.
  \param i2,j2: Final image point.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine ( const vpImage<vpRGBa> &I,
                              int i1, int j1,
                              int i2, int j2,
			      const vpColor &color, 
			      unsigned int thickness )
{

  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_i( i1 );
      ip1.set_j( j1 );
      ip2.set_i( i2 );
      ip2.set_j( j2 );
      ( I.display )->displayLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param I : The image associated to the display.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplay::displayLine ( const vpImage<vpRGBa> &I,
                              const vpImagePoint &ip1, 
			      const vpImagePoint &ip2,
			      const vpColor &color, 
			      unsigned int thickness )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayLine ( ip1, ip2, color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a point at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point
*/
void vpDisplay::displayPoint ( const vpImage<unsigned char> &I,
                               const vpImagePoint &ip,
                               const vpColor &color,
                               unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      if (thickness == 1)
        ( I.display )->displayPoint ( ip, color ) ;
      else {
        vpRect rect(0, 0, thickness, thickness);
        rect.moveCenter(ip);
        ( I.display )->displayRectangle ( rect, color, true ) ;
      }
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a point at the image point \e ip location.
  \param I : The image associated to the display.
  \param ip : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point
*/
void vpDisplay::displayPoint ( const vpImage<vpRGBa> &I,
                               const vpImagePoint &ip,
                               const vpColor &color,
                               unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      if (thickness == 1)
        ( I.display )->displayPoint ( ip, color ) ;
      else {
        vpRect rect(0, 0, thickness, thickness);
        rect.moveCenter(ip);
        ( I.display )->displayRectangle ( rect, color, true ) ;
      }
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a point at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point
*/
void vpDisplay::displayPoint ( const vpImage<unsigned char> &I,
                               int i, int j,
                               const vpColor &color,
                               unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      if (thickness == 1)
        ( I.display )->displayPoint ( ip, color ) ;
      else {
        vpRect rect(0, 0, thickness, thickness);
        rect.moveCenter(ip);
        ( I.display )->displayRectangle ( rect, color, true ) ;
      }
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  Display a point at the image point (i,j) location.
  \param I : The image associated to the display.
  \param i,j : Point location.
  \param color : Point color.
  \param thickness : Thickness of the point

*/
void vpDisplay::displayPoint ( const vpImage<vpRGBa> &I,
                               int i, int j,
                               const vpColor &color,
                               unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      if (thickness == 1)
        ( I.display )->displayPoint ( ip, color ) ;
      else {
        vpRect rect(0, 0, thickness, thickness);
        rect.moveCenter(ip);
        ( I.display )->displayRectangle ( rect, color, true ) ;
      }
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a polygon defined by a vector of image points.
  \param I : The image associated to the display.
  \param vip : Vector of image point that define the vertexes of the polygon.
  \param color : Line color.
  \param thickness : Line thickness.

*/
void
vpDisplay::displayPolygon(const vpImage<unsigned char> &I,
                          const std::vector<vpImagePoint> &vip,
                          const vpColor &color,
                          unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      for (unsigned int i=0; i< vip.size(); i++)
        ( I.display )->displayLine ( vip[i], vip[(i+1)%vip.size()], color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display a polygon defined by a vector of image points.
  \param I : The image associated to the display.
  \param vip : Vector of image point that define the vertexes of the polygon.
  \param color : Line color.
  \param thickness : Line thickness.

*/
void
vpDisplay::displayPolygon(const vpImage<vpRGBa> &I,
                          const std::vector<vpImagePoint> &vip,
                          const vpColor &color,
                          unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      for (unsigned int i=0; i< vip.size(); i++)
        ( I.display )->displayLine ( vip[i], vip[(i+1)%vip.size()], color, thickness );
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!  
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle ( const vpImage<unsigned char> &I,
                              const vpImagePoint &topLeft,
			      unsigned int width, unsigned int height,
			      const vpColor &color, bool fill,
			      unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( topLeft, width, height, color, 
					fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!  
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle ( const vpImage<unsigned char> &I,
                              const vpImagePoint &topLeft,
			      const vpImagePoint &bottomRight,
			      const vpColor &color, bool fill,
			      unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( topLeft, bottomRight, color, 
					fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void
vpDisplay::displayRectangle ( const vpImage<unsigned char> &I,
                              const vpRect &rectangle,
			      const vpColor &color, bool fill,
			      unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( rectangle, color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!

  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param center : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left 
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle. 

*/
void
vpDisplay::displayRectangle(const vpImage<unsigned char> &I,
			    const vpImagePoint &center,
			    float angle,
			    unsigned int width, unsigned int height,
			    const vpColor &color,
			    unsigned int thickness)
{
  try
    {
      if (I.display != NULL)
	{
	  double i = center.get_i();
	  double j = center.get_j();

	  //A, B, C, D, corners of the rectangle clockwise
	  vpImagePoint ipa, ipb, ipc, ipd;
	  double cosinus = cos(angle);
	  double sinus = sin(angle);
	  ipa.set_u(j + 0.5*width*cosinus + 0.5*height*sinus);
	  ipa.set_v(i + 0.5*width*sinus - 0.5*height*cosinus);
	  ipb.set_u(j + 0.5*width*cosinus - 0.5*height*sinus);
	  ipb.set_v(i + 0.5*width*sinus + 0.5*height*cosinus);
	  ipc.set_u(j - 0.5*width*cosinus - 0.5*height*sinus);
	  ipc.set_v(i - 0.5*width*sinus + 0.5*height*cosinus);
	  ipd.set_u(j - 0.5*width*cosinus + 0.5*height*sinus);
	  ipd.set_v(i - 0.5*width*sinus - 0.5*height*cosinus);

	  ( I.display )->displayLine(I, ipa, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipa, ipd, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipd, color, thickness);
	}
    }
  catch(...)
    {
      vpERROR_TRACE("Error caught in displayRectangle") ;
      throw ;
    }
}

/*!  
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle ( const vpImage<vpRGBa> &I,
                              const vpImagePoint &topLeft,
			      unsigned int width, unsigned int height,
			      const vpColor &color, bool fill,
			      unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( topLeft, width, height, color, 
					fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!  
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.
*/
void
vpDisplay::displayRectangle ( const vpImage<vpRGBa> &I,
                              const vpImagePoint &topLeft,
			      const vpImagePoint &bottomRight,
			      const vpColor &color, bool fill,
			      unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( topLeft, bottomRight, color, 
					fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void
vpDisplay::displayRectangle ( const vpImage<vpRGBa> &I,
                              const vpRect &rectangle,
			      const vpColor &color, bool fill,
			      unsigned int thickness )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayRectangle ( rectangle, color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!

  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param center : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left 
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle. 

*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I,
			    const vpImagePoint &center,
			    float angle,
			    unsigned int width, unsigned int height,
			    const vpColor &color,
			    unsigned int thickness)
{
  try
    {
      if (I.display != NULL)
	{
	  double i = center.get_i();
	  double j = center.get_j();

	  //A, B, C, D, corners of the rectangle clockwise
	  vpImagePoint ipa, ipb, ipc, ipd;
	  double cosinus = cos(angle);
	  double sinus = sin(angle);
	  ipa.set_u(j + 0.5*width*cosinus + 0.5*height*sinus);
	  ipa.set_v(i + 0.5*width*sinus - 0.5*height*cosinus);
	  ipb.set_u(j + 0.5*width*cosinus - 0.5*height*sinus);
	  ipb.set_v(i + 0.5*width*sinus + 0.5*height*cosinus);
	  ipc.set_u(j - 0.5*width*cosinus - 0.5*height*sinus);
	  ipc.set_v(i - 0.5*width*sinus + 0.5*height*cosinus);
	  ipd.set_u(j - 0.5*width*cosinus + 0.5*height*sinus);
	  ipd.set_v(i - 0.5*width*sinus - 0.5*height*cosinus);

	  ( I.display )->displayLine(I, ipa, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipa, ipd, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipd, color, thickness);
	}
    }
  catch(...)
    {
      vpERROR_TRACE("Error caught in displayRectangle") ;
      throw ;
    }
}

/*!

  Display a rectangle with (i,j) as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param i,j : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void
vpDisplay::displayRectangle ( const vpImage<unsigned char> &I,
                              int i, int j,
			      unsigned int width, unsigned int height,
                              const vpColor &color, bool fill,
			      unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint topLeft;
      topLeft.set_i( i );
      topLeft.set_j( j );

      ( I.display )->displayRectangle ( topLeft, width, height, 
					color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!

  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param i,j : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left 
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle. 

*/
void
vpDisplay::displayRectangle(const vpImage<unsigned char> &I,
			    unsigned int i, unsigned int j, float angle,
			    unsigned int width, unsigned int height,
			    const vpColor &color,unsigned int thickness)
{
  try
    {
      if (I.display != NULL)
	{
	  //A, B, C, D, corners of the rectangle clockwise
	  vpImagePoint ipa, ipb, ipc, ipd;
	  float cosinus = cos(angle);
	  float sinus = sin(angle);
	  ipa.set_u(j + 0.5*width*cosinus + 0.5*height*sinus);
	  ipa.set_v(i + 0.5*width*sinus - 0.5*height*cosinus);
	  ipb.set_u(j + 0.5*width*cosinus - 0.5*height*sinus);
	  ipb.set_v(i + 0.5*width*sinus + 0.5*height*cosinus);
	  ipc.set_u(j - 0.5*width*cosinus - 0.5*height*sinus);
	  ipc.set_v(i - 0.5*width*sinus + 0.5*height*cosinus);
	  ipd.set_u(j - 0.5*width*cosinus + 0.5*height*sinus);
	  ipd.set_v(i - 0.5*width*sinus - 0.5*height*cosinus);

	  ( I.display )->displayLine(I, ipa, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipa, ipd, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipd, color, thickness);
	}
    }
  catch(...)
    {
      vpERROR_TRACE("Error caught in displayRectangle") ;
      throw ;
    }
}

/*!

  Display a rectangle with (i,j) as the top-left corner and \e
  width and \e height the rectangle size.

  \param I : The image associated to the display.
  \param i,j : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.

  \param thickness : Thickness of the four lines used to display the
  rectangle. This parameter is only useful when \e fill is set to
  false.

*/
void
vpDisplay::displayRectangle ( const vpImage<vpRGBa> &I,
                              int i, int j,
			      unsigned int width, unsigned int height,
                              const vpColor &color, bool fill,
			      unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint topLeft;
      topLeft.set_i( i );
      topLeft.set_j( j );

      ( I.display )->displayRectangle ( topLeft, width, height, 
					color, fill, thickness ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!

  Display a rectangle defined by its center, its orientation (angle)
  and its size.

  \param I : Image associated to the display.
  \param i,j : Rectangle center point.
  \param angle : Angle in radians width an horizontal axis oriented from left 
  to right.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param thickness : Thickness of the four lines used to display the
  rectangle. 
*/
void
vpDisplay::displayRectangle(const vpImage<vpRGBa> &I,
			    unsigned int i, unsigned int j, float angle,
			    unsigned int width, unsigned int height,
			    const vpColor &color, unsigned int thickness)
{
  try
    {
      if (I.display != NULL)
	{
	  //A, B, C, D, corners of the rectangle clockwise
	  vpImagePoint ipa, ipb, ipc, ipd;
	  float cosinus = cos(angle);
	  float sinus = sin(angle);
	  ipa.set_u(j + 0.5*width*cosinus + 0.5*height*sinus);
	  ipa.set_v(i + 0.5*width*sinus - 0.5*height*cosinus);
	  ipb.set_u(j + 0.5*width*cosinus - 0.5*height*sinus);
	  ipb.set_v(i + 0.5*width*sinus + 0.5*height*cosinus);
	  ipc.set_u(j - 0.5*width*cosinus - 0.5*height*sinus);
	  ipc.set_v(i - 0.5*width*sinus + 0.5*height*cosinus);
	  ipd.set_u(j - 0.5*width*cosinus + 0.5*height*sinus);
	  ipd.set_v(i - 0.5*width*sinus - 0.5*height*cosinus);

	  ( I.display )->displayLine(I, ipa, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipa, ipd, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipb, color, thickness);
	  ( I.display )->displayLine(I, ipc, ipd, color, thickness);
	}
    }
  catch(...)
    {
      vpERROR_TRACE("Error caught in displayRectangle") ;
      throw ;
    }
}

/*!
  Flushes the output buffer associated to image \e I display.
  It's necessary to use this function to see the results of any drawing.

  \warning This function is particular and must be called
  to show the overlay. Because it's time spending, use it parcimoniously.

  \code
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>

int main() {
  vpImage<unsigned char> I(240, 380);
  vpDisplayGDI d;
  d.init(I);
  vpDisplay::display(I); // display the image
  vpImagePoint center;
  unsigned int radius = 100;
  vpDisplay::displayCircle(I, center, radius, vpColor::red);

  vpDisplay::flush(I); // Mendatory to display the requested features. 
}
  \endcode
    
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

void vpDisplay::flushROI ( const vpImage<unsigned char> &I, const vpRect &roi )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->flushDisplayROI(roi.getTopLeft(),(unsigned int)roi.getWidth(),(unsigned int)roi.getHeight()) ;
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
void vpDisplay::close ( vpImage<unsigned char> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->closeDisplay() ;
      I.display = NULL;
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
  \note This functionality is not implemented when vpDisplayOpenCV is used.

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
  Set the window background.

  \param I : Image associated to the display window.
  \param color: Background color.

  \exception vpDisplayException::notInitializedError : If the video
  device is not initialized.
*/
void 
vpDisplay::setBackground(const vpImage<vpRGBa> &I, const vpColor &color)
{
 try
  {
    if ( I.display != NULL )
    {
      ( I.display )->clearDisplay ( color ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning Suppress the overlay drawing.

  \param I : Image to display.

  \sa init(), close()
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
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

void
vpDisplay::displayROI(const vpImage<vpRGBa> &I, const vpRect &roi)
{
  vpImagePoint topLeft;
  double top = floor(roi.getTop());
  double left = floor(roi.getLeft());
  double roiheight = floor(roi.getHeight());
  double roiwidth = floor(roi.getWidth());
  double iheight = (double)(I.getHeight());
  double iwidth = (double)(I.getWidth());
  
  if (top < 0 || top >= iheight || left < 0 || left >= iwidth || top+roiheight >= iheight || left+roiwidth >= iwidth)
  {
    vpERROR_TRACE ( "Region of interest outside of the image" ) ;
    throw ( vpException ( vpException::dimensionError,
                                   "Region of interest outside of the image" ) ) ;
  }
  
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayImageROI ( I , vpImagePoint(top,left), (unsigned int)roiwidth,(unsigned int)roiheight ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  Get the window pixmap and put it in vpRGBa image.

  The code below shows how to use this method.
  \code
#include <visp3/core/vpConfig.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>

int main()
{
  vpImage<vpRGBa> I(240, 320); // Create a black RGB color image
  vpImage<vpRGBa> Ioverlay;

  vpDisplay *d;

  // Depending on the detected third party libraries, we instantiate here the
  // first video device which is available
#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Draw a red rectangle in the display overlay (foreground)
  vpDisplay::displayRectangle(I, 10, 10, 100, 20, vpColor::red, true);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Updates the color image with the original loaded image and the overlay
  vpDisplay::getImage(I, Ioverlay) ;

  // Write the color image on the disk
  std::string ofilename("overlay.ppm");
  vpImageIo::write(Ioverlay, ofilename) ;

  // Wait for a click in the display window
  vpDisplay::getClick(I);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
  \endcode

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
    else {
      Idest = Isrc;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  Flushes the output buffer associated to image \e I display.
  It's necessary to use this function to see the results of any drawing.

  \warning This function is particular and must be called
  to show the overlay. Because it's time spending, use it parcimoniously.

  \code
#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/core/vpColor.h>
#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRGBa.h>

int main() {
  vpImage<vpRGBa> I(240, 380);
  vpDisplayGDI d;
  d.init(I);
  vpDisplay::display(I); // display the image
  vpImagePoint center;
  unsigned int radius = 100;
  vpDisplay::displayCircle(I, center, radius, vpColor::red);

  vpDisplay::flush(I); // Mendatory to display the requested features. 
}
  \endcode
    
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

void vpDisplay::flushROI ( const vpImage<vpRGBa> &I, const vpRect &roi )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->flushDisplayROI(roi.getTopLeft(),(unsigned int)roi.getWidth(),(unsigned int)roi.getHeight()) ;
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
void vpDisplay::close ( vpImage<vpRGBa> &I )
{

  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->closeDisplay() ;
      I.display = NULL;
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
  Wait for a mouse button click and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The button used to click.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise.

*/
bool  vpDisplay::getClick ( const vpImage<unsigned char> &I,
                            vpMouseButton::vpMouseButtonType& button,
                            bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClick(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The button used to click.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    pressed and then returns always true.
  - When set to false, returns true only if a mouse button is
    pressed, otherwise returns false.

  \return true if a mouse button is pressed, false otherwise.

*/
bool  vpDisplay::getClick ( const vpImage<vpRGBa> &I,
                            vpMouseButton::vpMouseButtonType& button,
                            bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClick(I, ip, button, blocking);
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
  Wait for a mouse button click release and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The clicked button.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise.

*/
bool  vpDisplay::getClickUp ( const vpImage<unsigned char> &I,
                            vpMouseButton::vpMouseButtonType& button,
                            bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClickUp(I, ip, button, blocking);
}

/*!
  Wait for a mouse button click release and get the position of the clicked
  image point. The button used to click is also set.

  \param I [in] : The displayed image.

  \param button [out] : The clicked button.

  \param blocking [in] :
  - When set to true, this method waits until a mouse button is
    released and then returns always true.
  - When set to false, returns true only if a mouse button is
    released, otherwise returns false.

  \return true if a mouse button is released, false otherwise.

*/
bool  vpDisplay::getClickUp ( const vpImage<vpRGBa> &I,
                            vpMouseButton::vpMouseButtonType& button,
                            bool blocking)
{
  vpImagePoint ip;
  return vpDisplay::getClickUp(I, ip, button, blocking);
}

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

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

  Below you will find an example showing how to use this method.
\code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>

int main()
{
  vpImage<unsigned char> I(240, 320); // Create a black image

  vpDisplay *d;

#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#else
  std::cout << "Sorry, no video device is available" << std::endl;
  return -1;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;

  // Non blocking keyboard event loop
  int cpt_event = 0;
  bool event;
  char key[10];
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    event = vpDisplay::getKeyboardEvent(I, &key[0], false);
    if (event) {
      std::cout << "Key detected: " << key << std::endl;
      cpt_event ++;
    }

    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
\endcode
*/
bool
vpDisplay::getKeyboardEvent(const vpImage<unsigned char> &I, bool blocking)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getKeyboardEvent ( blocking ) ;
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

  Get a keyboard event.

  \param I [in] : The displayed image.

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

  Below you will find an example showing how to use this method.
\code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>

int main()
{
  vpImage<unsigned char> I(240, 320); // Create a black image

  vpDisplay *d;

#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#else
  std::cout << "Sorry, no video device is available" << std::endl;
  return -1;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;

  // Non blocking keyboard event loop
  int cpt_event = 0;
  bool event;
  char key[10];
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    event = vpDisplay::getKeyboardEvent(I, &key[0], false);
    if (event) {
      std::cout << "Key detected: " << key << std::endl;
      cpt_event ++;
    }

    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
\endcode
*/
bool
vpDisplay::getKeyboardEvent(const vpImage<unsigned char> &I,
			    char *string, bool blocking)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getKeyboardEvent ( string, blocking ) ;
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
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.
  
  \return true if a pointer motion event was received, false otherwise.
  
*/
bool 
vpDisplay::getPointerMotionEvent (const vpImage<unsigned char> &I,
				  vpImagePoint &ip)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getPointerMotionEvent ( ip ) ;
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
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.
  
  \return true.
  
*/
bool 
vpDisplay::getPointerPosition (const vpImage<unsigned char> &I,
				  vpImagePoint &ip)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getPointerPosition ( ip ) ;
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

/*!
  Get a keyboard event.

  \param I [in] : The displayed image.

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

  Below you will find an example showing how to use this method.
\code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>

int main()
{
  vpImage<vpRGBa> I(240, 320); // Create a black RGB color image

  vpDisplay *d;

#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#else
  std::cout << "Sorry, no video device is available" << std::endl;
  return -1;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;

  // Non blocking keyboard event loop
  int cpt_event = 0;
  bool event;
  char key[10];
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    event = vpDisplay::getKeyboardEvent(I, &key[0], false);
    if (event) {
      std::cout << "Key detected: " << key << std::endl;
      cpt_event ++;
    }
    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);

#ifdef VISP_HAVE_DISPLAY
  delete d;
#endif
}
\endcode
*/
bool
vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I, bool blocking)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getKeyboardEvent ( blocking ) ;
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

  Get a keyboard event.

  \param I [in] : The displayed image.

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

  Below you will find an example showing how to use this method.
\code
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>

int main()
{
  vpImage<vpRGBa> I(240, 320); // Create a black RGB color image

  vpDisplay *d;

#if defined(VISP_HAVE_X11)
  d = new vpDisplayX;
#elif defined(VISP_HAVE_GTK)
  d = new vpDisplayGTK;
#elif defined(VISP_HAVE_GDI)
  d = new vpDisplayGDI;
#elif defined(VISP_HAVE_D3D9)
  d = new vpDisplayD3D;
#elif defined(VISP_HAVE_OPENCV)
  d = new vpDisplayOpenCV;
#else
  std::cout << "Sorry, no video device is available" << std::endl;
  return -1;
#endif

  // Initialize the display with the image I. Display and image are
  // now link together.
#ifdef VISP_HAVE_DISPLAY
  d->init(I);
#endif

  // Set the display background with image I content
  vpDisplay::display(I);

  // Flush the foreground and background display
  vpDisplay::flush(I);

  // Wait for keyboard event
  std::cout << "Waiting a keyboard event..." << std::endl;
  vpDisplay::getKeyboardEvent(I, true);
  std::cout << "A keyboard event was detected" << std::endl;

  // Non blocking keyboard event loop
  int cpt_event = 0;
  bool event;
  char key[10];
  std::cout << "Enter a non blocking keyboard event detection loop..." << std::endl;
  do {
    event = vpDisplay::getKeyboardEvent(I, &key[0], false);
    if (event) {
      std::cout << "Key detected: " << key << std::endl;
      cpt_event ++;
    }

    vpTime::wait(5); // wait 5 ms
  } while(cpt_event < 5);

  delete d;
}
\endcode
*/
bool
vpDisplay::getKeyboardEvent(const vpImage<vpRGBa> &I,
			    char *string, bool blocking)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getKeyboardEvent ( string, blocking ) ;
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
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.
  
  \return true if a pointer motion event was received, false otherwise.
  
*/
bool 
vpDisplay::getPointerMotionEvent (const vpImage<vpRGBa> &I, vpImagePoint &ip)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getPointerMotionEvent ( ip ) ;
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
  Get the coordinates of the mouse pointer.

  \param I [in] : The displayed image.

  \param ip [out] : The coordinates of the mouse pointer.
  
  \return true.
*/
bool 
vpDisplay::getPointerPosition (const vpImage<vpRGBa> &I, vpImagePoint &ip)
{
  try
  {
    if ( I.display != NULL )
    {
      return ( I.display )->getPointerPosition ( ip ) ;
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
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e use_centered_moments these parameters
  are:
  - the centered moments expressed in pixels: \f$\mu_{20}, \mu_{11}, \mu_{02}\f$;
  - the major and minor axis lenght in pixels and the excentricity of the ellipse in radians: \f$a, b, e\f$.
  \param use_centered_moments : When false, the parameters coef1, coef2, coef3
  are the parameters \f$a, b, e\f$. When true, the parameters coef1, coef2, coef3 are rather the centered moments
  \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ expressed in pixels. In that case, we compute the parameters \e a, \e b and \e e
  from the centered moments.
  \param color : Drawings color.
  \param thickness : Drawings thickness.

  All the points \f$(u_\theta,v_\theta)\f$ on the ellipse are drawn thanks to its parametric representation:

  \f[ \left(\begin{array}{c}
  u_\theta \\
  v_\theta
  \end{array} \right) = \left(\begin{array}{c}
  u_c \\
  v_c
  \end{array} \right) + \left(\begin{array}{cc}
  \cos(e) & -\sin(e) \\
  \sin(e) & \cos(e)
  \end{array} \right) \left(\begin{array}{c}
  a \cos(\theta) \\
  b \sin(\theta)
  \end{array} \right) \f]

  with \f$0 \leq \theta \leq 2\pi\f$.

  The following example shows how to use for example this function to display the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I) ;

    vpDisplay::displayEllipse(I, ellipse.getCenter(), ellipse.get_mu20(), ellipse.get_mu11(), ellipse.get_mu02(),
                              true, vpColor::orange, 1);
    vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<unsigned char> &I,
                               const vpImagePoint &center,
                               const double &coef1, const double &coef2, const double &coef3,
                               bool use_centered_moments,
                               const vpColor &color,
                               unsigned int thickness)
{
  vpDisplay::displayEllipse(I, center, coef1, coef2, coef3, 0., vpMath::rad(360), use_centered_moments, color, thickness);
}

/*!
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e use_centered_moments these parameters
  are:
  - the centered moments expressed in pixels: \f$\mu_{20}, \mu_{11}, \mu_{02}\f$;
  - the major and minor axis lenght in pixels and the excentricity of the ellipse in radians: \f$a, b, e\f$.
  \param theta1, theta2 : Angles \f$(\theta_1, \theta_2)\f$ in radians used to select a portion of the ellipse.
  If theta1=0 and theta2=vpMath::rad(360) all the ellipse is displayed.
  \param use_centered_moments : When false, the parameters coef1, coef2, coef3
  are the parameters \f$a, b, e\f$. When true, the parameters coef1, coef2, coef3 are rather the centered moments
  \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ expressed in pixels. In that case, we compute the parameters \e a, \e b and \e e
  from the centered moments.
  \param color : Drawings color.
  \param thickness : Drawings thickness.

  All the points \f$(u_\theta,v_\theta)\f$ on the ellipse are drawn thanks to its parametric representation:

  \f[ \left(\begin{array}{c}
  u_\theta \\
  v_\theta
  \end{array} \right) = \left(\begin{array}{c}
  u_c \\
  v_c
  \end{array} \right) + \left(\begin{array}{cc}
  \cos(e) & -\sin(e) \\
  \sin(e) & \cos(e)
  \end{array} \right) \left(\begin{array}{c}
  a \cos(\theta) \\
  b \sin(\theta)
  \end{array} \right) \f]

  with \f$\theta_1 \leq \theta \leq \theta_2\f$.

  The following example shows how to use for example this function to display the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I) ;

    vpDisplay::displayEllipse(I, ellipse.getCenter(), ellipse.get_mu20(),
                              ellipse.get_mu11(), ellipse.get_mu02(),
                              ellipse.getSmallestAngle(), ellipse.getHighestAngle(),
                              true, vpColor::orange, 1);
    vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<vpRGBa> &I,
                               const vpImagePoint &center,
                               const double &coef1, const double &coef2, const double &coef3,
                               const double &theta1, const double &theta2, bool use_centered_moments,
                               const vpColor &color,
                               unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      double j1, i1;
      vpImagePoint iP11;
      double j2, i2;
      vpImagePoint iP22;
      j1 = j2 = i1 = i2 = 0 ;
      double a=0., b=0., e=0.;

      double mu20_p = coef1;
      double mu11_p = coef2;
      double mu02_p = coef3;

      if (use_centered_moments) {
        if (std::fabs(mu11_p) > std::numeric_limits<double>::epsilon()) {

          double val_p = sqrt(vpMath::sqr(mu20_p-mu02_p) + 4*vpMath::sqr(mu11_p));
          a = sqrt((mu20_p + mu02_p + val_p)/2);
          b = sqrt((mu20_p + mu02_p - val_p)/2);

          e = (mu02_p - mu20_p + val_p)/(2*mu11_p);
          e = atan(e);
        }
        else {
          a = sqrt(mu20_p);
          b = sqrt(mu02_p);
          e = 0.;
        }
      }
      else {
        a = coef1;
        b = coef2;
        e = coef3;
      }

      // Approximation of the circumference of an ellipse:
      // [Ramanujan, S., "Modular Equations and Approximations to ,"
      // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
      double t = (a-b)/(a+b);
      double circumference = M_PI*(a+b)*(1 + 3*vpMath::sqr(t)/(10 + sqrt(4 - 3*vpMath::sqr(t))));

      int nbpoints = (int)(floor(circumference/5));
      if (nbpoints < 10)
        nbpoints = 10;
      double incr = 2*M_PI / nbpoints ; // angle increment

      double smallalpha = theta1;
      double highalpha  = theta2;
      double ce = cos(e);
      double se = sin(e);

      double k = smallalpha ;
      j1 = a *cos(k) ; // equation of an ellipse
      i1 = b *sin(k) ; // equation of an ellipse

      // (i1,j1) are the coordinates on the origin centered ellipse ;
      // a rotation by "e" and a translation by (xci,jc) are done
      // to get the coordinates of the point on the shifted ellipse
      iP11.set_j ( center.get_j() + ce *j1 - se *i1 );
      iP11.set_i ( center.get_i() + se *j1 + ce *i1 );

      while (k+incr<highalpha+incr)
      {
        j2 = a *cos(k+incr) ; // equation of an ellipse
        i2 = b *sin(k+incr) ; // equation of an ellipse

        // to get the coordinates of the point on the shifted ellipse
        iP22.set_j ( center.get_j() + ce *j2 - se *i2 );
        iP22.set_i ( center.get_i() + se *j2 + ce *i2 );

        ( I.display )->displayLine(iP11, iP22, color, thickness) ;

        i1 = i2;
        j1 = j2;
        iP11 = iP22;

        k += incr ;
      }
    }
  }
  catch ( vpException &e )
  {
    throw(e) ;
  }
}

/*!
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e use_centered_moments these parameters
  are:
  - the centered moments expressed in pixels: \f$\mu_{20}, \mu_{11}, \mu_{02}\f$;
  - the major and minor axis lenght in pixels and the excentricity of the ellipse in radians: \f$a, b, e\f$.
  \param use_centered_moments : When false, the parameters coef1, coef2, coef3
  are the parameters \f$a, b, e\f$. When true, the parameters coef1, coef2, coef3 are rather the centered moments
  \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ expressed in pixels. In that case, we compute the parameters \e a, \e b and \e e
  from the centered moments.
  \param color : Drawings color.
  \param thickness : Drawings thickness.

  All the points \f$(u_\theta,v_\theta)\f$ on the ellipse are drawn thanks to its parametric representation:

  \f[ \left(\begin{array}{c}
  u_\theta \\
  v_\theta
  \end{array} \right) = \left(\begin{array}{c}
  u_c \\
  v_c
  \end{array} \right) + \left(\begin{array}{cc}
  \cos(e) & -\sin(e) \\
  \sin(e) & \cos(e)
  \end{array} \right) \left(\begin{array}{c}
  a \cos(\theta) \\
  b \sin(\theta)
  \end{array} \right) \f]

  with \f$0 \leq \theta \leq 2\pi\f$.

  The following example shows how to use for example this function to display the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I) ;

    vpDisplay::displayEllipse(I, ellipse.getCenter(), ellipse.get_mu20(), ellipse.get_mu11(), ellipse.get_mu02(),
                              true, vpColor::orange, 1);
    vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<vpRGBa> &I,
                               const vpImagePoint &center,
                               const double &coef1, const double &coef2, const double &coef3,
                               bool use_centered_moments,
                               const vpColor &color,
                               unsigned int thickness)
{
  vpDisplay::displayEllipse(I, center, coef1, coef2, coef3, 0., vpMath::rad(360), use_centered_moments, color, thickness);
}

/*!
  Display an ellipse from its parameters expressed in pixels.
  \param I : Image to consider.
  \param center : Center \f$(u_c, v_c)\f$ of the ellipse.
  \param coef1, coef2, coef3 : Depending on the parameter \e use_centered_moments these parameters
  are:
  - the centered moments expressed in pixels: \f$\mu_{20}, \mu_{11}, \mu_{02}\f$;
  - the major and minor axis lenght in pixels and the excentricity of the ellipse in radians: \f$a, b, e\f$.
  \param theta1, theta2 : Angles \f$(\theta_1, \theta_2)\f$ in radians used to select a portion of the ellipse.
  If theta1=0 and theta2=vpMath::rad(360) all the ellipse is displayed.
  \param use_centered_moments : When false, the parameters coef1, coef2, coef3
  are the parameters \f$a, b, e\f$. When true, the parameters coef1, coef2, coef3 are rather the centered moments
  \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ expressed in pixels. In that case, we compute the parameters \e a, \e b and \e e
  from the centered moments.
  \param color : Drawings color.
  \param thickness : Drawings thickness.

  All the points \f$(u_\theta,v_\theta)\f$ on the ellipse are drawn thanks to its parametric representation:

  \f[ \left(\begin{array}{c}
  u_\theta \\
  v_\theta
  \end{array} \right) = \left(\begin{array}{c}
  u_c \\
  v_c
  \end{array} \right) + \left(\begin{array}{cc}
  \cos(e) & -\sin(e) \\
  \sin(e) & \cos(e)
  \end{array} \right) \left(\begin{array}{c}
  a \cos(\theta) \\
  b \sin(\theta)
  \end{array} \right) \f]

  with \f$\theta_1 \leq \theta \leq \theta_2\f$.

  The following example shows how to use for example this function to display the result of a tracking.
  \code
    vpMeEllipse ellipse;
    ...
    vpDisplay::display(I);
    ellipse.track(I) ;

    vpDisplay::displayEllipse(I, ellipse.getCenter(), ellipse.get_mu20(),
                              ellipse.get_mu11(), ellipse.get_mu02(),
                              ellipse.getSmallestAngle(), ellipse.getHighestAngle(),
                              true, vpColor::orange, 1);
    vpDisplay::flush(I);
  \endcode
*/
void vpDisplay::displayEllipse(const vpImage<unsigned char> &I,
                               const vpImagePoint &center,
                               const double &coef1, const double &coef2, const double &coef3,
                               const double &theta1, const double &theta2, bool use_centered_moments,
                               const vpColor &color,
                               unsigned int thickness)
{
  try
  {
    if ( I.display != NULL )
    {
      double j1, i1;
      vpImagePoint iP11;
      double j2, i2;
      vpImagePoint iP22;
      j1 = j2 = i1 = i2 = 0 ;
      double a=0., b=0., e=0.;

      double mu20_p = coef1;
      double mu11_p = coef2;
      double mu02_p = coef3;

      if (use_centered_moments) { 
        if (std::fabs(mu11_p) > std::numeric_limits<double>::epsilon()) {

          double val_p = sqrt(vpMath::sqr(mu20_p-mu02_p) + 4*vpMath::sqr(mu11_p));
          a = sqrt((mu20_p + mu02_p + val_p)/2);
          b = sqrt((mu20_p + mu02_p - val_p)/2);

          e = (mu02_p - mu20_p + val_p)/(2*mu11_p);
          e = atan(e);
        }
        else {
          a = sqrt(mu20_p);
          b = sqrt(mu02_p);
          e = 0.;
        }
      }
      else {
        a = coef1;
        b = coef2;
        e = coef3;
      }

      // Approximation of the circumference of an ellipse:
      // [Ramanujan, S., "Modular Equations and Approximations to ,"
      // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
      double t = (a-b)/(a+b);
      double circumference = M_PI*(a+b)*(1 + 3*vpMath::sqr(t)/(10 + sqrt(4 - 3*vpMath::sqr(t))));

      int nbpoints = (int)(floor(circumference/5));
      if (nbpoints < 10)
        nbpoints = 10;
      double incr = 2*M_PI / nbpoints ; // angle increment

      double smallalpha = theta1;
      double highalpha  = theta2;
      double ce = cos(e);
      double se = sin(e);

      double k = smallalpha ;
      j1 = a *cos(k) ; // equation of an ellipse
      i1 = b *sin(k) ; // equation of an ellipse

      // (i1,j1) are the coordinates on the origin centered ellipse ;
      // a rotation by "e" and a translation by (xci,jc) are done
      // to get the coordinates of the point on the shifted ellipse
      iP11.set_j ( center.get_j() + ce *j1 - se *i1 );
      iP11.set_i ( center.get_i() + se *j1 + ce *i1 );

      while (k+incr<highalpha+incr)
      {
        j2 = a *cos(k+incr) ; // equation of an ellipse
        i2 = b *sin(k+incr) ; // equation of an ellipse

        // to get the coordinates of the point on the shifted ellipse
        iP22.set_j ( center.get_j() + ce *j2 - se *i2 );
        iP22.set_i ( center.get_i() + se *j2 + ce *i2 );

        ( I.display )->displayLine(iP11, iP22, color, thickness) ;

        i1 = i2;
        j1 = j2;
        iP11 = iP22;

        k += incr ;
      }
    }
  }
  catch ( vpException &e )
  {
    throw(e) ;
  }
}

