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
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp/vpDisplay.h>
#include <visp/vpDisplayException.h>

#include <visp/vpPoint.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpMath.h>


/*!
  \file vpDisplay.cpp
  \brief Generic class for image display.
*/

/*!
  Default constructor.
*/
vpDisplay::vpDisplay()
{
  title = NULL ;
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
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayOpenCV.h>

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
  d->init(I);

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
  vpImageIo::writePPM(Ioverlay, ofilename) ;

  // Wait for a click in the display window
  vpDisplay::getClick(I);

  delete d;
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
  
*/
void
vpDisplay::displayFrame ( const vpImage<unsigned char> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, 
			  const vpColor &color, unsigned int thickness)
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

  vpImagePoint ipo, ip1;

  if ( color == vpColor::none )
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, vpColor::red, 4*thickness, 2*thickness, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, vpColor::green, 4*thickness, 2*thickness, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I,ipo, ip1, vpColor::blue, 4*thickness, 2*thickness, thickness) ;
  }
  else
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, color, 4*thickness, 2*thickness, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, color, 4*thickness, 2*thickness, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I,ipo, ip1, color, 4*thickness, 2*thickness, thickness) ;

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
  
*/
void
vpDisplay::displayFrame ( const vpImage<vpRGBa> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, const vpColor &color,
                          unsigned int thickness )
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

  vpImagePoint ipo, ip1;
  if ( color == vpColor::none )
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, vpColor::red, 4, 2, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, vpColor::green, 4, 2, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I,ipo, ip1, vpColor::blue, 4, 2, thickness) ;
  }
  else
  {
    vpMeterPixelConversion::convertPoint ( cam, o.p[0], o.p[1], ipo) ;

    vpMeterPixelConversion::convertPoint ( cam, x.p[0], x.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, color, 4*thickness, 2*thickness, thickness) ;

    vpMeterPixelConversion::convertPoint ( cam, y.p[0], y.p[1], ip1) ;
    vpDisplay::displayArrow ( I, ipo, ip1, color, 4*thickness, 2*thickness, thickness) ;
    
    vpMeterPixelConversion::convertPoint ( cam, z.p[0], z.p[1], ip1) ;
    vpDisplay::displayArrow ( I,ipo, ip1, color, 4*thickness, 2*thickness, thickness) ;

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
  
*/
void
vpDisplay::displayCamera ( const vpImage<unsigned char> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, 
			  const vpColor &color)
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
    vpDisplay::displayLine ( I, ip_1, ip, color, 1);
    vpDisplay::displayLine ( I, ip0, ip_1, color, 1);
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
  
*/
void
vpDisplay::displayCamera( const vpImage<vpRGBa> &I,
                          const vpHomogeneousMatrix &cMo,
                          const vpCameraParameters &cam,
                          double size, const vpColor &color)
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
    vpDisplay::displayLine ( I, ip_1, ip, color, 1);
    vpDisplay::displayLine ( I, ip0, ip_1, color, 1);
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
    
  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont()

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
    
  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param ip : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont()

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
    
  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont()

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
    
  To select the font used to display the string, use setFont().
    
  \param I : Image associated to the display.
  \param i,j : Upper left image point location of the string in the display.
  \param string : String to display in overlay.
  \param color : String color.
  
  \sa setFont()

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
*/
void vpDisplay::displayPoint ( const vpImage<unsigned char> &I,
                               const vpImagePoint &ip,
			       const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayPoint ( ip, color ) ;
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
*/
void vpDisplay::displayPoint ( const vpImage<vpRGBa> &I,
                               const vpImagePoint &ip,
			       const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      ( I.display )->displayPoint ( ip, color ) ;
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
*/
void vpDisplay::displayPoint ( const vpImage<unsigned char> &I,
                               int i, int j,
                               const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayPoint ( ip, color ) ;
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
*/
void vpDisplay::displayPoint ( const vpImage<vpRGBa> &I,
                               int i, int j,
                               const vpColor &color )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayPoint ( ip, color ) ;
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
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpColor.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>

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
#include <visp/vpConfig.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayOpenCV.h>

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
  d->init(I);

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
  vpImageIo::writePPM(Ioverlay, ofilename) ;

  // Wait for a click in the display window
  vpDisplay::getClick(I);

  delete d;
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
  Flushes the output buffer associated to image \e I display.
  It's necessary to use this function to see the results of any drawing.

  \warning This function is particular and must be called
  to show the overlay. Because it's time spending, use it parcimoniously.

  \code
#include <visp/vpDisplay.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpColor.h>
#include <visp/vpImage.h>
#include <visp/vpImagePoint.h>
#include <visp/vpRGBa.h>

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
#include <visp/vpConfig.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>

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
  d->init(I);

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
#include <visp/vpConfig.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>

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
  d->init(I);

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
#include <visp/vpConfig.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>

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
  d->init(I);

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
#include <visp/vpConfig.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayD3D.h>

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
  d->init(I);

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

/****************************************************************

           Deprecated functions

*****************************************************************/

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  Display the windows title.
  \deprecated Use vpDisplay::setTitle() instead.
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
  \deprecated Use vpDisplay::setTitle() instead.
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

  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<unsigned char> &, const vpImagePoint &, bool) instead.

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

  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<unsigned char> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<vpRGBa> &, const vpImagePoint &, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<vpRGBa> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClickUp(const
  vpImage<unsigned char> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClickUp(const
  vpImage<vpRGBa> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<unsigned char> &, const vpImagePoint &, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<unsigned char> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<vpRGBa> &, const vpImagePoint &, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClick(const
  vpImage<vpRGBa> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClickUp(const
  vpImage<unsigned char> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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
  \deprecated This method is deprecated. Use vpDisplay::getClickUp(const
  vpImage<vpRGBa> &, const vpImagePoint &,
  vpMouseButton::vpMouseButtonType&, bool) instead.

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



/*!

  \deprecated This method is deprecated. You should use
  vpDisplay::displayArrow(const vpImage<unsigned char> &, const vpImagePoint &,
  const vpImagePoint &, vpColor, unsigned int, unsigned
  int, unsigned int)

  Display an arrow from coordinates (u1,v1) to (u2,v2) in the display
  window
*/
void
vpDisplay::displayArrow_uv ( const vpImage<unsigned char> &I,
                             int u1, int v1, int u2, int v2,
                             vpColor col,
                             unsigned int L,unsigned int l )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_u(u1);
      ip1.set_v(v1);
      ip2.set_u(u2);
      ip2.set_v(v2);
      ( I.display )->displayArrow ( ip1, ip2, col, L, l ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayArrow(const vpImage<vpRGBa> &, const vpImagePoint &,
  const vpImagePoint &, vpColor, unsigned int, unsigned
  int, unsigned int)
 
  Display an arrow from coordinates  (u1,v1) to (u2,v2) in the display
  window
*/
void
vpDisplay::displayArrow_uv ( const vpImage<vpRGBa> &I,
                             int u1, int v1, int u2, int v2,
                             vpColor col,
                             unsigned int L,unsigned int l )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_u(u1);
      ip1.set_v(v1);
      ip2.set_u(u2);
      ip2.set_v(v2);
      ( I.display )->displayArrow ( ip1, ip2,col,L,l ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}



/*!

  \deprecated This method is deprecated. You should use
  vpDisplay::displayCharString(const vpImage<unsigned char> &, const
  vpImagePoint &, const char *, vpColor) instead.

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
                                  vpColor c )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCharString ( ip, s, c ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  \deprecated This method is deprecated. You should use
  vpDisplay::displayCharString(const vpImage<vpRGBa> &, const vpImagePoint
  &, const char *, vpColor) instead.

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
                                  vpColor c )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCharString ( ip, s, c ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!  
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCircle(const vpImage<unsigned char> &, const
  vpImagePoint &, unsigned int, vpColor, bool, unsigned
  int) instead.

  Display a circle at coordinates (u,v) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle_uv ( const vpImage<unsigned char> &I,
                              int u, int v, unsigned int r,
                              vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCircle ( ip, r, col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCircle(const vpImage<vpRGBa> &, const vpImagePoint &,
  unsigned int, vpColor, bool, unsigned int) instead.

  Display a circle at coordinates (u,v) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle_uv ( const vpImage<vpRGBa> &I,
                              int u, int v, unsigned int r,
                              vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCircle ( ip, r, col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCross(const vpImage<unsigned char> &, const vpImagePoint &,
  unsigned int, vpColor, unsigned int) instead.

  Display a large cross at coordinates (i,j) in the display window
*/
void
vpDisplay::displayCrossLarge ( const vpImage<unsigned char> &I,
                               int i, int j,
                               unsigned int size,vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayCross ( ip, size, col, 3 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCross(const vpImage<vpRGBa> &, const vpImagePoint &,
  unsigned int, vpColor, unsigned int) instead.

  Display a large cross at coordinates (i,j) in the display window
*/
void
vpDisplay::displayCrossLarge ( const vpImage<vpRGBa> &I,
                               int i, int j,
                               unsigned int size,vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_i( i );
      ip.set_j( j );
      ( I.display )->displayCross ( ip, size, col, 3 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCross(const vpImage<unsigned char> &, const vpImagePoint &,
  unsigned int, vpColor, unsigned int) instead.

  Display a cross at coordinates (u,v) in the display window
*/
void vpDisplay::displayCross_uv ( const vpImage<unsigned char> &I,
                                  int u, int v,
                                  unsigned int size,vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCross ( ip, size, col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCross(const vpImage<unsigned char> &, const vpImagePoint &,
  unsigned int, vpColor, unsigned int) instead.

  Display a large cross at coordinates (u,v) in the display window
*/
void
vpDisplay::displayCrossLarge_uv ( const vpImage<unsigned char> &I,
                                  int u, int v,
                                  unsigned int size,vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCross ( ip, size, col, 3 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCross(const vpImage<vpRGBa> &, const vpImagePoint &,
  unsigned int, vpColor, unsigned int) instead.

  Display a cross at coordinates (u,v) in the display window
*/
void vpDisplay::displayCross_uv ( const vpImage<vpRGBa> &I,
                                  int u, int v,
                                  unsigned int size,vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCross ( ip, size, col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}
/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayCross(const vpImage<vpRGBa> &, const vpImagePoint &,
  unsigned int, vpColor, unsigned int) instead.

  Display a large cross at coordinates (u,v) in the display window
*/
void
vpDisplay::displayCrossLarge_uv ( const vpImage<vpRGBa> &I,
                                  int u, int v,
                                  unsigned int size,vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayCross ( ip, size, col, 3 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}



/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayDotLine(const vpImage<unsigned char> &, const
  vpImagePoint &, const vpImagePoint &, vpColor, unsigned
  int)

  Display a dotted line from coordinates (u1,v1) to (u2,v2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine_uv ( const vpImage<unsigned char> &I,
                                    int u1, int v1, int u2, int v2,
                                    vpColor col, unsigned int e2 )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_u( u1 );
      ip1.set_v( v1 );
      ip2.set_u( u2 );
      ip2.set_v( v2 );
      ( I.display )->displayDotLine ( ip1, ip2, col, e2 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!

  \deprecated This method is deprecated. You should use
  vpDisplay::displayDotLine(const vpImage<vpRGBa> &, const
  vpImagePoint &, const vpImagePoint &, vpColor, unsigned
  int)

  Display a dotted line from coordinates  (u1,v1) to (u2,v2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine_uv ( const vpImage<vpRGBa> &I,
                                    int u1, int v1, int u2, int v2,
                                    vpColor col, unsigned int e2 )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_u( u1 );
      ip1.set_v( v1 );
      ip2.set_u( u2 );
      ip2.set_v( v2 );
      ( I.display )->displayDotLine ( ip1, ip2, col, e2 ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayLine(const vpImage<unsigned char> &, const
  vpImagePoint &, const vpImagePoint &, vpColor, unsigned
  int)

  Display a line from coordinates (u1,v1) to (u2,v2) in the display window.
*/
void vpDisplay::displayLine_uv ( const vpImage<unsigned char> &I,
                                 int u1, int v1,
                                 int u2, int v2,
                                 vpColor col, unsigned int e )
{

  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_u( u1 );
      ip1.set_v( v1 );
      ip2.set_u( u2 );
      ip2.set_v( v2 );
      ( I.display )->displayLine ( ip1, ip2, col, e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::displayLine(const vpImage<vpRGBa> &, const
  vpImagePoint &, const vpImagePoint &, vpColor, unsigned
  int)

  Display a line from coordinates (u1,v1) to (u2,v2) in the display window.
*/
void vpDisplay::displayLine_uv ( const vpImage<vpRGBa> &I,
                                 int u1, int v1, int u2, int v2,
                                 vpColor col, unsigned int e )
{

  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip1, ip2;
      ip1.set_u( u1 );
      ip1.set_v( v1 );
      ip2.set_u( u2 );
      ip2.set_v( v2 );
      ( I.display )->displayLine ( ip1, ip2, col, e ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}


/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::vpImage<unsigned char> &, const vpImagePoint &, vpColor)

  Display a point at coordinates (u,v) in the display window
*/
void vpDisplay::displayPoint_uv ( const vpImage<unsigned char> &I,
                                  int u, int v,
                                  vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayPoint ( ip, col ) ;

    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}

/*!
  \deprecated This method is deprecated. You should use
  vpDisplay::vpImage<vpRGBa> &, const vpImagePoint &, vpColor)

  Display a point at coordinates (u,v) in the display window
*/
void vpDisplay::displayPoint_uv ( const vpImage<vpRGBa> &I,
                                  int u, int v,
                                  vpColor col )
{
  try
  {
    if ( I.display != NULL )
    {
      vpImagePoint ip;
      ip.set_u( u );
      ip.set_v( v );
      ( I.display )->displayPoint ( ip, col ) ;
    }
  }
  catch ( ... )
  {
    vpERROR_TRACE ( "Error caught" ) ;
    throw ;
  }
}




/*!

  \deprecated This method is deprecated. You should use
  vpDisplay::const vpImage<unsigned char> &, const vpImagePoint &,
  float, unsigned int, unsigned int, vpColor, unsigned) instead.

  Display a rectangle in the display window.  The rectangle is defined
  by its center, its orientation (angle) and its size. The rectangle
  center has coordinates (u, v) in the display window. The size of the
  rectangle is fixed by \e width and \e height.
  
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
			       vpColor col,unsigned int e)
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

  \deprecated This method is deprecated. You should use
  vpDisplay::const vpImage<vpRGBa> &, const vpImagePoint &,
  float, unsigned int, unsigned int, vpColor, unsigned) instead.

  Display a rectangle in the display window.  The rectangle is defined
  by its center, its orientation (angle) and its size. The rectangle
  center has coordinates (u, v) in the display window. The size of the
  rectangle is fixed by \e width and \e height.
  
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
			       vpColor col,unsigned int e)
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


#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
