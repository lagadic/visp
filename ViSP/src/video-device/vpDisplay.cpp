#include <visp/vpDisplay.h>
#include <visp/vpDisplayException.h>

#include <visp/vpPoint.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpMath.h>


/*!
  \file vpDisplay.cpp
  \brief  generic class for image display

*/

vpDisplay::vpDisplay()
{
  title = NULL ;
}

/*!
  Display the windows title.
*/
void
vpDisplay::displayTitle(vpImage<unsigned char> &I, const char *windowtitle)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->flushTitle(windowtitle) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  Display a 8bits image in the display window
*/
void
vpDisplay::display(vpImage<unsigned char> &I)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayImage(I) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  \brief get the window pixmap and put it in vpRGBa image
*/
void
vpDisplay::getImage(vpImage<unsigned  char> &Isrc, vpImage<vpRGBa> &Idest)
{

  try
  {
    if (Isrc.display != NULL)
    {
      (Isrc.display)->getImage(Idest) ;
    }
    else
    {
      ERROR_TRACE("Display not initialized") ;
      throw(vpDisplayException(vpDisplayException::notInitializedError,
			       "Display not initialized")) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  Display a point at coordinates (i,j) in the display window
*/
void vpDisplay::displayPoint(vpImage<unsigned char> &I,
			     int i,int j,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayPoint(i,j,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (i,j) in the display window
*/
void vpDisplay::displayCross(vpImage<unsigned char> &I,
		  int i,int j,
		  int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCross(i,j,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (i,j) in the display window
*/
void
vpDisplay::displayCrossLarge(vpImage<unsigned char> &I,
			     int i,int j,
			     int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCrossLarge(i,j,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (i,j) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle(vpImage<unsigned char> &I,
			 int i, int j, int r, int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCircle(i,j,r,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
*/
void vpDisplay::displayLine(vpImage<unsigned char> &I,
			    int i1, int j1, int i2, int j2,
			    int col, int e)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayLine(i1,j1,i2,j2,col,e) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!  Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine(vpImage<unsigned char> &I,
		    int i1, int j1, int i2, int j2,
		    int col, int e2)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayDotLine(i1,j1,i2,j2,col,e2) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

void
vpDisplay::displayFrame(vpImage<unsigned char> &I,
			vpHomogeneousMatrix &cMo,
			vpCameraParameters &cam,
			double size, int col)
{
 // used by display
  vpPoint o; o.setWorldCoordinates(0.0,0.0,0.0) ;
  vpPoint x; x.setWorldCoordinates(size,0.0,0.0) ;
  vpPoint y; y.setWorldCoordinates(0.0,size,0.0) ;
  vpPoint z; z.setWorldCoordinates(0.0,0.0,size) ;

  o.track(cMo) ;
  x.track(cMo) ;
  y.track(cMo) ;
  z.track(cMo) ;

  double ox,oy, x1,y1 ;

  if (col == vpColor::none)
    {
      vpMeterPixelConversion::convertPoint(cam,o.p[0],o.p[1],ox,oy) ;

      vpMeterPixelConversion::convertPoint(cam,x.p[0],x.p[1],x1,y1) ;
      vpDisplay::displayArrow(I,
			      vpMath::round(oy), vpMath::round(ox),
			      vpMath::round(y1), vpMath::round(x1),
			      vpColor::green) ;

      vpMeterPixelConversion::convertPoint(cam,y.p[0],y.p[1],x1,y1) ;
      vpDisplay::displayArrow(I,
			      vpMath::round(oy), vpMath::round(ox),
			      vpMath::round(y1), vpMath::round(x1),
			      vpColor::blue) ;

      vpMeterPixelConversion::convertPoint(cam,z.p[0],z.p[1],x1,y1) ;
      vpDisplay::displayArrow(I,
			      vpMath::round(oy), vpMath::round(ox),
			      vpMath::round(y1), vpMath::round(x1),
			      vpColor::red) ;
    }
  else
     {
      vpMeterPixelConversion::convertPoint(cam,o.p[0],o.p[1],ox,oy) ;

      vpMeterPixelConversion::convertPoint(cam,x.p[0],x.p[1],x1,y1) ;
      vpDisplay::displayArrow(I,
			      vpMath::round(oy), vpMath::round(ox),
			      vpMath::round(y1), vpMath::round(x1),
			      col) ;

      vpMeterPixelConversion::convertPoint(cam,y.p[0],y.p[1],x1,y1) ;
      vpDisplay::displayArrow(I,
			      vpMath::round(oy), vpMath::round(ox),
			      vpMath::round(y1), vpMath::round(x1),
			      col) ;

      vpMeterPixelConversion::convertPoint(cam,z.p[0],z.p[1],x1,y1) ;
      vpDisplay::displayArrow(I,
			      vpMath::round(oy), vpMath::round(ox),
			      vpMath::round(y1), vpMath::round(x1),
			      col) ;
    }
}


/*! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  window
*/
void
vpDisplay::displayArrow(vpImage<unsigned char> &I,
			int i1,int j1, int i2, int j2,
			int col, int L,int l)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayArrow(i1,j1,i2,j2,col,L,l) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!

  Display a rectangle in the display window.  The rectangle upper left corner
  has coordinates (i,j). The size of the rectangle is fixed by \e width and \e
  height.

  \param i Row number of the rectangle upper corner
  \param j Column number of the rectangle upper corner
  \param width Width of the rectangle.
  \param height Height of the rectangle.
  \param col Color of the rectangle.

*/
void
vpDisplay::displayRectangle(vpImage<unsigned char> &I, int i, int j,
			    int width, int height, int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayRectangle(i,j,width,height,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*! display a string at coordinates (i,j) to (i2,j2) in the display
  window
*/
void
vpDisplay::displayCharString(vpImage<unsigned char> &I,
			     int i,int j,char *s, int c)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCharString(i,j,s,c) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
   flushes the output buffer and then waits until all
  requests have been received and processed by the server
*/
void vpDisplay::flush(vpImage<unsigned char> &I)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->flushDisplay() ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
   Close the display attached to I.
*/
void vpDisplay::close(vpImage<unsigned char> &I)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->closeDisplay() ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  return true way a button is pressed
 */
bool  vpDisplay::getClick(vpImage<unsigned char> &I,
	       int& i, int& j)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(i,j) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
 */
bool  vpDisplay::getClick(vpImage<unsigned char> &I,
	       int& i, int& j, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(i,j,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}

/*!
  wait for a click
 */
void  vpDisplay::getClick(vpImage<unsigned char> &I)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->getClick() ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  return true way  button is released
 */
bool
vpDisplay::getClickUp(vpImage<unsigned char> &I,
		      int& i, int& j, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClickUp(i,j,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}


/*!
  Display the windows title.
*/
void
vpDisplay::displayTitle(vpImage<vpRGBa> &I, const char *windowtitle)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->flushTitle(windowtitle) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  Display a 32bits image in the display window
*/
void
vpDisplay::display(vpImage<vpRGBa> &I)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayImage(I) ;
    }
    else
    {
      ERROR_TRACE("Display not initialized") ;
      throw(vpDisplayException(vpDisplayException::notInitializedError,
			       "Display not initialized")) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  \brief get the window pixmap and put it in vpRGBa image
*/
void
vpDisplay::getImage(vpImage<vpRGBa> &Isrc, vpImage<vpRGBa> &Idest)
{

  try
  {
    if (Isrc.display != NULL)
    {
      (Isrc.display)->getImage(Idest) ;
    }
    else
    {
      ERROR_TRACE("Display not initialized") ;
      throw(vpDisplayException(vpDisplayException::notInitializedError,
			       "Display not initialized")) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  Display a point at coordinates (i,j) in the display window
*/


void vpDisplay::displayPoint(vpImage<vpRGBa> &I,
			     int i,int j,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayPoint(i,j,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (i,j) in the display window
*/
void vpDisplay::displayCross(vpImage<vpRGBa> &I,
		  int i,int j,
		  int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCross(i,j,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (i,j) in the display window
*/
void
vpDisplay::displayCrossLarge(vpImage<vpRGBa> &I,
			     int i,int j,
			     int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCrossLarge(i,j,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (i,j) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle(vpImage<vpRGBa> &I,
			 int i, int j, int r, int col)
{
  try
  {
    if (I.display != NULL)
    {(I.display)->displayCircle(i,j,r,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
*/
void vpDisplay::displayLine(vpImage<vpRGBa> &I,
		 int i1, int j1, int i2, int j2,
		 int col, int e)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayLine(i1,j1,i2,j2,col,e) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!  Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine(vpImage<vpRGBa> &I,
		    int i1, int j1, int i2, int j2,
		    int col, int e2)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayDotLine(i1,j1,i2,j2,col,e2) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
  window
*/
void
vpDisplay::displayArrow(vpImage<vpRGBa> &I,
			int i1,int j1, int i2, int j2,
			int col, int L,int l)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayArrow(i1,j1,i2,j2,col,L,l) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*! display a string at coordinates (i,j) to (i2,j2) in the display
  window
*/
void
vpDisplay::displayCharString(vpImage<vpRGBa> &I,
			     int i,int j,char *s, int c)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCharString(i,j,s,c) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
   flushes the output buffer and then waits until all
  requests have been received and processed by the server
*/
void vpDisplay::flush(vpImage<vpRGBa> &I)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->flushDisplay() ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
   Close the display attached to I.
*/
void vpDisplay::close(vpImage<vpRGBa> &I)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->closeDisplay() ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  return true way a button is pressed
 */
bool  vpDisplay::getClick(vpImage<vpRGBa> &I,
	       int& i, int& j)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(i,j) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
 */
bool  vpDisplay::getClick(vpImage<vpRGBa> &I,
	       int& i, int& j, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(i,j,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}

/*!
  wait for a click
 */
void  vpDisplay::getClick(vpImage<vpRGBa> &I)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->getClick() ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}



/*!
  return true way  button is released
 */
bool
vpDisplay::getClickUp(vpImage<vpRGBa> &I,
		      int& i, int& j, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClickUp(i,j,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}


//--------------
// uv

/*!
  Display a point at coordinates (u,v) in the display window
*/
void vpDisplay::displayPoint_uv(vpImage<unsigned char> &I,
			     int u,int v,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayPoint(v,u,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (u,v) in the display window
*/
void vpDisplay::displayCross_uv(vpImage<unsigned char> &I,
		  int u,int v,
		  int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCross(v,u,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (u,v) in the display window
*/
void
vpDisplay::displayCrossLarge_uv(vpImage<unsigned char> &I,
			     int u,int v,
			     int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCrossLarge(v,u,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (u,v) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle_uv(vpImage<unsigned char> &I,
			 int u, int v, int r, int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCircle(v,u,r,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a line from coordinates (u1,v1) to (u2,v2) in the display window.
*/
void vpDisplay::displayLine_uv(vpImage<unsigned char> &I,
			    int u1, int v1, int u2, int v2,
			    int col, int e)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayLine(v1,u1,v2,u2,col,e) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!  Display a dotted line from coordinates (u1,v1) to (u2,v2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine_uv(vpImage<unsigned char> &I,
		    int u1, int v1, int u2, int v2,
		    int col, int e2)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayDotLine(v1,u1,v2,u2,col,e2) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*! Display an arrow from coordinates (u1,v1) to (u2,v2) in the display
  window
*/
void
vpDisplay::displayArrow_uv(vpImage<unsigned char> &I,
			int u1,int v1, int u2, int v2,
			int col, int L,int l)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayArrow(v1,u1,v2,u2,col,L,l) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!

  Display a rectangle in the display window.  The rectangle upper left corner
  has coordinates (u,v). The size of the rectangle is fixed by \e width and \e
  height.

  \param i Row number of the rectangle upper corner
  \param j Column number of the rectangle upper corner
  \param width Width of the rectangle.
  \param height Height of the rectangle.
  \param col Color of the rectangle.

*/
void
vpDisplay::displayRectangle_uv(vpImage<unsigned char> &I, int u, int v,
			    int width, int height, int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayRectangle(v,u,width,height,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*! display a string at coordinates (u,v) in the display
  window
*/
void
vpDisplay::displayCharString_uv(vpImage<unsigned char> &I,
			     int u,int v,char *s, int c)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCharString(v,u,s,c) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}


/*!
  return true way a button is pressed
 */
bool  vpDisplay::getClick_uv(vpImage<unsigned char> &I,
	       int& u, int& v)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(v,u) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
 */
bool  vpDisplay::getClick_uv(vpImage<unsigned char> &I,
	       int& u, int& v, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(v,u,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}


/*!
  return true way  button is released
 */
bool
vpDisplay::getClickUp_uv(vpImage<unsigned char> &I,
		      int& u, int& v, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClickUp(v,u,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}


/*!
  Display a point at coordinates (u,v) in the display window
*/


void vpDisplay::displayPoint_uv(vpImage<vpRGBa> &I,
			     int u,int v,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayPoint(v,u,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }

}
/*!
  Display a cross at coordinates (u,v) in the display window
*/
void vpDisplay::displayCross_uv(vpImage<vpRGBa> &I,
		  int u,int v,
		  int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCross(v,u,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a large cross at coordinates (u,v) in the display window
*/
void
vpDisplay::displayCrossLarge_uv(vpImage<vpRGBa> &I,
			     int u,int v,
			     int size,int col)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCrossLarge(v,u,size,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!
  Display a circle at coordinates (u,v) in the display window.
  circle radius is given in pixel by paramter r
*/
void
vpDisplay::displayCircle_uv(vpImage<vpRGBa> &I,
			 int u, int v, int r, int col)
{
  try
  {
    if (I.display != NULL)
    {(I.display)->displayCircle(v,u,r,col) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}
/*!
  Display a line from coordinates (u1,v1) to (u2,v2) in the display window.
*/
void vpDisplay::displayLine_uv(vpImage<vpRGBa> &I,
		 int u1, int v1, int u2, int v2,
		 int col, int e)
{

  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayLine(v1,u1,v2,u2,col,e) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*!  Display a dotted line from coordinates  (u1,v1) to (u2,v2) in the display
  window.  circle radius is given in pixel by paramter r
*/
void vpDisplay::displayDotLine_uv(vpImage<vpRGBa> &I,
		    int u1, int v1, int u2, int v2,
		    int col, int e2)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayDotLine(v1,u1,v2,u2,col,e2) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*! Display an arrow from coordinates  (u1,v1) to (u2,v2) in the display
  window
*/
void
vpDisplay::displayArrow_uv(vpImage<vpRGBa> &I,
			int u1,int v1, int u2, int v2,
			int col, int L,int l)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayArrow(v1,u1,v2,u2,col,L,l) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}

/*! display a string at coordinates (u,v) to (i2,j2) in the display
  window
*/
void
vpDisplay::displayCharString_uv(vpImage<vpRGBa> &I,
			     int u,int v,char *s, int c)
{
  try
  {
    if (I.display != NULL)
    {
      (I.display)->displayCharString(v,u,s,c) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
}



/*!
  return true way a button is pressed
 */
bool  vpDisplay::getClick_uv(vpImage<vpRGBa> &I,
	       int& u, int& v)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(v,u) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}

/*!
  return true way button is pressed
 */
bool  vpDisplay::getClick_uv(vpImage<vpRGBa> &I,
	       int& u, int& v, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClick(v,u,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}


/*!
  return true way  button is released
 */
bool
vpDisplay::getClickUp_uv(vpImage<vpRGBa> &I,
		      int& u, int& v, int& button)
{
  try
  {
    if (I.display != NULL)
    {
      return (I.display)->getClickUp(v,u,button) ;
    }
  }
  catch(...)
  {
    ERROR_TRACE(" ") ;
    throw ;
  }
  return false ;
}
