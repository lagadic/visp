/****************************************************************************
 *
 * $Id$
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
 * This file is part of the ViSP toolkit
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
 * Windows 32 display base class
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#if ( defined(WIN32) )

#include <visp/vpDisplayWin32.h>
#include <visp/vpDisplayException.h>
#include <string>



const int vpDisplayWin32::MAX_INIT_DELAY  = 5000;

/*!
  Thread entry point.
  Used as a detour to initWindow.
*/
void vpCreateWindow(threadParam * param)
{
  char* title = param->title;
  (param->vpDisp)->window.initWindow(title, param->x, param->y,
				     param->w, param->h);
  delete param;
}

/*!
  Constructor.
*/
vpDisplayWin32::vpDisplayWin32(vpWin32Renderer * rend) :
  iStatus(false), window(rend)
{
}


/*!
  Destructor.
*/
vpDisplayWin32::~vpDisplayWin32()
{
  closeDisplay();
}


/*!

  Constructor. Initialize a display to visualize a gray level image
  (8 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void vpDisplayWin32::init(vpImage<unsigned char> &I,
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

  window.renderer->setImg(I);

  init (I.getWidth(), I.getHeight(), x, y, title) ;
  I.display = this ;
}


/*!
  Constructor. Initialize a display to visualize a RGBa level image
  (32 bits).

  \param I : Image to be displayed (not that image has to be initialized)
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.
*/
void vpDisplayWin32::init(vpImage<vpRGBa> &I,
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

  window.renderer->setImg(I);

  init (I.getWidth(), I.getHeight(), x, y, title) ;
  I.display = this ;
}


/*!
  Initialize the display size, position and title.

  \param width, height : Width and height of the window.
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void vpDisplayWin32::init(unsigned int width, unsigned int height,
			  int x, int y,
			  const char *title)
{


  if (this->title != NULL)//delete apr�s init du thread.... ou destructeur
    {
      delete [] this->title;
      this->title = NULL;
    }

  if (title != NULL) {
    this->title = new char[strlen(title) + 1] ;
    strcpy(this->title, title) ;
  }

  //we prepare the window's thread creation
  threadParam * param = new threadParam;
  param->x = x;
  param->y = y;
  param->w = width;
  param->h = height;
  param->vpDisp = this;
  param->title = this->title;

  //creates the window in a separate thread
  hThread = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)vpCreateWindow,
			 param,0,&threadId);

  //the initialization worked
  iStatus = (hThread != (HANDLE)NULL);

  displayHasBeenInitialized = true;
}


/*!
  If the window is not initialized yet, wait a little (MAX_INIT_DELAY).
  \exception notInitializedError : the window isn't initialized
*/
void vpDisplayWin32::waitForInit()
{
  //if the window is not initialized yet
  if(!window.isInitialized())
    {
      //wait
      if( WAIT_OBJECT_0 != WaitForSingleObject(window.semaInit,MAX_INIT_DELAY))
	throw(vpDisplayException(vpDisplayException::notInitializedError,
				 "Window not initialized")) ;
      //problem : the window is not initialized
    }
}


/*!
  Display the color image \e I in RGBa format (32bits).

  \warning Display has to be initialized.

  \warning suppres the overlay drawing

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayWin32::displayImage(const vpImage<vpRGBa> &I)
{
  //waits if the window is not initialized
  waitForInit();

  //sets the image to render
  window.renderer->setImg(I);
  //sends a message to the window
  //PostMessage(window.getHWnd(),vpWM_DISPLAY,0,0);
}


/*!
  Display the gray level image \e I (8bits).

  \warning Display has to be initialized.

  \warning suppres the overlay drawing

  \param I : Image to display.

  \sa init(), closeDisplay()
*/
void vpDisplayWin32::displayImage(const vpImage<unsigned char> &I)
{
  //wait if the window is not initialized
  waitForInit();

  //sets the image to render
  window.renderer->setImg(I);
  //sends a message to the window
  //PostMessage(window.getHWnd(), vpWM_DISPLAY, 0,0);
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
bool vpDisplayWin32::getClick( bool blocking)
{
  //wait if the window is not initialized
  waitForInit();
  bool ret = false;
  //sends a message to the window
//   PostMessage(window.getHWnd(), vpWM_GETCLICK, 0,0);

  //waits for a button to be pressed
  if(blocking){ 
    WaitForSingleObject(window.semaClick, NULL);
    WaitForSingleObject(window.semaClickUp, NULL); //to erase previous events 
    WaitForSingleObject(window.semaClick, INFINITE);
    ret = true;
  }
  else
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClick, NULL));
  
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
bool vpDisplayWin32::getClick(vpImagePoint &ip, bool blocking)
{
  //wait if the window is not initialized
  waitForInit();

  bool ret = false ;
  double u, v;
  //tells the window there has been a getclick demand
//   PostMessage(window.getHWnd(), vpWM_GETCLICK, 0,0);
  //waits for a click
  if(blocking){
    WaitForSingleObject(window.semaClick, NULL);
    WaitForSingleObject(window.semaClickUp, NULL);//to erase previous events
    WaitForSingleObject(window.semaClick, INFINITE);
    ret = true;  
  }  
  else
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClick, NULL));
  
  u = window.clickX;
  v = window.clickY;
  ip.set_u( u );
  ip.set_v( v );

  return ret;
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
bool vpDisplayWin32::getClick(vpImagePoint &ip,
                              vpMouseButton::vpMouseButtonType& button,
                              bool blocking)
{
  //wait if the window is not initialized
  waitForInit();
  bool ret = false;
  double u, v;
  //tells the window there has been a getclickup demand
//   PostMessage(window.getHWnd(), vpWM_GETCLICK, 0,0);
  //waits for a click
  if(blocking){
    WaitForSingleObject(window.semaClick, NULL);
    WaitForSingleObject(window.semaClickUp, NULL);//to erase previous events
    WaitForSingleObject(window.semaClick, INFINITE);
    ret = true;
  }
  else
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClick, NULL));
  
  u = window.clickX;
  v = window.clickY;
  ip.set_u( u );
  ip.set_v( v );
  button = window.clickButton;

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
bool vpDisplayWin32::getClickUp(vpImagePoint &ip,
                                vpMouseButton::vpMouseButtonType& button,
                                bool blocking)
{
  //wait if the window is not initialized
  waitForInit();
  bool ret = false;
  double u, v;
  //tells the window there has been a getclickup demand
//   PostMessage(window.getHWnd(), vpWM_GETCLICKUP, 0,0);

  //waits for a click release
  if(blocking){
    WaitForSingleObject(window.semaClickUp, NULL);
    WaitForSingleObject(window.semaClick, NULL);//to erase previous events
    WaitForSingleObject(window.semaClickUp, INFINITE);
    ret = true;
  }
  else
    ret = (WAIT_OBJECT_0 == WaitForSingleObject(window.semaClickUp, NULL));
  
  u = window.clickXUp;
  v = window.clickYUp;
  ip.set_u( u );
  ip.set_v( v );
  button = window.clickButtonUp;

  return ret;
}

/*!
  Get a keyboard event.

  \warning Not implemented yet.

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
bool vpDisplayWin32::getKeyboardEvent( bool /* blocking */)
{
  vpTRACE("Not implemented yet.");
  return false;
}
/*!

  Get a keyboard event.

  \warning Not implemented yet.

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
bool vpDisplayWin32::getKeyboardEvent(char * /* string */, bool /* blocking */)
{
  vpTRACE("Not implemented yet.");
  return false;
}

/*!
  Changes the window's position.

  \param winx, winy : Position of the upper-left window's border in the screen.

*/
void vpDisplayWin32::setWindowPosition(int winx, int winy)
{
  //wait if the window is not initialized
  waitForInit();

  //cahange the window position only
  SetWindowPos(window.hWnd,HWND_TOP, winx, winy, 0, 0,
	       SWP_ASYNCWINDOWPOS | SWP_NOACTIVATE | SWP_NOZORDER |SWP_NOSIZE);

}


/*!
  Changes the window's titlebar text

  \param windowtitle : Window title.
*/
void vpDisplayWin32::setTitle(const char *windowtitle)
{
  //wait if the window is not initialized
  waitForInit();
  SetWindowText(window.hWnd, windowtitle);
}


/*!
  \brief Set the font used to display text.
  \param fontname : Name of the font.
 */

void vpDisplayWin32::setFont(const char *fontname)
{
	vpERROR_TRACE("Not yet implemented" ) ;
}


/*!
  \brief flush the Win32 buffer
  It's necessary to use this function to see the results of any drawing

*/
void vpDisplayWin32::flushDisplay()
{
  //waits if the window is not initialized
  waitForInit();

  //sends a message to the window
  PostMessage(window.getHWnd(), vpWM_DISPLAY, 0,0);
}


/*!
  Display a point at the image point \e ip location.
  \param ip : Point location.
  \param color : Point color.
*/
void vpDisplayWin32::displayPoint(const vpImagePoint &ip,
                                  vpColor color )
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->setPixel(ip, color);
}

/*!
  Display a line from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayWin32::displayLine( const vpImagePoint &ip1, 
			                      const vpImagePoint &ip2,
                                  vpColor color, 
			                      unsigned int thickness )
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawLine(ip1, ip2, color, thickness);
}


/*!
  Display a dashed line from image point \e ip1 to image point \e ip2.

  \warning This line is a dashed line only if the thickness is equal to 1.

  \param ip1,ip2 : Initial and final image points.
  \param color : Line color.
  \param thickness : Line thickness.
*/
void vpDisplayWin32::displayDotLine(const vpImagePoint &ip1, 
				    const vpImagePoint &ip2,
                                    vpColor color, 
				    unsigned int thickness )
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawLine(ip1,ip2,color,thickness,PS_DASHDOT);
}

/*!  
  Display a rectangle with \e topLeft as the top-left corner and \e
  width and \e height the rectangle size.

  \param topLeft : Top-left corner of the rectangle.
  \param width,height : Rectangle size.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the four lines used to display the
  rectangle.

  \warning The thickness can not be set if the display uses the d3d library.
*/
void vpDisplayWin32::displayRectangle( const vpImagePoint &topLeft,
                                       unsigned int width, unsigned int height,
                                       vpColor color, bool fill,
			               unsigned int thickness )
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawRect(topLeft,width,height,color, fill, thickness);
}


/*!  
  Display a rectangle.

  \param topLeft : Top-left corner of the rectangle.
  \param bottomRight : Bottom-right corner of the rectangle.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the four lines used to display the
  rectangle.

  \warning The thickness can not be set if the display uses the d3d library.
*/
void vpDisplayWin32::displayRectangle( const vpImagePoint &topLeft,
                                       const vpImagePoint &bottomRight,
                                       vpColor color, bool fill,
			               unsigned int thickness )
{
  //wait if the window is not initialized
  waitForInit();
  double width = bottomRight.get_j() - topLeft.get_j();
  double height = bottomRight.get_i() - topLeft.get_i();
  window.renderer->drawRect(topLeft,(int)width,(int)height,color, fill, thickness);
}

/*!
  Display a rectangle.

  \param rectangle : Rectangle characteristics.
  \param color : Rectangle color.
  \param fill : When set to true fill the rectangle.
  \param thickness : Thickness of the four lines used to display the
  rectangle.

  \warning The thickness can not be set if the display uses the d3d library.
*/
void vpDisplayWin32::displayRectangle( const vpRect &rectangle,
                                       vpColor color, bool fill,
			               unsigned int thickness )
{
  //wait if the window is not initialized
  waitForInit();
  vpImagePoint topLeft;
  topLeft.set_i(rectangle.getTop());
  topLeft.set_j(rectangle.getLeft());
  window.renderer->drawRect(topLeft,(int)rectangle.getWidth(),(int)rectangle.getHeight(),
			    color, fill, thickness);
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
void vpDisplayWin32::displayCircle(const vpImagePoint &center,
				   unsigned int radius,
                                   vpColor color,
				   bool fill,
				   unsigned int thickness )
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawCircle(center,radius,color,fill,thickness);
}

/*!
  Displays a string.
  \param ip : its top left point's coordinates
  \param text : The string to display
  \param color : The text's color
*/
void vpDisplayWin32::displayCharString(const vpImagePoint &ip,
                                     const char *text, 
				     vpColor color )
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawText(ip,text,color);
}

/*!
  Display a cross at the image point \e ip location.
  \param ip : Cross location.
  \param size : Size (width and height) of the cross.
  \param color : Cross color.
  \param thickness : Thickness of the lines used to display the cross.
*/
void vpDisplayWin32::displayCross( const vpImagePoint &ip, 
                                   unsigned int size, 
				   vpColor color,
				   unsigned int thickness)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawCross(ip, size, color, thickness);
}


/*!
  Display an arrow from image point \e ip1 to image point \e ip2.
  \param ip1,ip2 : Initial and final image point.
  \param color : Arrow color.
  \param w,h : Width and height of the arrow.
  \param thickness : Thickness of the lines used to display the arrow.
*/
void vpDisplayWin32::displayArrow(const vpImagePoint &ip1, 
		    const vpImagePoint &ip2,
		    vpColor color,
		    unsigned int w,unsigned int h,
		    unsigned int thickness)

{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawArrow(ip1, ip2, color, w, h, thickness);
}


/*!
  Clears the display.
  \param color : the color to fill the display with
*/
void vpDisplayWin32::clearDisplay(vpColor color){
  //wait if the window is not initialized
  waitForInit();
  window.renderer->clear(color);
}


/*!
  Closes the display.
  Destroys the window.
*/
void vpDisplayWin32::closeDisplay()
{
  if (displayHasBeenInitialized) {
    waitForInit();
    PostMessage(window.getHWnd(), vpWM_CLOSEDISPLAY, 0,0);
    //if the destructor is called for a reason different than a
    //problem in the thread creation
    if (iStatus) {
      //waits for the thread to end
      WaitForSingleObject(hThread, INFINITE);
      CloseHandle(hThread);
    }
    displayHasBeenInitialized = false ;
	window.initialized = false ;
  }
  if (this->title != NULL) {
    delete [] this->title;
    this->title = NULL;
  }
}

/*!
  Gets the displayed image (if overlay, if any).
  \param I : Image to fill.
*/
void vpDisplayWin32::getImage(vpImage<vpRGBa> &I)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->getImage(I);
}

#endif

