/****************************************************************************
 *
 * $Id: vpDisplayWin32.cpp,v 1.25 2008-11-10 10:26:39 fspindle Exp $
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
  \brief Initialized the display of a gray level image

  \param I : Image to be displayed (note that image has to be initialized).
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
  \brief Initialized the display of a RGBa  image

  \param I : Image to be displayed (note that image has to be initialized).
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
  \brief actual member used to Initialize the display of a
  gray level or RGBa  image

  \param width, height : weight, height of the window
  \param x, y : The window is set at position x,y (column index, row index).
  \param title : Window title.

*/
void vpDisplayWin32::init(unsigned int width, unsigned int height,
			  int x, int y,
			  const char *title)
{


  if (this->title != NULL)//delete après init du thread.... ou destructeur
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
  Displays an RGBa image in the window.
  \param I : image to display
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
  Displays a grayscale image in the window.
  \param I : image to display
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
  Waits for a click and returns its coordinates.
  \param i : first coordinate of the click position
  \param j : second coordinate of the click position
  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return 
  - true if a button was clicked. This is always the case if blocking is set 
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayWin32::getClick(unsigned int& i, unsigned int& j, bool blocking)
{
  //wait if the window is not initialized
  waitForInit();

  bool ret = false ;
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
  
  j = window.clickX;
  i = window.clickY;

  return ret;
}

/*!
  Waits for a click from a certain button and returns its coordinates.
  \param i : first coordinate of the click position
  \param j : second coordinate of the click position
  \param button : button to use for the click
  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return 
  - true if a button was clicked. This is always the case if blocking is set 
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayWin32::getClick(unsigned int& i, unsigned int& j,
                              vpMouseButton::vpMouseButtonType& button,
                              bool blocking)
{
  //wait if the window is not initialized
  waitForInit();
  bool ret = false;
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
  
  j = window.clickX;
  i = window.clickY;
  button = window.clickButton;

  return ret;
}

/*!
  Waits for a click "up" from a certain button and returns its coordinates.
  \param i : first coordinate of the click position
  \param j : second coordinate of the click position
  \param button : button to use for the click
  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

  \return 
  - true if a button was clicked. This is always the case if blocking is set 
    to \e true.
  - false if no button was clicked. This can occur if blocking is set
    to \e false.
*/
bool vpDisplayWin32::getClickUp(unsigned int& i, unsigned int& j,
                                vpMouseButton::vpMouseButtonType& button,
                                bool blocking)
{
  //wait if the window is not initialized
  waitForInit();
  bool ret = false;
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
  
  j = window.clickXUp;
  i = window.clickYUp;
  button = window.clickButtonUp;

  return ret;
}

/*!
  Waits for a click.
  \param blocking : true for a blocking behaviour waiting a mouse
  button click, false for a non blocking behaviour.

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
  Changes the window's position
  \param _winx : its first new coordinate
  \param _winy : its second new coordinate
*/
void vpDisplayWin32::setWindowPosition(int _winx, int _winy)
{
  //wait if the window is not initialized
  waitForInit();

  //cahange the window position only
  SetWindowPos(window.hWnd,HWND_TOP,_winx,_winy, 0, 0,
	       SWP_ASYNCWINDOWPOS | SWP_NOACTIVATE | SWP_NOZORDER |SWP_NOSIZE);

}

/*!
  Changes the window's titlebar text
  \deprecated Use setTitle() instead.
  \param string The new text to display
*/
void vpDisplayWin32::flushTitle(const char *string)
{
  //wait if the window is not initialized
  waitForInit();
  SetWindowText(window.hWnd,string);
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
  Displays a point.
  \param i : its first coordinate
  \param j : its second coordinate
  \param col : The point's color
*/
void vpDisplayWin32::displayPoint(int i, int j,
				  vpColor::vpColorType col)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->setPixel(i,j,col);
}

/*!
  Displays a line.
  \param i1 : its starting point's first coordinate
  \param j1 : its starting point's second coordinate
  \param i2 : its ending point's first coordinate
  \param j2 : its ending point's second coordinate
  \param e : width of the line
  \param col : The point's color
*/
void vpDisplayWin32::displayLine(int i1, int j1, int i2, int j2,
				 vpColor::vpColorType col, unsigned int e)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawLine(i1,j1,i2,j2,col,e);
}

/*!
  Displays a dotted line.
  \param i1 : its starting point's first coordinate
  \param j1 : its starting point's second coordinate
  \param i2 : its ending point's first coordinate
  \param j2 : its ending point's second coordinate
  \param e : width of the line
  \param col : The line's color
*/
void vpDisplayWin32::displayDotLine(int i1, int j1, int i2, int j2,
				    vpColor::vpColorType col, unsigned int e)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawLine(i1,j1,i2,j2,col,e,PS_DASHDOT);
}

/*!
  Displays a rectangle.
  \param i : its top left point's first coordinate
  \param j : its top left point's second coordinate
  \param width : width of the rectangle
  \param height : height of the rectangle
  \param col : The rectangle's color
  \param fill : set as true to fill the rectangle.
  \param e : line thick
*/
void vpDisplayWin32::displayRectangle(int i, int j,
				      unsigned int width, unsigned int height,
				      vpColor::vpColorType col, bool fill,
				      unsigned int e)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawRect(i,j,width,height,col, fill, e);
}
/*!
  Displays a rectangle.
  \param rect : Rectangle characteristics.
  \param col : The rectangle's color
  \param fill : set as true to fill the rectangle.
  \param e : line thick
*/
void vpDisplayWin32::displayRectangle(const vpRect &rect,
				      vpColor::vpColorType col, bool fill,
				      unsigned int e)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawRect((int)rect.getTop(),(int)rect.getLeft(),
			    (int)rect.getWidth(),(int)rect.getHeight(),
			    col, fill, e);
}

/*!
  Displays a circle.
  \param i : its center point's first coordinate
  \param j : its center point's second coordinate
  \param r : The circle's radius
  \param c : The circle's color
*/
void vpDisplayWin32::displayCircle(int i, int j,
				   unsigned int r,
				   vpColor::vpColorType c)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawCircle(i,j,r,c);
}

/*!
  Displays a string.
  \param i : its top left point's first coordinate
  \param j : its top left point's second coordinate
  \param s : The string to display
  \param c : The text's color
*/
void vpDisplayWin32::displayCharString(int i, int j,const char *s,
				       vpColor::vpColorType c)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawText(i,j,s,c);
}

/*!
  Displays a cross.
  \param i : its center point's first coordinate
  \param j : its center point's second coordinate
  \param size : Size of the cross
  \param col : The cross' color
*/
void vpDisplayWin32::displayCross(int i, int j,
				  unsigned int size,
				  vpColor::vpColorType col)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawCross(i, j, size, col);
}

/*!
  Displays a large cross.
  \param i : its center point's first coordinate
  \param j : its center point's second coordinate
  \param size : Size of the cross
  \param col : The cross' color
*/
void vpDisplayWin32::displayCrossLarge(int i, int j,
				       unsigned int size,
				       vpColor::vpColorType col)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawCross(i, j, size, col, 3);
}

/*!
  Displays an arrow.
  \param i1 : its starting point's first coordinate
  \param j1 : its starting point's second coordinate
  \param i2 : its ending point's first coordinate
  \param j2 : its ending point's second coordinate
  \param col : The line's color
  \param L : ...
  \param l : ...
*/
void vpDisplayWin32::displayArrow(int i1, int j1,
				  int i2, int j2,
				  vpColor::vpColorType col,
				  unsigned int L, unsigned int l)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->drawArrow(i1, j1, i2, j2, col, L, l);
}


/*!
  Clears the display.
  \param c : the color to fill the display with
*/
void vpDisplayWin32::clearDisplay(vpColor::vpColorType c)
{
  //wait if the window is not initialized
  waitForInit();
  window.renderer->clear(c);
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

