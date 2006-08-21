/****************************************************************************
 *
 * $Id: vpDisplayWin32.cpp,v 1.2 2006-08-21 10:02:43 brenier Exp $
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


/*!
	Thread entry point.
	Used as a detour to initWindow.
*/
void createWindow(threadParam * param)
{
	string title = param->title;
	(param->vpDisp)->window.initWindow(title, param->x, param->y, param->w, param->h);
	delete param;
}

/*!
	Constructor.
*/
vpDisplayWin32::vpDisplayWin32(vpWin32Renderer * rend) : iStatus(false), window(rend)
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

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
void vpDisplayWin32::init(vpImage<unsigned char> &I,
		   int _x,
		   int _y,
		   char *_title)
 {

   if ((I.getRows() == 0) || (I.getCols()==0))
     {
       vpERROR_TRACE("Image not initialized " ) ;
       throw(vpDisplayException(vpDisplayException::notInitializedError,
				"Image not initialized")) ;
     }

   window.renderer->setImg(I);

   init (I.getCols(), I.getRows(), _x, _y, _title) ;
   I.display = this ;
   I.initDisplay =  true ;

 }

/*!
  \brief Initialized the display of a RGBa  image

  \param I : image to be displayed (not that image has to be initialized)
  \param _x, _y The window is set at position x,y (column index, row index).
  \param _title  window  titled

*/
void vpDisplayWin32::init(vpImage<vpRGBa> &I,
		   int _x,
		   int _y,
		   char *_title)
{
  if ((I.getRows() == 0) || (I.getCols()==0))
     {
       vpERROR_TRACE("Image not initialized " ) ;
       throw(vpDisplayException(vpDisplayException::notInitializedError,
				"Image not initialized")) ;
     }

  window.renderer->setImg(I);

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
void vpDisplayWin32::init(int _ncol, int _nrow,
		   int _x, int _y,
		   char *_title)
{


	if (_title != NULL)//delete après init du thread.... ou destructeur
	{
		title = new char[strlen(_title) + 1] ;
		strcpy(title,_title) ;
	}

	//we prepare the window's thread creation
	threadParam * param = new threadParam;
	param->x = _x;
	param->y = _y;
	param->w = _ncol;
	param->h = _nrow;
	param->vpDisp = this;
	param->title = title;

	//creates the window in a separate thread
	hThread = CreateThread(NULL,0,(LPTHREAD_START_ROUTINE)createWindow,param,0,&threadId);

	//the initialization worked
	iStatus = (hThread != (HANDLE)NULL);

	displayHasBeenInitialized =true;
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
		if( WAIT_OBJECT_0 != WaitForSingleObject(window.semaInit, MAX_INIT_DELAY) )
			 throw(vpDisplayException(vpDisplayException::notInitializedError,
				"Window not initialized")) ;
				//problem : the window is not initialized
	}
}

/*!
	Displays an RGBa image in the window.
	\param I image to display
*/
void vpDisplayWin32::displayImage(vpImage<vpRGBa> &I)
{	
	//waits if the window is not initialized
	waitForInit();

	//sets the image to render
	window.renderer->setImg(I);
	//sends a message to the window 
	PostMessage(window.getHWnd(),vpWM_DISPLAY,NULL,NULL);
}

/*!
	Displays a grayscale image in the window.
	\param I image to display
*/
void vpDisplayWin32::displayImage(vpImage<unsigned char> &I)
{
	//wait if the window is not initialized
	waitForInit();
	
	//sets the image to render
	window.renderer->setImg(I);
	//sends a message to the window 
	PostMessage(window.getHWnd(), vpWM_DISPLAY, NULL, NULL);
}

/*!
	Waits for a click and returns its coordinates.
	\param i first coordinate of the click position
	\param j second coordinate of the click position
	\return true
*/
bool vpDisplayWin32::getClick(int& i, int& j)
{
	//wait if the window is not initialized
	waitForInit();

	//we don't care about which button is pressed
	window.clickButton = vpNO_BUTTON_QUERY;
	//tells the window there has been a getclickup demand
	PostMessage(window.getHWnd(), vpWM_GETCLICK, NULL, NULL);
	//waits for a click
	WaitForSingleObject(window.semaClick,INFINITE);

	j = window.clickX;
	i = window.clickY;
	return true;
}

/*!
	Waits for a click from a certain button and returns its coordinates.
	\param i first coordinate of the click position
	\param j second coordinate of the click position
	\param button button to use for the click
	\return true
*/
bool vpDisplayWin32::getClick(int& i, int& j, int& button)
{
	//wait if the window is not initialized
	waitForInit();

	//we want a click from a specific button
	window.clickButton = button;
	//tells the window there has been a getclickup demand
	PostMessage(window.getHWnd(), vpWM_GETCLICK, NULL, NULL);
	//waits for a click
	WaitForSingleObject(window.semaClick, INFINITE);

	j = window.clickX;
	i = window.clickY;
	return true;
}

/*!
	Waits for a click "up" from a certain button and returns its coordinates.
	\param i first coordinate of the click position
	\param j second coordinate of the click position
	\param button button to use for the click
	\return true
*/
bool vpDisplayWin32::getClickUp(int& i, int& j, int& button)
{
	//wait if the window is not initialized
	waitForInit();

	//we want a click from a specific button
	window.clickButton = button;

	//tells the window there has been a getclickup demand
	PostMessage(window.getHWnd(), vpWM_GETCLICKUP, NULL, NULL);
	
	//waits for a click release
	WaitForSingleObject(window.semaClick, INFINITE);
	
	j = window.clickX;
	i = window.clickY;

	return true;
}

/*!
	Waits for a click.
*/
void vpDisplayWin32::getClick()
{
	//wait if the window is not initialized
	waitForInit();

	//we don't care about which button is pressed
	window.clickButton = vpNO_BUTTON_QUERY;
	//sends a message to the window
	PostMessage(window.getHWnd(), vpWM_GETCLICK, NULL, NULL);
	
	//waits for a button to be pressed
	WaitForSingleObject(window.semaClick, INFINITE);
}

/*!
	Changes the window's position
	\param _winx its first new coordinate
	\param _winy its second new coordinate
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
	\param string The new text to display
*/
void vpDisplayWin32::flushTitle(const char *string)
{
	//wait if the window is not initialized
	waitForInit();
	SetWindowText(window.hWnd,string);
}

/*!
	Displays a point.
	\param i its first coordinate
	\param j its second coordinate
	\param col The point's color
*/
void vpDisplayWin32::displayPoint(int i,int j,int col)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->setPixel(i,j,col);
}

/*!
	Displays a line.
	\param i1 its starting point's first coordinate
	\param j1 its starting point's second coordinate
	\param i2 its ending point's first coordinate
	\param j2 its ending point's second coordinate
	\param e width of the line
	\param col The point's color
*/
void vpDisplayWin32::displayLine(int i1, int j1, int i2, int j2, int col, int e)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawLine(i1,j1,i2,j2,col,e);
}

/*!
	Displays a dotted line.
	\param i1 its starting point's first coordinate
	\param j1 its starting point's second coordinate
	\param i2 its ending point's first coordinate
	\param j2 its ending point's second coordinate
	\param e width of the line
	\param col The line's color
*/
void vpDisplayWin32::displayDotLine(int i1, int j1, int i2, int j2, int col, int e)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawLine(i1,j1,i2,j2,col,e,PS_DASHDOT);
}

/*!
	Displays a rectangle.
	\param i its top left point's first coordinate
	\param j its top left point's second coordinate
	\param width width of the rectangle
	\param height height of the rectangle
	\param col The rectangle's color
*/
void vpDisplayWin32::displayRectangle(int i, int j, int width, int height, int col)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawRect(i,j,width,height,col);
}

/*!
	Displays a circle.
	\param i its center point's first coordinate
	\param j its center point's second coordinate
	\param r The circle's radius
	\param col The circle's color
*/
void vpDisplayWin32::displayCircle(int i, int j, int r, int c)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawCircle(i,j,r,c);
}

/*!
	Displays a string.
	\param i its top left point's first coordinate
	\param j its top left point's second coordinate
	\param s The string to display
	\param col The text's color
*/
void vpDisplayWin32::displayCharString(int i,int j,char *s, int c)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawText(i,j,s,c);
}

/*!
	Displays a cross.
	\param i its center point's first coordinate
	\param j its center point's second coordinate
	\param size Size of the cross
	\param col The cross' color
*/
void vpDisplayWin32::displayCross(int i,int j, int size,int col)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawCross(i, j, size, col);
}

/*!
	Displays a large cross.
	\param i its center point's first coordinate
	\param j its center point's second coordinate
	\param size Size of the cross
	\param col The cross' color
*/
void vpDisplayWin32::displayCrossLarge(int i,int j, int size,int col)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawCross(i, j, size, col, 3);
}

/*!
	Displays an arrow.
	\param i1 its starting point's first coordinate
	\param j1 its starting point's second coordinate
	\param i2 its ending point's first coordinate
	\param j2 its ending point's second coordinate
	\param col The line's color
	\param L ...
	\param l ...
*/
void vpDisplayWin32::displayArrow(int i1,int j1, int i2, int j2, int col, int L,int l)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->drawArrow(i1, j1, i2, j2, col, L, l);
}


/*!
	Clears the display.
	\param c the color to fill the display with
*/
void vpDisplayWin32::clearDisplay(int c)
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
	//wait if the window is not initialized
	waitForInit();

	//tells the window that it has to close
	PostMessage(window.getHWnd(), vpWM_CLOSEDISPLAY, NULL, NULL);

	//if the destructor is called for a reason different than a problem in the thread creation
	if (iStatus)
	{
		//waits for the thread to end
		WaitForSingleObject(hThread, INFINITE);
		CloseHandle(hThread);
	};
}

/*!
	Gets the displayed image (if overlay, if any).
	\param I Image to fill.
*/
void vpDisplayWin32::getImage(vpImage<vpRGBa> &I)
{
	//wait if the window is not initialized
	waitForInit();
	window.renderer->getImage(I);
}

#endif

