/****************************************************************************
 *
 * $Id: vpDisplayWin32.h,v 1.3 2006-09-05 08:02:01 fspindle Exp $
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

#ifndef vpDisplayWin32_hh
#define vpDisplayWin32_hh

#include <visp/vpImage.h>
#include <visp/vpDisplay.h>
#include <windows.h>
#include <visp/vpWin32Window.h>


/*!
	Used to pass parameters to the window's thread.
*/
struct threadParam
{
  //! Pointer to the display associated with the window.
  vpDisplayWin32 * vpDisp;

  //! X position of the window.
  int x;

  //! Y position of the window.
  int y;

  //! Width of the window's client area.
  int w;

  //! Height of the window's client area.
  int h;

  //! Title of the window.
  char * title;
};


/*!
  \class vpDisplayWin32

  \brief Base abstract class for Windows 32 displays.
  Implements the window creation in a separate thread
  and the associated event handling functions for
  Windows 32 displays.
  Uses calls to a renderer to do some display.
  (i.e. all display methods are implemented in the renderer)

  \author Bruno Renier
*/
class VISP_EXPORT vpDisplayWin32 : public vpDisplay
{
protected:
	//! Maximum delay for window initialization
	static const int MAX_INIT_DELAY = 5000;

	//! Handle of the window's thread.
	HANDLE hThread;

	//! Id of the window's thread.
	DWORD threadId;

	//! Initialization status.
	bool iStatus;

	//! The window.
	vpWin32Window window;

	//! Function used to launch the window in a thread.
	friend void createWindow(threadParam * param);

public:

	vpDisplayWin32(vpWin32Renderer * rend = NULL);

	vpDisplayWin32(vpImage<vpRGBa> &I,
		int winx=-1, int winy=-1,
		char *title=NULL);

	vpDisplayWin32(vpImage<unsigned char> &I,
		int winx=-1, int winy=-1,
		char *title=NULL);

	virtual ~vpDisplayWin32();



	//! Initialization function
	void init(vpImage<unsigned char> &I,
		int winx=-1, int winy=-1,
		char *title=NULL)  ;

	//! Initialization function
	void init(vpImage<vpRGBa> &I,
		int winx=-1, int winy=-1,
		char *title=NULL)  ;

	//! Initialization function
	void init(int cols, int rows,
		int winx=-1, int winy=-1 ,
		char *title=NULL) ;

	//! Sets the window's position
	void setWindowPosition(int _winx, int _winy);

	//! Changes the window's title
	void flushTitle(const char *string);

	//! Displays a 8bits image
	void displayImage(vpImage<vpRGBa> &I);
	//! Displays a 32 bits image
	void displayImage(vpImage<unsigned char> &I);

	//! Closes the display
	void closeDisplay();

	//! Not used by this Display
	void flushDisplay(){}

	//! Clears the whole window
	void clearDisplay(int c=vpColor::white);

	//! Gets the window pixmap and put it in vpRGBa image
	void getImage(vpImage<vpRGBa> &I);

protected:
	//! Used to wait for the window to be initialized
	void waitForInit();

	//! Display a point at coordinates (i,j) in the display window
	void displayPoint(int i,int j,int col);

	//! Display a line from coordinates (i1,j1) to (i2,j2) in the display window.
	void displayLine(int i1, int j1, int i2, int j2, int col, int e=1);

	void displayRectangle(int i, int j, int width, int height, int col);

	//! Display a circle at coordinates (i,j) in the display window.
	void displayCircle(int i, int j, int r, int c);

	//! Display a dotted line from coordinates (i1,j1) to (i2,j2) in the display
	//! window.
	void displayDotLine(int i1, int j1, int i2, int j2, int col, int e=1);




	//! Display a cross at coordinates (i,j) in the display window
	void displayCross(int i,int j, int size,int col);
	//! Display a large cross at coordinates (i,j) in the display window
	void displayCrossLarge(int i,int j, int size,int col);

	//! Display an arrow from coordinates (i1,j1) to (i2,j2) in the display
	//! window
	void displayArrow(int i1,int j1, int i2, int j2, int col=1, int L=4,int l=2);

	void displayCharString(int i,int j,char *s, int c=vpColor::green);



	bool  getClick(int& i, int& j);

	//!  return true when button is pressed
	bool  getClick(int& i, int& j, int& button);

	//! return true when  button is released
	bool  getClickUp(int& i, int& j, int& button);

	//! wait for a click
	void  getClick();

};
#endif
#endif

