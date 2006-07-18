/****************************************************************************
 *
 * $Id: vpWin32Window.h,v 1.1 2006-07-18 14:43:30 brenier Exp $
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
 * Windows 32 display's window class
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#if ( defined(WIN32) ) 
#ifndef vpWin32Window_HH
#define vpWin32Window_HH


#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <windows.h>
#include <visp/vpGDIRenderer.h>
#include <visp/vpDisplay.h>


//ViSP-defined messages for window's callback function
#define vpWM_GETCLICK WM_USER+1
#define vpWM_DISPLAY WM_USER+2
#define vpWM_GETCLICKUP WM_USER+3
#define vpWM_CLOSEDISPLAY WM_USER+4

//No specific mouse button query 
#define vpNO_BUTTON_QUERY -1

class vpWin32Display;

class VISP_EXPORT vpWin32Window
{

	HINSTANCE hInst;

	//! Window's handle
	HWND hWnd;

	//! Window is initialized
	bool initialized;
	//! Handle for the initialization semaphore
	HANDLE semaInit;

	//! Handle for the getClick semaphore
	HANDLE semaClick;

	//! If there is a getClick demand
	bool waitForClick;
	//! If there is a getClickUp demand
	bool waitForClickUp;
	//! X coordinate of the click
	int clickX;
	//! Y coordinate of the click
	int clickY;
	//! Button used for the click
	int clickButton;
	
	//! True if the window's class has already been registered
	static bool registered;

	//! The renderer used by the window
	vpWin32Renderer * renderer;


public:

	vpWin32Window(vpWin32Renderer * rend = NULL);
	~vpWin32Window();

	//! Returns the displayed image's width
	int getImageWidth(){ return renderer->getImageWidth(); }
	//! Returns the displayed image's height
	int getImageHeight(){ return renderer->getImageHeight(); }
	//! Returns the window's handle
	HWND getHWnd(){ return hWnd;}

	//! Returns true if the window is initialized
	bool isInitialized(){ return initialized; }

	//! Initialize the window
	void initWindow(string title, int posx, int posy, int w, int h);

	// Friend classes
	friend class vpDisplayWin32;
	friend class vpDirect3DDisplay;
	friend class vpDisplayGDI;

	//! The message loop
	friend LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam);
};

#endif
#endif
#endif
