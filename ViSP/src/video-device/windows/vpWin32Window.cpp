/****************************************************************************
 *
 * $Id: vpWin32Window.cpp,v 1.5 2007-06-04 09:12:28 fspindle Exp $
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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpWin32Window.h>

//Should be already defined ...
#ifndef GET_X_LPARAM
# define GET_X_LPARAM(lp)                        ((int)(short)LOWORD(lp))
#endif

#ifndef GET_Y_LPARAM
# define GET_Y_LPARAM(lp)                        ((int)(short)HIWORD(lp))
#endif

//declares the window as thread local
//allows multiple displays
_declspec(thread) vpWin32Window * window;

bool vpWin32Window::registered = false;

/*!
  The message callback
*/
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  //the first time this callback is executed, put the window in initialized state
  if(window != NULL)
    {
      if(!window->isInitialized())
	{
	  window->initialized = true;
	  ReleaseSemaphore(window->semaInit,1,NULL);
	}
    }

  switch (message) 
    {
    case vpWM_DISPLAY:
      //redraw the whole window
      InvalidateRect(window->getHWnd(), NULL, true);
      UpdateWindow(window->getHWnd());
      break;


      //Beginning of mouse input handlers

    case vpWM_GETCLICK:
      window->waitForClick = true;
      break;

    case vpWM_GETCLICKUP:
      window->waitForClickUp = true;
      break;
		
		
    case WM_LBUTTONDOWN:
      //if there has been a "click demand"
      if(window->waitForClick) 
	{	
	  window->clickX = GET_X_LPARAM(lParam);
	  window->clickY = GET_Y_LPARAM(lParam);

	  window->clickButton = vpMouseButton::button1;
	  ReleaseSemaphore(window->semaClick,1,NULL);
	  window->waitForClick = false;
	}
      break;

    case WM_MBUTTONDOWN:
      //if there has been a "click demand"
      if(window->waitForClick) 
	{	
	  window->clickX = GET_X_LPARAM(lParam);
	  window->clickY = GET_Y_LPARAM(lParam);

	  window->clickButton = vpMouseButton::button2;
	  ReleaseSemaphore(window->semaClick,1,NULL);
	  window->waitForClick = false;
	}
      break;

    case WM_RBUTTONDOWN:
      //if there has been a "click demand"
      if(window->waitForClick) 
	{	
	  window->clickX = GET_X_LPARAM(lParam);
	  window->clickY = GET_Y_LPARAM(lParam);

	  window->clickButton = vpMouseButton::button3;
	  ReleaseSemaphore(window->semaClick,1,NULL);
	  window->waitForClick = false;
	}
      break;

    case WM_LBUTTONUP:
      if(window->waitForClickUp) 
	{	
	  window->clickX = GET_X_LPARAM(lParam);
	  window->clickY = GET_Y_LPARAM(lParam);
					
	  window->clickButton = vpMouseButton::button1;
	  ReleaseSemaphore(window->semaClick,1,NULL);
	  window->waitForClickUp = false;
	}
      break;

    case WM_MBUTTONUP:
      if(window->waitForClickUp) 
	{	
	  window->clickX = GET_X_LPARAM(lParam);
	  window->clickY = GET_Y_LPARAM(lParam);

	  window->clickButton = vpMouseButton::button2;
	  ReleaseSemaphore(window->semaClick,1,NULL);
	  window->waitForClickUp = false;
	}
      break;

    case WM_RBUTTONUP:
      if(window->waitForClickUp) 
	{	
	  window->clickX = GET_X_LPARAM(lParam);
	  window->clickY = GET_Y_LPARAM(lParam);

	  window->clickButton = vpMouseButton::button3;
	  ReleaseSemaphore(window->semaClick,1,NULL);
	  window->waitForClickUp = false;
	}
      break;


    case WM_COMMAND:
			
      break;

      //we must prevent the window from erasing the background each time a
      //repaint is needed
    case WM_ERASEBKGND:
      return (LRESULT)1; 

    case WM_PAINT:
      //render the display
      window->renderer->render();
      break;

    case vpWM_CLOSEDISPLAY:
      //cleanup code here, if needed
      //destroys the window
      DestroyWindow(hWnd);
      break;

    case WM_DESTROY:
      PostQuitMessage(0);
      break;
    default:
      return DefWindowProc(hWnd, message, wParam, lParam);
    }
  return 0;
}

/*!
  Constructor.
*/
vpWin32Window::vpWin32Window(vpWin32Renderer * rend): initialized(false)
{
  renderer = rend;

  //creates the semaphores
  semaInit = CreateSemaphore(NULL,0,1,NULL);
  semaClick = CreateSemaphore(NULL,0,1,NULL);

  //no queries directly after initialization
  waitForClick = false;
  waitForClickUp = false;
}

/*!
  Destructor.
*/
vpWin32Window::~vpWin32Window()
{
  delete renderer;
  CloseHandle(semaInit);
  CloseHandle(semaClick);
}


/*!
  A standard window creation procedure.
  Initialize the renderer and associate it with this thread
  \param title String to display in the window's titlebar
  \param posx Initial window X position
  \param posy Initial window Y position
  \param w Initial window's width
  \param h Initial window's height

*/
void vpWin32Window::initWindow(std::string title, int posx, int posy, int w, int h)
{
  //the window's style
  DWORD style = WS_CAPTION|WS_SYSMENU|WS_MINIMIZEBOX |WS_MAXIMIZEBOX;
  const char g_szClassName[] = "ViSPWindowClass";
	
  RECT rect;
  rect.left=0;
  rect.right=w;
  rect.top=0;
  rect.bottom=h;

  AdjustWindowRectEx(&rect, style, false, 0);

  //the window's required dimensions to have a client area of w*h
  int windowW = rect.right - rect.left - 4;
  int windowH = rect.bottom - rect.top - 4;

  //now we register the window's class
  WNDCLASSEX wcex;

  wcex.cbSize = sizeof(WNDCLASSEX); 

  wcex.style			= CS_HREDRAW | CS_VREDRAW | CS_NOCLOSE;
  wcex.lpfnWndProc	= (WNDPROC)WndProc;
  wcex.cbClsExtra		= 0;
  wcex.cbWndExtra		= 0;
  wcex.hInstance		= hInst;
  wcex.hIcon			= LoadIcon(NULL, IDI_APPLICATION);
  wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
  wcex.hbrBackground	= (HBRUSH)(COLOR_WINDOW+1);
  wcex.lpszMenuName	= NULL;
  wcex.lpszClassName	= g_szClassName;
  wcex.hIconSm		= LoadIcon(NULL, IDI_APPLICATION);

  //we register it only if it is not yet registered
  if(!registered)
    {
      if(!RegisterClassEx(&wcex))
	{
	  throw vpDisplayException(vpDisplayException::cannotOpenWindowError,
				   "Can't register the window's class!");
	}
      else{ registered = true; }
    }
	
  //creates the window
  hWnd = CreateWindow(g_szClassName, title.c_str(), style,
		      posx, posy, windowW, windowH, NULL, NULL, hInst, NULL);

  if (hWnd == NULL)
    {
      throw vpDisplayException(vpDisplayException::cannotOpenWindowError, 
			       "Can't create the window!");
    }

  //needed if we want to access it from the callback method (message handler)
  window = this;

  //initialize the renderer
  renderer->init(hWnd, w, h);

  //displays the window
  ShowWindow(hWnd, SW_SHOWDEFAULT);
  UpdateWindow(hWnd);

  MSG msg;

  //starts the message loop
  while (GetMessage(&msg, NULL, 0, 0)) 
    {
      if (!TranslateAccelerator(msg.hwnd, NULL, &msg)) 
	{
	  TranslateMessage(&msg);
	  DispatchMessage(&msg);
	}
    }
}

#endif
#endif
