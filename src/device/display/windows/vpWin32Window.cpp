/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Windows 32 display's window class
 *
 * Authors:
 * Bruno Renier
 * Anthony Saunier
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <iostream>

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
  if (window != NULL)
  {
    if (!window->isInitialized())
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

    case vpWM_DISPLAY_ROI:
	{
      //redraw the whole window
	  RECT* rect = (RECT *)wParam;
      InvalidateRect(window->getHWnd(), rect, true);
      UpdateWindow(window->getHWnd());
	}
      break;

    case WM_LBUTTONDOWN:
     {
        window->clickX = GET_X_LPARAM(lParam);
        window->clickY = GET_Y_LPARAM(lParam);

        window->clickButton = vpMouseButton::button1;
        ReleaseSemaphore(window->semaClick,1,NULL);
      }
      break;

    case WM_MBUTTONDOWN:
      {
        window->clickX = GET_X_LPARAM(lParam);
        window->clickY = GET_Y_LPARAM(lParam);

        window->clickButton = vpMouseButton::button2;
        ReleaseSemaphore(window->semaClick,1,NULL);
      }
      break;

    case WM_RBUTTONDOWN:
      {
        window->clickX = GET_X_LPARAM(lParam);
        window->clickY = GET_Y_LPARAM(lParam);

        window->clickButton = vpMouseButton::button3;
        ReleaseSemaphore(window->semaClick,1,NULL);
       }
      break;

    case WM_LBUTTONUP:
      {
        window->clickXUp = GET_X_LPARAM(lParam);
        window->clickYUp = GET_Y_LPARAM(lParam);

        window->clickButtonUp = vpMouseButton::button1;
        ReleaseSemaphore(window->semaClickUp,1,NULL);
      }
      break;

    case WM_MBUTTONUP:
      {
        window->clickXUp = GET_X_LPARAM(lParam);
        window->clickYUp = GET_Y_LPARAM(lParam);

        window->clickButtonUp = vpMouseButton::button2;
        ReleaseSemaphore(window->semaClickUp,1,NULL);
      }
      break;

    case WM_RBUTTONUP:
      {
        window->clickXUp = GET_X_LPARAM(lParam);
        window->clickYUp = GET_Y_LPARAM(lParam);

        window->clickButtonUp = vpMouseButton::button3;
        ReleaseSemaphore(window->semaClickUp,1,NULL);
      }
      break;
    case WM_MOUSEMOVE:
      {
        window->coordX = GET_X_LPARAM(lParam);
        window->coordY = GET_Y_LPARAM(lParam);
		ReleaseSemaphore(window->semaMove,1,NULL);
      }
      break;

    case WM_SYSKEYDOWN: 
      // case WM_SYSKEYUP:
    case WM_KEYDOWN: 
      //case WM_KEYUP: 
      {
	GetKeyNameText(lParam, window->lpString, 10); // 10 is the size of lpString
	//window->key = MapVirtualKey(wParam, MAPVK_VK_TO_CHAR);
	ReleaseSemaphore(window->semaKey,1,NULL);
	break;
      }

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
  
  //registered is static member class and is initialized at the beginning of this file (registered = false)
  
  //creates the semaphores
  semaInit = CreateSemaphore(NULL,0,1,NULL);
  semaClick = CreateSemaphore(NULL,0,1,NULL);
  semaClickUp = CreateSemaphore(NULL,0,1,NULL);
  semaKey = CreateSemaphore(NULL,0,1,NULL);
  semaMove = CreateSemaphore(NULL,0,1,NULL);

}

/*!
  Destructor.
*/
vpWin32Window::~vpWin32Window()
{
  delete renderer;
  CloseHandle(semaInit);
  CloseHandle(semaClick);
  CloseHandle(semaClickUp);
  CloseHandle(semaKey);
  CloseHandle(semaMove);
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
void vpWin32Window::initWindow(const char* title, int posx, int posy, unsigned int w, unsigned int h)
{
  //the window's style
  DWORD style = WS_CAPTION|WS_SYSMENU|WS_MINIMIZEBOX |WS_MAXIMIZEBOX;
  const char g_szClassName[] = "ViSPWindowClass";

  RECT rect;
  rect.left=0;
  rect.right=static_cast<int>(w);
  rect.top=0;
  rect.bottom=static_cast<int>(h);

  AdjustWindowRectEx(&rect, style, false, 0);

  //the window's required dimensions to have a client area of w*h
  //int windowW = rect.right - rect.left - 4;
  //int windowH = rect.bottom - rect.top - 4;

  //now we register the window's class
  WNDCLASSEX wcex;

  wcex.cbSize = sizeof(WNDCLASSEX);

  wcex.style   = CS_HREDRAW | CS_VREDRAW | CS_NOCLOSE ;
  wcex.lpfnWndProc = (WNDPROC)WndProc;
  wcex.cbClsExtra  = 0;
  wcex.cbWndExtra  = 0;
  wcex.hInstance  = hInst;
  wcex.hIcon   = LoadIcon(NULL, IDI_APPLICATION);
  wcex.hCursor  = LoadCursor(NULL, IDC_ARROW);
  wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW+1);
  wcex.lpszMenuName = NULL;
  wcex.lpszClassName = g_szClassName;
  wcex.hIconSm  = LoadIcon(NULL, IDI_APPLICATION);
  
  RegisterClassEx(&wcex);

  //creates the window
  hWnd = CreateWindowEx(WS_EX_APPWINDOW,g_szClassName, title, style,
                      posx, posy, static_cast<int>(w), static_cast<int>(h), NULL, NULL, hInst, NULL);
  if (hWnd == NULL)
  {
	DWORD err= GetLastError();
	std::cout << "err=" << err << std::endl;
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
