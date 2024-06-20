/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
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
 * Windows 32 display's window class
 */

#include <iostream>
#include <visp3/core/vpConfig.h>
#include <visp3/gui/vpWin32API.h>

#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9))

#define MAX_SEM_COUNT 2147483647

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/gui/vpWin32Window.h>

BEGIN_VISP_NAMESPACE

// Should be already defined ...
#ifndef GET_X_LPARAM
#define GET_X_LPARAM(lp) ((int)(short)LOWORD(lp))
#endif

#ifndef GET_Y_LPARAM
#define GET_Y_LPARAM(lp) ((int)(short)HIWORD(lp))
#endif

// declares the window as thread local
// allows multiple displays
#ifdef __MINGW32__
__thread vpWin32Window *window;
#else
__declspec(thread) vpWin32Window *window;
#endif

bool vpWin32Window::registered = false;
/*!
  The message callback
*/
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
  // the first time this callback is executed, put the window in initialized
  // state
  if (window != nullptr) {
    if (!window->isInitialized()) {
      window->initialized = true;
      vpReleaseSemaphore(window->semaInit, 1, nullptr);
    }
  }
  switch (message) {
  case vpWM_DISPLAY:
    // redraw the whole window
    InvalidateRect(window->getHWnd(), nullptr, TRUE);
    UpdateWindow(window->getHWnd());
    break;

  case vpWM_DISPLAY_ROI: {
    RECT rect;

    rect.left = LOWORD(wParam);
    rect.right = HIWORD(wParam);

    rect.top = LOWORD(lParam);
    rect.bottom = HIWORD(lParam);

    InvalidateRect(window->getHWnd(), &rect, TRUE);
    UpdateWindow(window->getHWnd());
  } break;

  case WM_LBUTTONDOWN: {
    window->clickX = GET_X_LPARAM(lParam);
    window->clickY = GET_Y_LPARAM(lParam);

    window->clickButton = vpMouseButton::button1;
    vpReleaseSemaphore(window->semaClick, 1, nullptr);
  } break;

  case WM_MBUTTONDOWN: {
    window->clickX = GET_X_LPARAM(lParam);
    window->clickY = GET_Y_LPARAM(lParam);

    window->clickButton = vpMouseButton::button2;
    vpReleaseSemaphore(window->semaClick, 1, nullptr);
  } break;

  case WM_RBUTTONDOWN: {
    window->clickX = GET_X_LPARAM(lParam);
    window->clickY = GET_Y_LPARAM(lParam);

    window->clickButton = vpMouseButton::button3;
    vpReleaseSemaphore(window->semaClick, 1, nullptr);
  } break;

  case WM_LBUTTONUP: {
    window->clickXUp = GET_X_LPARAM(lParam);
    window->clickYUp = GET_Y_LPARAM(lParam);

    window->clickButtonUp = vpMouseButton::button1;
    vpReleaseSemaphore(window->semaClickUp, 1, nullptr);
  } break;

  case WM_MBUTTONUP: {
    window->clickXUp = GET_X_LPARAM(lParam);
    window->clickYUp = GET_Y_LPARAM(lParam);

    window->clickButtonUp = vpMouseButton::button2;
    vpReleaseSemaphore(window->semaClickUp, 1, nullptr);
  } break;

  case WM_RBUTTONUP: {
    window->clickXUp = GET_X_LPARAM(lParam);
    window->clickYUp = GET_Y_LPARAM(lParam);

    window->clickButtonUp = vpMouseButton::button3;
    vpReleaseSemaphore(window->semaClickUp, 1, nullptr);
  } break;
  case WM_MOUSEMOVE: {
    window->coordX = GET_X_LPARAM(lParam);
    window->coordY = GET_Y_LPARAM(lParam);
    vpReleaseSemaphore(window->semaMove, 1, nullptr);
  } break;

  case WM_SYSKEYDOWN:
  // case WM_SYSKEYUP:
  case WM_KEYDOWN:
    // case WM_KEYUP:
  {
    GetKeyNameText((LONG)lParam, window->lpString,
                   10); // 10 is the size of lpString
    // window->key = MapVirtualKey(wParam, MAPVK_VK_TO_CHAR);
    vpReleaseSemaphore(window->semaKey, 1, nullptr);
    break;
  }

  case WM_COMMAND:

    break;

  // we must prevent the window from erasing the background each time a
  // repaint is needed
  case WM_ERASEBKGND:
    return (LRESULT)1;

  case WM_PAINT:
    // render the display
    window->renderer->render();
    break;

  case vpWM_CLOSEDISPLAY:
    // cleanup code here, if needed
    // destroys the window
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
vpWin32Window::vpWin32Window(vpWin32Renderer *rend) : initialized(false)
{
  renderer = rend;

  // registered is static member class and is initialized at the beginning of
  // this file (registered = false)

  // creates the semaphores
  semaInit = CreateSemaphore(nullptr, 0, 1, nullptr);
  semaClick = CreateSemaphore(nullptr, 0, MAX_SEM_COUNT, nullptr);
  semaClickUp = CreateSemaphore(nullptr, 0, MAX_SEM_COUNT, nullptr);
  semaKey = CreateSemaphore(nullptr, 0, MAX_SEM_COUNT, nullptr);
  semaMove = CreateSemaphore(nullptr, 0, MAX_SEM_COUNT, nullptr);
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
void vpWin32Window::initWindow(const char *title, int posx, int posy, unsigned int w, unsigned int h)
{
  // the window's style
  DWORD style = WS_OVERLAPPEDWINDOW | WS_VISIBLE;
  const char g_szClassName[] = "ViSPWindowClass";

  RECT rect;
  rect.left = 0;
  rect.right = static_cast<int>(w);
  rect.top = 0;
  rect.bottom = static_cast<int>(h);

  // now we register the window's class
  WNDCLASSEX wcex;

  wcex.cbSize = sizeof(WNDCLASSEX);

  wcex.style = CS_HREDRAW | CS_VREDRAW | CS_NOCLOSE;
  wcex.lpfnWndProc = (WNDPROC)WndProc;
  wcex.cbClsExtra = 0;
  wcex.cbWndExtra = 0;
  wcex.hInstance = hInst;
  wcex.hIcon = LoadIcon(nullptr, IDI_APPLICATION);
  wcex.hCursor = LoadCursor(nullptr, IDC_ARROW);
  wcex.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
  wcex.lpszMenuName = nullptr;
  wcex.lpszClassName = g_szClassName;
  wcex.hIconSm = LoadIcon(nullptr, IDI_APPLICATION);

  RegisterClassEx(&wcex);

  AdjustWindowRectEx(&rect, style, false, style);
  // std::cout << "win client size  (orig)(w,h): " << rect.left << " " <<
  // rect.top << " " << rect.right << " " << rect.bottom << std::endl;

  // creates the window
  hWnd = CreateWindowEx(WS_EX_APPWINDOW, g_szClassName, title, style, posx, posy, rect.right - rect.left,
                        rect.bottom - rect.top, nullptr, nullptr, hInst, nullptr);
  if (hWnd == nullptr) {
    DWORD err = GetLastError();
    std::cout << "err CreateWindowEx=" << err << std::endl;
    throw vpDisplayException(vpDisplayException::cannotOpenWindowError, "Can't create the window!");
  }
  SetWindowPos(hWnd, nullptr, 0, 0, rect.right - rect.left, rect.bottom - rect.top, SWP_NOZORDER | SWP_NOMOVE);

  // needed if we want to access it from the callback method (message handler)
  window = this;

  // initialize the renderer
  renderer->init(hWnd, w, h);

  // displays the window
  ShowWindow(hWnd, SW_SHOWDEFAULT);
  // ShowWindow(hWnd, SW_SHOW);
  UpdateWindow(hWnd);

  MSG msg;

  // starts the message loop
  while (true) {
    BOOL val = GetMessage(&msg, nullptr, 0, 0);
    if (val == -1) {
      std::cout << "GetMessage error:" << GetLastError() << std::endl;
      break;
    }
    else if (val == 0) {
      break;
    }
    else {
      if (!TranslateAccelerator(msg.hwnd, nullptr, &msg)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
      }
    }
  }
}

END_VISP_NAMESPACE

#endif
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_gui.a(vpWin32Window.cpp.o) has no symbols
void dummy_vpWin32Window() { };
#endif
