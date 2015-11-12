/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
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
 *
 * Authors:
 * Bruno Renier
 * Anthony Saunier
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#if ( defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) )
#ifndef vpWin32Window_HH
#define vpWin32Window_HH


#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <windows.h>
#include <visp3/gui/vpWin32Renderer.h>
#include <visp3/core/vpGDIRenderer.h>
#include <visp3/core/vpDisplay.h>
#include <visp3/core/vpDisplayException.h>


//ViSP-defined messages for window's callback function
#define vpWM_GETCLICK WM_USER+1
#define vpWM_DISPLAY WM_USER+2
#define vpWM_GETCLICKUP WM_USER+3
#define vpWM_CLOSEDISPLAY WM_USER+4
#define vpWM_GETPOINTERMOTIONEVENT WM_USER+5
#define vpWM_DISPLAY_ROI WM_USER+6

//No specific mouse button query
#define vpNO_BUTTON_QUERY -1

class vpDisplayWin32;

class VISP_EXPORT vpWin32Window
{
private :

  HINSTANCE hInst;

  //! Window's handle
  HWND hWnd;

  //! Window is initialized
  bool initialized;
  //! Handle for the initialization semaphore
  HANDLE semaInit;

  //! Handle for the getClick semaphore
  HANDLE semaClick;
  //! Handle for the getClickUp semaphore
  HANDLE semaClickUp;
  //! Handle for the keyborad event semaphore
  HANDLE semaKey;
  //! Handle for the mouse event semaphore
  HANDLE semaMove;
  
  //! X coordinate of the click
  int clickX;
  int clickXUp; 
  //! Y coordinate of the click
  int clickY;
  int clickYUp; 
  //! X coordinate of the mouse
  int coordX;
  //! Y coordinate of the mouse
  int coordY;
  // Keyboard key
  char lpString[10];
  //! Button used for the click
  vpMouseButton::vpMouseButtonType clickButton;
  vpMouseButton::vpMouseButtonType clickButtonUp;

  //! True if the window's class has already been registered
  static bool registered;

  //! The renderer used by the window
  vpWin32Renderer * renderer;


 public:

  vpWin32Window(vpWin32Renderer * rend = NULL);
  virtual ~vpWin32Window();

  //! Returns the displayed image's width
  unsigned int getImageWidth(){ return renderer->getImageWidth(); }
  //! Returns the displayed image's height
  unsigned int getImageHeight(){ return renderer->getImageHeight(); }
  //! Returns the window's handle
  HWND getHWnd(){ return hWnd;}

  //! Returns true if the window is initialized
  bool isInitialized(){ return initialized; }

  //! Initialize the window
  void initWindow(const char* title, int posx, int posy, unsigned int w, unsigned int h);

  // Friend classes
  friend class vpDisplayWin32;
  friend class vpDisplayD3D;
  friend class vpDisplayGDI;

  //! The message loop
  friend LRESULT CALLBACK WndProc(HWND hWnd, UINT message,
				  WPARAM wParam, LPARAM lParam);
};

#endif
#endif
#endif
