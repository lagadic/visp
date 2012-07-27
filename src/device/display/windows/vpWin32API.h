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
 * GDI renderer for windows 32 display
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#include <windows.h>
#include <string>

#if ( defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9) )
#ifndef vpWin32API_HH
#define vpWin32API_HH

DWORD vpProcessErrors(const std::string &api_name);
void vpSelectObject(HWND hWnd, HDC hDC, HDC hDCMem, HGDIOBJ h);
void vpPrepareImageWithPen(CRITICAL_SECTION* CriticalSection, HWND hWnd,HBITMAP bmp,COLORREF color,unsigned int thickness, int style, HDC& hDCScreen,HDC& hDCMem,HPEN& hPen);
void vpEnterCriticalSection(LPCRITICAL_SECTION lpCriticalSection);
void vpLeaveCriticalSection(LPCRITICAL_SECTION lpCriticalSection);
BOOL vpReleaseSemaphore(HANDLE hSemaphore,LONG IReleaseCount,LPLONG lpPreviousCount);
BOOL vpLineTo(HDC hdc, int nXEnd, int nYEnd);
BOOL vpMoveToEx(HDC hdc, int X, int Y, LPPOINT lpPoint);
BOOL vpBitBlt(HDC hdcDest, int nXDest, int nYDest, int nWidth, int nHeight, HDC hdcSrc, int nXSrc, int nYSrc, DWORD dwRop);
BOOL vpInvalidateRect(HWND hWnd, const RECT *lpRect, BOOL bErase);
COLORREF vpSetPixel(HDC hdc, int X, int Y, COLORREF crColor);
HBITMAP vpCreateBitmap(int nWidth, int nHeight, UINT cPlanes, UINT cBitsPerPel, const VOID *lpvBits);
#endif
#endif
