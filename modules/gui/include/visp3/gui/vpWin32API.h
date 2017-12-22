/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * GDI renderer for windows 32 display
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#ifndef vpWin32API_HH
#define vpWin32API_HH

#include <visp3/core/vpConfig.h>

#if (defined(VISP_HAVE_GDI) || defined(VISP_HAVE_D3D9))

// Include WinSock2.h before windows.h to ensure that winsock.h is not
// included by windows.h since winsock.h and winsock2.h are incompatible
#include <WinSock2.h>
#include <string>
#include <windows.h>

DWORD vpProcessErrors(const std::string &api_name);
void vpSelectObject(HWND hWnd, HDC hDC, HDC hDCMem, HGDIOBJ h);
void vpPrepareImageWithPen(CRITICAL_SECTION *CriticalSection, HWND hWnd, HBITMAP bmp, COLORREF color,
                           unsigned int thickness, int style, HDC &hDCScreen, HDC &hDCMem, HPEN &hPen);
void vpEnterCriticalSection(LPCRITICAL_SECTION lpCriticalSection);
void vpLeaveCriticalSection(LPCRITICAL_SECTION lpCriticalSection);
BOOL vpReleaseSemaphore(HANDLE hSemaphore, LONG IReleaseCount, LPLONG lpPreviousCount);
BOOL vpLineTo(HDC hdc, int nXEnd, int nYEnd);
BOOL vpMoveToEx(HDC hdc, int X, int Y, LPPOINT lpPoint);
BOOL vpBitBlt(HDC hdcDest, int nXDest, int nYDest, int nWidth, int nHeight, HDC hdcSrc, int nXSrc, int nYSrc,
              DWORD dwRop);
BOOL vpInvalidateRect(HWND hWnd, const RECT *lpRect, BOOL bErase);
COLORREF vpSetPixel(HDC hdc, int X, int Y, COLORREF crColor);
HBITMAP vpCreateBitmap(int nWidth, int nHeight, UINT cPlanes, UINT cBitsPerPel, const VOID *lpvBits);
#endif
#endif
