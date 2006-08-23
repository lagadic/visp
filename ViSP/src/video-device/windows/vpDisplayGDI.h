/****************************************************************************
 *
 * $Id: vpDisplayGDI.h,v 1.3 2006-08-23 12:08:17 brenier Exp $
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
 * Windows 32 display using GDI
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/
#include <visp/vpConfig.h>

#if ( defined(WIN32) ) 

#ifndef vpDisplayGDI_HH
#define vpDisplayGDI_HH

#include <visp/vpDisplayWin32.h>

/*!
  \class vpDisplayGDI

  \brief Display for windows using GDI (available on any windows 32 platform).
  GDI stands for Graphics Device Interface and is a core component of Microsoft
  Windows operating systems used for displaying graphics in a window.

  \author Bruno Renier
*/
class VISP_EXPORT vpDisplayGDI : public vpDisplayWin32
{
public:
	

  vpDisplayGDI();
  
  vpDisplayGDI(vpImage<vpRGBa> &I,
	       int winx=-1, int winy=-1,
	       char *_title=NULL);
  
  
  vpDisplayGDI(vpImage<unsigned char> &I,
	       int winx=-1, int winy=-1,
	       char *_title=NULL);
  
  ~vpDisplayGDI();
  
};

#endif
#endif

