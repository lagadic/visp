/****************************************************************************
 *
 * $Id: vpDisplayD3D.h,v 1.2 2007-02-26 17:26:45 fspindle Exp $
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
 * Windows 32 display using D3D
 *
 * Authors:
 * Bruno Renier
 *
 *****************************************************************************/

#include <visp/vpConfig.h>
#if ( defined(VISP_HAVE_D3D9) ) 

#ifndef VPDISPLAYD3D_HH
#define VPDISPLAYD3D_HH


#include <visp/vpDisplayWin32.h>

/*!
  \class vpDisplayD3D

  \brief Display for windows using Direct3D.
  Direct3D is part of the DirectX API available under Windows operating systems.

  \warning Requires DirectX9 SDK to compile and DirectX9 DLLs to run.
*/
class VISP_EXPORT vpDisplayD3D : public vpDisplayWin32
{
public:
  vpDisplayD3D();
  
  vpDisplayD3D(const vpImage<vpRGBa> &I,
		    int winx=-1, int winy=-1,
		    char *_title=NULL);
  
  vpDisplayD3D(const vpImage<unsigned char> &I,
		    int winx=-1, int winy=-1,
		    char *_title=NULL);
  
  ~vpDisplayD3D();
  
};
#endif
#endif
