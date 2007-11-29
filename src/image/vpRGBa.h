/****************************************************************************
 *
 * $Id: vpRGBa.h,v 1.3 2007-11-29 15:06:38 asaunier Exp $
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
 * RGBA pixel.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*
#----------------------------------------------------------------------------
#  Copyright (C) 1998  IRISA-INRIA Rennes Vista Project
#  All Rights Reserved.
#
#    Contact:
#       Eric Marchand
#       IRISA-INRIA Rennes
#       Campus Universitaire de Beaulieu
#       35042 Rennes Cedex
#       France
#
#    email: marchand@irisa.fr
#    www  : http://www.irisa.fr/vista
#
#----------------------------------------------------------------------------
*/


#ifndef vpRGBa_h
#define vpRGBa_h

/*!
  \file vpRGBa.h
  \brief Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used)
*/

#include <visp/vpConfig.h>
#include <visp/vpColor.h>


/*!
  \class vpRGBa

  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes


  Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used)

  \sa vpImage
*/
class VISP_EXPORT vpRGBa : public vpColor
{
public:
  unsigned char R ; //!< red component
  unsigned char G ; //!< green component
  unsigned char B ; //!< blue component
  unsigned char A ; //!<

  vpRGBa()  ;
  vpRGBa(unsigned char R1, unsigned char G1, unsigned char B1,
	 unsigned char A=0) ;
  vpRGBa(int color) ;
  vpRGBa(const vpRGBa &c) ;

  void  operator=(const unsigned char x) ;
  void  operator=(const vpRGBa &m) ;

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
