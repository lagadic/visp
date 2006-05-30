/****************************************************************************
 *
 * $Id: vpImageConvert.cpp,v 1.2 2006-05-30 08:40:43 fspindle Exp $
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
 * Convert image types.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpImageConvert.cpp
  \brief Convert image types
*/



// image
#include <visp/vpImageConvert.h>

void
vpImageConvert::convert(const vpImage<unsigned char> &src,
			vpImage<vpRGBa> & dest)
{
  dest.resize(src.getRows(), src.getCols()) ;

  for (int i =0 ;i < src.getRows()* src.getCols() ; i++)
    {
      unsigned char p =  *(src.bitmap + i) ;
      (dest.bitmap + i)->R =   p ;
      (dest.bitmap + i)->G =   p ;
      (dest.bitmap + i)->B =   p ;
    }

}

void
vpImageConvert::convert(const vpImage<vpRGBa> &src,
			vpImage<unsigned char> & dest)
{
  dest.resize(src.getRows(), src.getCols()) ;

  for (int i =0 ;i < src.getRows()* src.getCols() ; i++)
  {
    vpRGBa p =  *(src.bitmap + i) ;
    *(dest.bitmap + i) =
      (unsigned char)( 0.299 * p.R + 0.597 * p.G + 0.114 * p.B) ;
  }

}



/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
