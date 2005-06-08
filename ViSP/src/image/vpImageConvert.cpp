
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageConvert.cpp
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageConvert.cpp,v 1.1.1.1 2005-06-08 07:08:08 fspindle Exp $
 *
 * Description
 * ============
 *
 * Convert image types
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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
