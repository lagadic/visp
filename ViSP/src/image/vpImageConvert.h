
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpImageConvert.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpImageConvert.h,v 1.1.1.1 2005-06-08 07:08:08 fspindle Exp $
 *
 * Description
 * ============
 *
 * Convert image types
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

/*!
  \file vpImageConvert.h
  \brief Convert image types
*/

#ifndef vpIMAGECONVERT_H
#define vpIMAGECONVERT_H


// image
#include <visp/vpImage.h>

// color
#include <visp/vpRGBa.h>

/*!
  \class vpImageConvert.h
  \brief Convert image types
*/
class vpImageConvert
{

public:
  static void convert(const vpImage<unsigned char> &src,
		      vpImage<vpRGBa> & dest) ;
  static void convert(const vpImage<vpRGBa> &src,
		      vpImage<unsigned char> & dest) ;
} ;

#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
