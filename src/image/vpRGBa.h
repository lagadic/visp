
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

#include <visp/vpColor.h>


/*!
  \class vpRGBa

  \author Eric Marchand (Eric.Marchand@irisa.fr), Irisa / Inria Rennes


  Define the object vpRGBa that is used to build color
  images (it define a RGB 32 bits structure, fourth byte is not used)

  \sa CImage
*/
class vpRGBa : public vpColor
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
