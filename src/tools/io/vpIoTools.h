
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMatrixException.h
 * Project:   ViSP2
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpIoTools.h,v 1.2 2006-04-19 09:01:22 fspindle Exp $
 *
 * Description
 * ============
 * io basic tools
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#ifndef vpIoTools_HH
#define vpIoTools_HH

/*!
  \file vpIoTools.h
  \brief io basic tools
*/

/*!
  \class vpIoTools
  \brief io basic tools

*/

class vpIoTools
{

public:
  static void checkDirectory(const char *dir );
  static void makeDirectory(const  char *dir );
} ;


#endif
