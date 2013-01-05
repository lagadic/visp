/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Exception handling.
 *
 * Authors:
 * Nicolas Mansard
 *
 *****************************************************************************/


/* \file vpException.cpp
   \brief error that can be emited by the vp class and its derivates
 */

#include "visp/vpException.h"


/* ------------------------------------------------------------------------- */
/* --- CONSTRUCTORS -------------------------------------------------------- */
/* ------------------------------------------------------------------------- */


vpException::vpException (int _code)
    :
    code (_code),
    message ()

{
    return ;
}


vpException::vpException (int _code,
	     const std::string & _msg)
    :
    code (_code),
    message (_msg)

{
    return ;
}


vpException::vpException (int _code,
	     const char * _msg)
    :
    code (_code),
    message (_msg)
{
    return ;
}

/* ------------------------------------------------------------------------ */
/* --- DESTRUCTORS -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* Destructeur par default suffisant. */
// vpException::
// ~vpException (void)
// {
// }

/* ------------------------------------------------------------------------ */
/* --- ACCESSORS ---------------------------------------------------------- */
/* ------------------------------------------------------------------------ */


const char *vpException::getMessage (void)
{
    return (this->message) .c_str();
}

const std::string &vpException::getStringMessage (void)
{
    return this->message;
}

int
vpException::getCode (void)
{
    return this->code;
}

/*!
  Overloading of the what() method of std::exception to return the vpException
  message.
  
  \return pointer on the array of  \e char related to the error string.
*/
const char* vpException::what () const throw()
{
  return (this->message) .c_str();
}


/* ------------------------------------------------------------------------- */
/* --- MODIFIORS ----------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* --- OP << --------------------------------------------------------------- */
/* ------------------------------------------------------------------------- */

std::ostream &
operator << (std::ostream & os,
	     const vpException & error)
{
  os << "Error [" << error.code << "]:\t" << error.message << std::endl;

    return os;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
