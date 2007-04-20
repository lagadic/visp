/****************************************************************************
 *
 * $Id: vpException.cpp,v 1.4 2007-04-20 14:22:15 asaunier Exp $
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
