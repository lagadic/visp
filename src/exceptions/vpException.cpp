
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpException.cpp
 * Project:   ViSP2
 * Author:    Nicolas Mansard
 *
 * Version control
 * ===============
 *
 *  $Id: vpException.cpp,v 1.1.1.1 2005-06-08 07:08:09 fspindle Exp $
 *
 * Description
 * ============
 *  Handling exception
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/* \file vpException.cpp
   \brief error that can be emited by the vp class and its derivates
 */

#include "vpException.h"


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
	     const string & _msg)
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

const string &vpException::getStringMessage (void)
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

ostream &
operator << (ostream & os,
	     const vpException & error)
{
  os << "Error [" << error.code << "]:\t" << error.message << endl;

    return os;
}


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
