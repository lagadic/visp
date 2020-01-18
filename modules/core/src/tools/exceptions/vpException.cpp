/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
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

#include "visp3/core/vpException.h"
#include <stdio.h>

/* -------------------------------------------------------------------------
 */
/* --- CONSTRUCTORS --------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */
vpException::vpException(int id) : code(id), message() {}

vpException::vpException(int id, const std::string &msg) : code(id), message(msg) {}

vpException::vpException(int id, const char *format, ...) : code(id), message()
{
  va_list args;
  va_start(args, format);
  setMessage(format, args);
  va_end(args);
}

vpException::vpException(const int id, const char *format, va_list args) : code(id), message()
{
  setMessage(format, args);
}
/* ------------------------------------------------------------------------ */
/* --- DESTRUCTORS -------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

/* Destructeur par default suffisant. */
// vpException::
// ~vpException (void)
// {
// }

void vpException::setMessage(const char *format, va_list args)
{
  char buffer[1024];
  vsnprintf(buffer, 1024, format, args);
  std::string msg(buffer);
  message = msg;
}

/* ------------------------------------------------------------------------ */
/* --- ACCESSORS ---------------------------------------------------------- */
/* ------------------------------------------------------------------------ */

const char *vpException::getMessage(void) const { return (this->message).c_str(); }

const std::string &vpException::getStringMessage(void) const { return this->message; }

int vpException::getCode(void) { return this->code; }

/*!
  Overloading of the what() method of std::exception to return the vpException
  message.

  \return pointer on the array of  \e char related to the error string.
*/
const char *vpException::what() const throw() { return (this->message).c_str(); }

/* -------------------------------------------------------------------------
 */
/* --- MODIFIORS -----------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

/* -------------------------------------------------------------------------
 */
/* --- OP << ---------------------------------------------------------------
 */
/* -------------------------------------------------------------------------
 */

VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpException &error)
{
  os << "Error [" << error.code << "]:\t" << error.message << std::endl;

  return os;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
