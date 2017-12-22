/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

/* \file vpException.h
   \brief error that can be emited by the vp class and its derivates
 */

#ifndef __vpException_H
#define __vpException_H

/* --------------------------------------------------------------------- */
/* --- INCLUDE --------------------------------------------------------- */
/* --------------------------------------------------------------------- */

#include <visp3/core/vpConfig.h>

/* Classes standards. */
#include <iostream> /* Classe std::ostream.    */
#include <stdarg.h>
#include <string> /* Classe string.     */

/* --------------------------------------------------------------------- */
/* --- CLASS ----------------------------------------------------------- */
/* --------------------------------------------------------------------- */

/*!
   \class vpException
   \ingroup group_core_debug
   \brief error that can be emited by ViSP classes.

   This class inherites from the standard std::exception contained in the C++
   STL.
   It is therefore possible to catch vpException with any other derivative of
   std::exception in the same catch.
 */
class VISP_EXPORT vpException : public std::exception
{
protected:
  //! Contains the error code, see the errorCodeEnum table for details.
  int code;

  //! Contains an error message (can be empty)
  std::string message;

  //! Set the message container
  void setMessage(const char *format, va_list args);

  //!  forbid the empty constructor (protected)
  vpException() : code(notInitialized), message(""){};

public:
  enum generalExceptionEnum {
    memoryAllocationError,
    memoryFreeError,
    functionNotImplementedError,
    ioError,
    cannotUseConstructorError,
    notImplementedError,
    divideByZeroError,
    dimensionError,
    fatalError,
    badValue,      /*!< Used to indicate that a value is not in the allowed range.
                    */
    notInitialized /*!< Used to indicate that a parameter is not initialized.
                    */
  };

  vpException(const int code, const char *format, va_list args);
  vpException(const int code, const char *format, ...);
  vpException(const int code, const std::string &msg);
  explicit vpException(const int code);

  /*!
    Basic destructor. Do nothing but implemented to fit the inheritance from
    std::exception
  */
  virtual ~vpException() throw() {}

  /** @name Inherited functionalities from vpException */
  //@{
  //! Send the object code.
  int getCode(void);

  //! Send a reference (constant) related the error message (can be empty).
  const std::string &getStringMessage(void) const;
  //! send a pointer on the array of  \e char related to the error string.
  //! Cannot be  \e NULL.
  const char *getMessage(void) const;
  //@}

  //! Print the error structure.
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpException &art);

  const char *what() const throw();
};

#endif /* #ifndef __vpException_H */
