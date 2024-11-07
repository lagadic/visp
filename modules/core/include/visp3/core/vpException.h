/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
 * \file vpException.h
 * \brief error that can be emitted by the vp class and its derivatives
 */

#ifndef VP_EXCEPTION_H
#define VP_EXCEPTION_H

#include <visp3/core/vpConfig.h>

#include <iostream>
#include <stdarg.h>
#include <string>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpException
 * \ingroup group_core_debug
 * \brief error that can be emitted by ViSP classes.
 *
 * This class inherits from the standard std::exception contained in the C++
 * STL.
 * It is therefore possible to catch vpException with any other derivative of
 * std::exception in the same catch.
*/
class VISP_EXPORT vpException : public std::exception
{
public:
  enum generalExceptionEnum
  {
    memoryAllocationError,       //!< Memory allocation error
    memoryFreeError,             //!< Memory free error
    functionNotImplementedError, //!< Function not implemented
    ioError,                     //!< I/O error
    cannotUseConstructorError,   //!< constructor error
    notImplementedError,         //!< Not implemented
    divideByZeroError,           //!< Division by zero
    dimensionError,              //!< Bad dimension
    fatalError,                  //!< Fatal error
    badValue,                    //!< Used to indicate that a value is not in the allowed range.
    notInitialized               //!< Used to indicate that a parameter is not initialized.
  };

  /*!
   * Constructor.
   */
  vpException(int code, const char *format, va_list args);
  /*!
   * Constructor.
   */
  vpException(int code, const char *format, ...);

  /*!
   * Constructor.
   */
  vpException(int code, const std::string &msg);

  /*!
    Basic destructor. Do nothing but implemented to fit the inheritance from
    std::exception
  */
#if (VISP_CXX_STANDARD == VISP_CXX_STANDARD_98)
  virtual ~vpException() throw() { }
#endif
  /*!
   * Constructor.
   */
  VP_EXPLICIT vpException(int code);

  /** @name Inherited functionalities from vpException */
  //@{
  /*!
   * Send the object code.
   */
  int getCode() const;

  /*!
   * Send a reference (constant) related the error message (can be empty).
   */
  const std::string &getStringMessage() const;

  /*!
   * Send a pointer on the array of  \e char related to the error string.
   * Cannot be  \e nullptr.
   */
  const char *getMessage() const;

  /*!
   * Overloading of the what() method of std::exception to return the vpException
   * message.
   *
   * \return pointer on the array of  \e char related to the error string.
   */
  const char *what() const throw();
  //@}

  /*!
   * Print the error structure.
   */
  friend VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpException &art);

protected:
  //! Contains the error code, see the errorCodeEnum table for details.
  int code;

  //! Contains an error message (can be empty)
  std::string message;

  //! Set the message container
  void setMessage(const char *format, va_list args);

  //!  forbid the empty constructor (protected)
  vpException() : code(notInitialized), message("") { }

};
END_VISP_NAMESPACE
#endif
