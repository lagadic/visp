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
 * Test threading capabilities.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!

  \example testThread.cpp

  \brief Test threading capabilities.

*/

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))

//! [Code]
#include <iostream>

#include <visp3/core/vpThread.h>
#include <visp3/core/vpTime.h>

vpThread::Return myFooFunction(vpThread::Args args)
{
  (void)(args); // Avoid warning: unused parameter args
  // do stuff...
  return 0;
}

vpThread::Return myBarFunction(vpThread::Args args)
{
  (void)(args); // Avoid warning: unused parameter args
  // do stuff...
  return 0;
}

vpThread::Return myQuxFunction(vpThread::Args args)
{
  unsigned int args_ = *((unsigned int *)args);
  std::cout << "qux arg: " << args_ << std::endl;
  // do stuff...
  return 0;
}

int main()
{
  unsigned int qux_arg = 12;
  vpThread foo;
  vpThread bar((vpThread::Fn)myBarFunction);
  vpThread qux((vpThread::Fn)myQuxFunction,
               (vpThread::Args)&qux_arg); // Pass qux_arg to myQuxFunction() function

  vpTime::wait(1000); // Sleep 1s to ensure myQuxFunction() internal printings
  std::cout << "Joinable after construction:" << std::endl;
  std::cout << "foo: " << foo.joinable() << std::endl;
  std::cout << "bar: " << bar.joinable() << std::endl;
  std::cout << "qux: " << qux.joinable() << std::endl;

  foo.create((vpThread::Fn)myFooFunction);

  std::cout << "Joinable after creation:" << std::endl;
  std::cout << "foo: " << foo.joinable() << std::endl;
  std::cout << "bar: " << bar.joinable() << std::endl;
  std::cout << "qux: " << qux.joinable() << std::endl;

  if (foo.joinable())
    foo.join();
  if (bar.joinable())
    bar.join();
  if (qux.joinable())
    qux.join();

  std::cout << "Joinable after joining:" << std::endl;
  std::cout << "foo: " << foo.joinable() << std::endl;
  std::cout << "bar: " << bar.joinable() << std::endl;
  std::cout << "qux: " << qux.joinable() << std::endl;

  return 0;
}
//! [Code]

#else

#include <iostream>

int main()
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#endif
}
#endif
