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

  \example testMutex.cpp

  \brief Test mutexes and threading capabilities.

*/

#include <iostream>

#include <visp3/core/vpMutex.h>
#include <visp3/core/vpThread.h>

#if defined(VISP_HAVE_PTHREAD) || (defined(_WIN32) && !defined(WINRT_8_0))

int thread_counter = 0;
vpMutex mutex;

vpThread::Return doSomeThing(vpThread::Args args)
{
  mutex.lock();
  unsigned int thread_id = *((unsigned int *)args);

  std::cout << "Started job " << thread_counter << " with id " << thread_id << std::endl;

  for (unsigned long i = 0; i < (0xFFFF); i++) {
  };

  std::cout << "Ended job " << thread_counter << std::endl;

  thread_counter++;
  mutex.unlock();

  return 0;
}

int main(void)
{
  unsigned int nthread = 10;
  vpThread *thread = new vpThread[nthread];
  unsigned int *thread_id = new unsigned int[nthread];

  for (unsigned int i = 0; i < nthread; i++) {
    thread_id[i] = i;
    thread[i].create((vpThread::Fn)&doSomeThing, (vpThread::Args)&thread_id[i]);
  }

  delete[] thread;
  delete[] thread_id;

  return 0;
}

#else
int main()
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  std::cout << "You should enable pthread usage and rebuild ViSP..." << std::endl;
#else
  std::cout << "Multi-threading seems not supported on this platform" << std::endl;
#endif
}
#endif
