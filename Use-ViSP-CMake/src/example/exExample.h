/****************************************************************************
 *
 * $Id: exExample.h,v 1.3 2007-05-10 11:43:03 fspindle Exp $
 *
 * Copyright (C) 2007 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP library.
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
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef exExample_h
#define exExample_h

#include "example/exConfig.h"

/*!
  \file exExample.h
*/
class EXAMPLE_EXPORT exExample
{
public:
  exExample();
  ~exExample();
  void setValue(unsigned int value);
  unsigned int getValue();

private:
  unsigned int value; // internal value
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
