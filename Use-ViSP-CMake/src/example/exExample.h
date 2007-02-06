/****************************************************************************
 *
 * $Id: exExample.h,v 1.1.1.1 2007-02-06 10:47:23 fspindle Exp $
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
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Author:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef exExample_h
#define exExample_h


/*!
  \file exExample.h
*/
class exExample
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
