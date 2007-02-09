/****************************************************************************
 *
 * $Id: exExample.cpp,v 1.1.1.1 2007-02-09 17:41:14 fspindle Exp $
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

/*!
  \file exExample.cpp
*/

/*!
  \class exExample

  \author Fabien Spindler

  \brief Class to show how to compile with ViSP library.

*/

#include <visp/vpDebug.h>

#include <example/exExample.h>

/*!

  Default constructor.

*/
exExample::exExample()
{
  vpTRACE("Call the constructor");
}

/*!

  Destructor.

*/
exExample::~exExample()
{
  vpTRACE("Call the destructor");
}

/*!

  Set a value.

  \param value The value to set.

  \sa getValue()
*/
void exExample::setValue(unsigned int value)
{
  this->value = value;
}

/*!

  Get a value.

  \return The value.

  \sa setValue()
*/
unsigned int exExample::getValue()
{
  return value;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
