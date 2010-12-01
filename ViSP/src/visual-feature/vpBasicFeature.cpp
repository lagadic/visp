/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Visual feature.
 *
 * Authors:
 * Eric Marchand
 * Nicolas Mansard
 *
 *****************************************************************************/



#include <visp/vpBasicFeature.h>

const int vpBasicFeature::FEATURE_LINE [32] =
    {
	1 <<  0,	1 <<  1,	1 <<  2,	1 <<  3,
	1 <<  4,	1 <<  5,	1 <<  6,	1 <<  7,
	1 <<  8,	1 <<  9,	1 << 10,	1 << 11,
	1 << 12,	1 << 13,	1 << 14,	1 << 15,
	1 << 16,	1 << 17,	1 << 18,	1 << 19,
	1 << 20,	1 << 21,	1 << 22,	1 << 23,
	1 << 24,	1 << 25,	1 << 26,	1 << 27,
	1 << 28,	1 << 29,	1 << 30,	1 << 31
    };
const int vpBasicFeature::FEATURE_ALL = 0xffff;

/*!
  \file vpBasicFeature.cpp
  \brief class that defines what is a visual feature
*/

vpBasicFeature::vpBasicFeature()
{
//     featureLine[0] = 0x1 ;
//     featureLine[1] = 0x2 ;
//     featureLine[2] = 0x4 ;
//     featureLine[3] = 0x8 ;
//     featureLine[4] = 0x10 ;
//     featureLine[5] = 0x20 ;
//     featureLine[6] = 0x40 ;
//     featureLine[7] = 0x80 ;
    //vpTRACE("0x%x", this);
    deallocate = vpBasicFeature::user ;
    flags = NULL;
}

//! get the feature dimension
int
vpBasicFeature::getDimension(int select) const
{
    int dim = 0 ;
    for (int i=0 ; i < s.getRows() ; i++)
    {
	//	printf("%x %x %d \n",select, featureLine[i], featureLine[i] & select);
	if (FEATURE_LINE[i] & select) dim +=1 ;
    }
    return dim ;
}

//! get the feature vecror
vpColVector
 vpBasicFeature::get_s() const
{
    vpColVector state  ; state = s ;
    return state ;
}

void vpBasicFeature::resetFlags()
{
  if (flags != NULL)
  {
    for (int i = 0; i < nbParameters; i++)
      flags[i] = false;
  }
}

//! set feature flags to true to prevent Warning when re-computing the interaction matrix without having updated the feature
void vpBasicFeature::setFlags()
{
  if (flags != NULL)
    {
	  for (int i = 0; i < nbParameters; i++)
	    flags[i] = true;
    }
}
/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
