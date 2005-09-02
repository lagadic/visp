
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpBasicFeature.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpBasicFeature.cpp,v 1.2 2005-09-02 08:43:52 aremazei Exp $
 *
 * Description
 * ============
 *     class that defines what is a visual feature
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


#include <visp/vpBasicFeature.h>


/*!
  \file vpBasicFeature.cpp
  \brief class that defines what is a visual feature
*/

vpBasicFeature::vpBasicFeature()
{
    featureLine[0] = 0x1 ;
    featureLine[1] = 0x2 ;
    featureLine[2] = 0x4 ;
    featureLine[3] = 0x8 ;
    featureLine[4] = 0x10 ;
    featureLine[5] = 0x20 ;
    featureLine[6] = 0x40 ;
    featureLine[7] = 0x80 ;

    deallocate = vpBasicFeature::user ;
}

//! get the feature dimension
int
vpBasicFeature::getDimension(int select) const
{
    int dim = 0 ;
    for (int i=0 ; i < s.getRows() ; i++)
    {
	//	printf("%x %x %d \n",select, featureLine[i], featureLine[i] & select);
	if (featureLine[i] & select) dim +=1 ;
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
/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
