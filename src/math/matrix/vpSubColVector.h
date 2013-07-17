/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Mask on a vpColVector .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/

#ifndef __VP_SUB_COL_VECTOR__
#define __VP_SUB_COL_VECTOR__

#include <visp/vpColVector.h>

/*!
  \file vpSubColVector.h

  \brief Definition of the vpSubColVector class
*/

/*!
  \class vpSubColVector
  \ingroup vpMath
  \brief Definition of the vpSubColVector
  vpSubColVector class provides a mask on a vpColVector
  all properties of vpColVector are available with
  a vpSubColVector

  \author Jean Laneurit (IRISA - INRIA Rennes)

  \sa vpMatrix vpColvector vpRowVector
*/
class VISP_EXPORT vpSubColVector : public vpColVector {

  private :
      //!Copy constructor
      vpSubColVector(const vpSubColVector& /* m */);      
       
  protected :
 
      //!Number of row of parent vpColvector at initialization
      unsigned int pRowNum;
      //!Parent vpColvector
      vpColVector *parent;
      
  public:

    //!Default constructor
    vpSubColVector();
    //!Constructor
    vpSubColVector(vpColVector &v, const unsigned int & offset,const unsigned int & nrows);
    //!Destructor
    ~vpSubColVector();
    
    //! Initialisation of vpSubColVector
    void init(vpColVector &v, const unsigned int & offset,const unsigned int & nrows);
    
    //!Check is partent vpColVector has changed since initialization
    void checkParentStatus();
      
    //! Operation such as subA = subB
    vpSubColVector & operator=(const vpSubColVector &B);
    //! Operation such as subA = B
    vpSubColVector & operator=(const vpColVector &B);
    //! Operation such as subA = matrix B with size of B(N,1)
    vpSubColVector & operator=(const vpMatrix &B); 
    //! Operation such as subA = x
    vpSubColVector & operator=(const double &x);
        
};

#endif
