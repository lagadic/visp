/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Mask on a vpRowVector .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/

#ifndef __VP_SUB_ROW_VECTOR__
#define __VP_SUB_ROW_VECTOR__

#include <visp/vpRowVector.h>


/*!
  \file vpSubRowVector.h

  \brief Definition of the vpSubRowVector class
*/

/*!
  \class vpSubRowVector
  \ingroup vpMath
  \brief Definition of the vpSubRowVector
  vpSubRowVector class provides a mask on a vpRowVector
  all properties of vpRowVector are available with
  a vpSubRowVector

  \author Jean Laneurit (IRISA - INRIA Rennes)

  \sa vpMatrix vpColvector vpRowVector
*/

class VISP_EXPORT vpSubRowVector : public vpRowVector {

  private :
      //!Copy constructor unavaible
      vpSubRowVector(const vpSubRowVector& /* m */);
       
  protected :
 
      //!Number of row of parent vpColvector at initialization
      unsigned int pColNum;
      //!Parent vpColvector
      vpRowVector *parent;
      
  public:

    //!Default contructor
    vpSubRowVector();
    //!Contructor
    vpSubRowVector(vpRowVector &v, const unsigned int & offset,const unsigned int & ncols);
    //!Destructor
    ~vpSubRowVector();
    
    //! Initialisation of vpSubRowVector
    void init(vpRowVector &v, const unsigned int & offset,const unsigned int & ncols);
    
    //!Check is parent vpRowVector has changed since initialization
    void checkParentStatus();
      
    //! Operation such as subA = subB
    vpSubRowVector & operator=(const vpSubRowVector &B);
    //! Operation such as subA = B
    vpSubRowVector & operator=(const vpRowVector &B);
    //! Operation such as subA = matrix B with size of B(N,1)
    vpSubRowVector & operator=(const vpMatrix &B); 
    //! Operation such as subA = x
    vpSubRowVector & operator=(const double &x);
    
};

#endif
