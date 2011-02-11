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
 * Mask on a vpMatrix .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/

#ifndef __VP_SUB_MATRIX__
#define __VP_SUB_MATRIX__

#include <visp/vpMatrix.h>


/*!
  \file vpSubMatrix.h

  \brief Definition of the vpSubMatrix class
*/

/*!
  \class vpSubMatrix
  \ingroup vpMath
  \brief Definition of the vpSubMatrix
  vpSubMatrix class provides a mask on a vpMatrix
  all properties of vpMatrix are available with
  a vpSubMatrix

  \author Jean Laneurit (IRISA - INRIA Rennes)

  \sa vpMatrix vpColvector vpRowVector
*/
class VISP_EXPORT vpSubMatrix : public vpMatrix{

  private :
      //! Resize method unavailable
      void resize(const unsigned int /* r */ , const unsigned int /* c */ ){}
      //! Eye method unavailable
      void eye(unsigned int n);
      //! Eye method unavailable
      void eye(unsigned int m, unsigned int n);
      //! Copy constructor unavailable
      vpSubMatrix(const vpSubMatrix& /* m */);      
       
  protected :
 
      unsigned int pRowNum;
      unsigned int pColNum;
      vpMatrix *parent;
      
  public:

    //!Default constructor
    vpSubMatrix();
    //!Constructor
    vpSubMatrix(vpMatrix &m, const unsigned int & row, const unsigned int &col , const unsigned int & nrows ,  const unsigned int & ncols);
    //!Destructor
    ~vpSubMatrix();
    
    //! Initialisation of vpMatrix
    void init(vpMatrix &m, const unsigned int & row, const unsigned int &col , const unsigned int & nrows ,  const unsigned int & ncols);
    
    //!Check is parent vpRowVector has changed since initialization
    void checkParentStatus();
	
    //! Operation such as subA = subB
    vpSubMatrix & operator=(const vpSubMatrix &B);
    //! Operation such as subA = B
    vpSubMatrix & operator=(const vpMatrix &B);
    //! Operation such as subA = x
    vpSubMatrix & operator=(const double &x);
    
};

#endif
