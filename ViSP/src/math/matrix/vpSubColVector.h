/****************************************************************************
 *
 * $Id $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
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
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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
class vpSubColVector : public vpColVector {

  private :
      
      //! Resize method do nothing
      void resize(const int r , const int c ){}
      //!Copy constructor
      vpSubColVector(const vpSubColVector& m){}      
       
  protected :
 
      //!Number of row of parent vpColvector at initialization
      int pRowNum;
      //!Parent vpColvector
      vpColVector *parent;
      
  public:

    //!Default contructor
    vpSubColVector();
    //!Contructor
    vpSubColVector(vpColVector &v, const int & offset,const int & nrows);
    //!Destructor
    ~vpSubColVector();
    
    //! Initialisation of vpSubColVector
    void init(vpColVector &v, const int & offset,const int & nrows);
    
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