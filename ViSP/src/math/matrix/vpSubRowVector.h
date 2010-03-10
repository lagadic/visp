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

class vpSubRowVector : public vpRowVector {

  private :
      
      //! Resize method unavaible
      void resize(const int r , const int c ){}
      //!Copy constructor unavaible
      vpSubRowVector(const vpSubRowVector& m){}      
       
  protected :
 
      //!Number of row of parent vpColvector at initialization
      int pColNum;
      //!Parent vpColvector
      vpRowVector *parent;
      
  public:

    //!Default contructor
    vpSubRowVector();
    //!Contructor
    vpSubRowVector(vpRowVector &v, const int & offset,const int & ncols);
    //!Destructor
    ~vpSubRowVector();
    
    //! Initialisation of vpSubRowVector
    void init(vpRowVector &v, const int & offset,const int & ncols);
    
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