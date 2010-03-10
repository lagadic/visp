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
class vpSubMatrix : public vpMatrix{

  private :
      //! Resize method unavailable
      void resize(const int r , const int c ){}
      //! Eye method unavailable
      void eye(int n);
      //! Eye method unavailable
      void eye(int m, int n);
      //! Copy constructor unavailable
      vpSubMatrix(const vpSubMatrix& m){}      
       
  protected :
 
      int pRowNum;
      int pColNum;
      vpMatrix *parent;
      
  public:

    //!Default constructor
    vpSubMatrix();
    //!Constructor
    vpSubMatrix(vpMatrix &m, const int & row, const int &col , const int & nrows ,  const int & ncols);
    //!Destructor
    ~vpSubMatrix();
    
    //! Initialisation of vpMatrix
    void init(vpMatrix &m, const int & row, const int &col , const int & nrows ,  const int & ncols);
    
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