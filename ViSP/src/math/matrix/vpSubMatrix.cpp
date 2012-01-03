/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
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
 * Description:
 * Mask on a vpMatrix .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/

#include <visp/vpSubMatrix.h>
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>
#include <stdlib.h>

vpSubMatrix::vpSubMatrix(){
  data=NULL;
  parent=NULL;
  rowPtrs=NULL;
  rowNum=0;
  colNum=0;
  pRowNum=0;
  pColNum=0;
  dsize=0;
  trsize=0;
}

/*!
  \brief Constructor
  \param m : parent matrix
  \param row : row offset 
  \param col : col offset 
  \param nrows : number of rows of the sub matrix
  \param ncols : number of columns of the sub matrix
*/
vpSubMatrix::vpSubMatrix(vpMatrix &m, const unsigned int & row, const unsigned int &col , const unsigned int & nrows ,  const unsigned int & ncols){
  init(m,row,col,nrows,ncols);
}

/*!
  \brief Initialisation of a sub matrix
  \param m : parent matrix
  \param row : row offset 
  \param col : col offset 
  \param nrows : number of rows of the sub matrix
  \param ncols : number of columns of the sub matrix
*/
void vpSubMatrix::init(vpMatrix &m, const unsigned int & row, const unsigned int &col , const unsigned int & nrows ,  const unsigned int & ncols){
  
  if(! m.data){
    vpERROR_TRACE("\n\t\t SubMatrix parent matrix is not allocated") ;
    throw(vpMatrixException(vpMatrixException::subMatrixError,
			    "\n\t\t SubMatrix parent matrix is not allocated")) ;
  } 
  
  if(row+nrows <= m.getRows() && col+ncols <= m.getCols()){	
    data=m.data;
    parent =&m; 
    rowNum = nrows;
    colNum = ncols;
    pRowNum=m.getRows(); 
    pColNum=m.getCols(); 
    
    if(rowPtrs)
      free(rowPtrs);
    
    rowPtrs=(double**) malloc(nrows * sizeof(double*));
    for(unsigned int r=0;r<nrows;r++)
      rowPtrs[r]= m.data+col+(r+row)*pColNum;
    
    dsize = pRowNum*pColNum ;
    trsize =0 ;
  }else{
    vpERROR_TRACE("Submatrix cannot be contain in parent matrix") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,"Submatrix cannot be contain in parent matrix")) ;
  }
}

/*!
  \brief This method can be used to detect if the parent matrix 
   always exits or its size have not changed and  throw an exception is not
*/
void vpSubMatrix::checkParentStatus(){
  if(!data){
    vpERROR_TRACE("\n\t\t vpSubMatrix parent vpMatrix has been destroyed");
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix parent vpMatrix has been destroyed")) ;
  }
  if(pRowNum!=parent->getRows() || pColNum!=parent->getCols()){
    vpERROR_TRACE("\n\t\t vpSubMatrix size of parent vpMatrix has been changed");
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix size of parent vpMatrix has been changed")) ;
  }
}

/*!
  \brief Operation A = B
  \param B : a matrix
*/
vpSubMatrix & vpSubMatrix::operator=(const vpMatrix &B){
  
  if ((colNum != B.getCols())||(rowNum != B.getRows()))
  {
    vpERROR_TRACE("\n\t\t vpSubMatrix mismatch in operator vpSubMatrix=vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix mismatch in operator vpSubMatrix=vpMatrix")) ;
  }
  
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j] = B[i][j];
    
    return *this;
}

/*!
  \brief Operation A = B
  \param B : a subMatrix
*/
vpSubMatrix & vpSubMatrix::operator=(const vpSubMatrix &B){
  
  if ((colNum != B.getCols())||(rowNum != B.getRows()))
  {
    vpERROR_TRACE("\n\t\t vpSubMatrix mismatch in operator vpSubMatrix=vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix mismatch in operator vpSubMatrix=vpMatrix")) ;
  }
  
  
  double ** BrowPtrs=B.rowPtrs;
  
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j] = BrowPtrs[i][j];
    
    return *this;
}

/*!
  \brief Operation A = x
  \param x : a scalar
*/
vpSubMatrix & vpSubMatrix::operator=(const double &x){
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j] = x;
    
    return *this;
}


vpSubMatrix::~vpSubMatrix()
{
  data=NULL;
}
