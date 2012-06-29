/****************************************************************************
 *
 * $Id$
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
 *
 * Description:
 * Mask on a vpColVector .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/


#include <visp/vpSubColVector.h>
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>
#include <stdlib.h>

vpSubColVector::vpSubColVector(){
   data=NULL;
   parent=NULL;
   rowPtrs=NULL;
   rowNum=0;
   colNum=0;
   pRowNum=0;
   dsize=0;
   trsize=0;
}

/*!
  \brief Constructor
  \param v : parent col vector
  \param offset : offset where subColVector start in the parent colVector
  \param nrows : size of the subColVector
*/
vpSubColVector::vpSubColVector(vpColVector &v, const unsigned int & offset,const unsigned int & nrows){
  init(v,offset,nrows);
}

/*!
  \brief Initialisation of a the subColVector
  \param v : parent col vector
  \param offset : offset where subColVector start in the parent colVector
  \param nrows : size of the subColVector
*/
void vpSubColVector::init(vpColVector &v, 
			  const unsigned int & offset, 
			  const unsigned int & nrows){
  
  if(!v.data){
    vpERROR_TRACE("\n\t\t vpSubColvector parent vpColVector has been destroyed");
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubColvector parent vpColVector has been destroyed")) ;
  }
  
  if(offset+nrows<=v.getRows()){
    data=v.data+offset;
    
    rowNum=nrows;
    colNum = 1;
    
    pRowNum=v.getRows();
    parent=&v;
    
    if(rowPtrs){
      free(rowPtrs);
    }
    
  rowPtrs=(double**)malloc( parent->getRows() * sizeof(double*));
    for(unsigned int i=0;i<nrows;i++)
      rowPtrs[i]=v.data+i+offset;
    
    dsize = rowNum ;
    trsize =0 ;
  }else{
    vpERROR_TRACE("SubColVector cannot be contain in parent ColVector") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,"SubColVector cannot be contain in parent ColVector")) ;
  }
}

vpSubColVector::~vpSubColVector(){
  data=NULL ;
}


/*!
  \brief This method can be used to detect if the parent colVector
  always exits or its size have not changed and throw an exception is not
*/
void vpSubColVector::checkParentStatus(){
  if(!data){
    vpERROR_TRACE("\n\t\t vpSubColvector parent vpColVector has been destroyed");
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubColvector parent vpColVector has been destroyed")) ;
  }
  if(pRowNum!=parent->getRows()){
    vpERROR_TRACE("\n\t\t vpSubColvector size of parent vpColVector has been changed");
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,"\n\t\t \n\t\t vpSubColvector size of parent vpColVector has been changed")) ;
  }
}

/*!
  \brief Operation A = B
  \param B : a subColvector
*/
vpSubColVector & vpSubColVector::operator=(const vpSubColVector &B){
  
  if ( rowNum != B.getRows())
  {
    vpERROR_TRACE("\n\t\t vpSubColVector mismatch in operator vpSubColVector=vpSubColVector") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix mismatch in operator vpSubColVector=vpSubColVector")) ;
  }
  
  for (unsigned int i=0;i<rowNum;i++)
    data[i] = B[i];
  return *this;
}

/*!
  \brief Operation A = B
  \param B : a rowVector
*/
vpSubColVector & vpSubColVector::operator=(const vpColVector &B){
  if ( rowNum != B.getRows())
  {
    vpERROR_TRACE("\n\t\t vpSubColVector mismatch in operator vpSubColVector=vpColVector") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubColVector mismatch in operator vpSubColVector=vpColVector")) ;
  }
  
  for (unsigned int i=0;i<rowNum;i++)
    data[i] = B[i];
  
  return *this;
}

/*!
  \brief Operation A = B 
  \param B : a vpMatrix of size nrow x 1
*/
vpSubColVector & vpSubColVector::operator=(const vpMatrix &B){
  if ((B.getCols()!=1)||(rowNum != B.getRows()))
  {
    vpERROR_TRACE("\n\t\t vpSubColVector mismatch in operator vpSubColVector=vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubColVector mismatch in operator vpSubColVector=vpMatrix")) ;
  }
  
  for (unsigned int i=0;i<rowNum;i++)
    data[i] = B[i][1];
  return *this;
}

/*!
  \brief Operation A = x 
  \param x : a scalar value
*/
vpSubColVector & vpSubColVector::operator=(const double &x){
  for (unsigned int i=0;i<rowNum;i++)
    data[i] = x;
  return *this;
}
