/****************************************************************************
 *
 * $Id$
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


#include <visp/vpSubColVector.h>
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>


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
vpSubColVector::vpSubColVector(vpColVector &v, const int & offset,const int & nrows){
  init(v,offset,nrows);
}

/*!
  \brief Initialisation of a the subColVector
  \param v : parent col vector
  \param offset : offset where subColVector start in the parent colVector
  \param nrows : size of the subColVector
*/
void vpSubColVector::init(vpColVector &v, const int & offset,const int & nrows){
  
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
    for(int i=0;i<nrows;i++)
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
  
  for (int i=0;i<rowNum;i++)
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
  
  for (int i=0;i<rowNum;i++)
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
  
  for (int i=0;i<rowNum;i++)
    data[i] = B[i][1];
  return *this;
}

/*!
  \brief Operation A = x 
  \param x : a scalar value
*/
vpSubColVector & vpSubColVector::operator=(const double &x){
  for (int i=0;i<rowNum;i++)
    data[i] = x;
  return *this;
}
