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
 * Mask on a vpRowVector .
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/


#include <visp/vpSubRowVector.h>
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>
#include <stdlib.h>

    vpSubRowVector::vpSubRowVector(){
       data=NULL;
       parent=NULL;
       rowPtrs=NULL;
       rowNum=0;
       colNum=0;
       pColNum=0;
       dsize=0;
       trsize=0;
    }

/*!
  \brief Constructor
  \param v : parent row vector
  \param offset : offset where subRowVector start in the parent vector
  \param ncols : size of the subRowVector
*/
vpSubRowVector::vpSubRowVector(vpRowVector &v, const unsigned int & offset,const unsigned int & ncols){
    init(v,offset,ncols);
}

/*!
  \brief Initialisation of a the subRowVector
  \param v : parent row vector
  \param offset : offset where subRowVector start in the parent vector
  \param ncols : size of the subRowVector
*/
void vpSubRowVector::init(vpRowVector &v, const unsigned int & offset,const unsigned int & ncols){
  
  if(!v.data){
      vpERROR_TRACE("\n\t\t vpSubColvector parent vpRowVector has been destroyed");
      throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
		    "\n\t\t \n\t\t vpSubColvector parent vpRowVector has been destroyed")) ;
  }
  
  if(offset+ncols<=v.getCols()){
	data=v.data+offset;
	  
	rowNum=1;
	colNum = ncols;
	
	pColNum=v.getCols();
	parent=&v;
	
	if(rowPtrs)
	  free(rowPtrs);
	
	rowPtrs=(double**) malloc(1 * sizeof(double*));
	for(unsigned int i=0;i<1;i++)
	  rowPtrs[i]=v.data+i+offset;
	
	dsize = colNum ;
	trsize =0 ;
  }else{
    	vpERROR_TRACE("SubRowVector cannot be contain in parent RowVector") ;
	throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,"SubRowVector cannot be contain in parent RowVector")) ;
  }
}

vpSubRowVector::~vpSubRowVector(){
  data=NULL ;
}

/*!
  \brief This method can be used to detect if the parent rowVector 
   always exits or its size have not changed and throw an exception is not
*/
void vpSubRowVector::checkParentStatus(){
      if(!data){
	vpERROR_TRACE("\n\t\t vpSubColvector parent vpRowVector has been destroyed");
	throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
		    "\n\t\t \n\t\t vpSubColvector parent vpRowVector has been destroyed")) ;
      }
      if(pColNum!=parent->getCols()){
	vpERROR_TRACE("\n\t\t vpSubColvector size of parent vpRowVector has been changed");
	throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
		    "\n\t\t \n\t\t vpSubColvector size of parent vpRowVector has been changed")) ;
     }
}

/*!
  \brief Operation A = B
  \param B : a subRowvector
*/
vpSubRowVector & vpSubRowVector::operator=(const vpSubRowVector &B){
  
	if ( colNum != B.getCols())
	{
		vpERROR_TRACE("\n\t\t vpSubRowVector mismatch in operator vpSubRowVector=vpSubRowVector") ;
		throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix mismatch in operator vpSubRowVector=vpSubRowVector")) ;
	}
	
	for (unsigned int i=0;i<rowNum;i++)
	    data[i] = B[i];
	
	return *this;
}

/*!
  \brief Operation A = B
  \param B : a rowVector
*/
vpSubRowVector & vpSubRowVector::operator=(const vpRowVector &B){
	if ( colNum != B.getCols())
	{
		vpERROR_TRACE("\n\t\t vpSubRowVector mismatch in operator vpSubRowVector=vpRowVector") ;
		throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix mismatch in operator vpSubRowVector=vpRowVector")) ;
	}
	
	for (unsigned int i=0;i<rowNum;i++)
	    data[i] = B[i];
	
	return *this;
}

/*!
  \brief Operation A = B 
  \param B : a vpMatrix of size 1 x ncols
*/
vpSubRowVector & vpSubRowVector::operator=(const vpMatrix &B){
        if ((B.getRows()!=1)||(colNum != B.getCols()))
	{
		vpERROR_TRACE("\n\t\t vpSubRowVector mismatch in operator vpSubRowVector=vpMatrix") ;
		throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubMatrix mismatch in operator vpSubRowVector=vpMatrix")) ;
	}
  
  	for (unsigned int i=0;i<rowNum;i++)
	    data[i] = B[i][1];
	return *this;
}
/*!
  \brief Operation A = x 
  \param x : a scalar value
*/
vpSubRowVector & vpSubRowVector::operator=(const double &x){
    	for (unsigned int i=0;i<rowNum;i++)
	    data[i] = x;
	return *this;
}
