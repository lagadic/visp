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
vpSubRowVector::vpSubRowVector(vpRowVector &v, const int & offset,const int & ncols){
    init(v,offset,ncols);
}

/*!
  \brief Initialisation of a the subRowVector
  \param v : parent row vector
  \param offset : offset where subRowVector start in the parent vector
  \param ncols : size of the subRowVector
*/
void vpSubRowVector::init(vpRowVector &v, const int & offset,const int & ncols){
  
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
	  delete [] rowPtrs;
	
	rowPtrs=new double*[1];
	for(int i=0;i<ncols;i++)
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
	
	for (int i=0;i<rowNum;i++)
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
	
	for (int i=0;i<rowNum;i++)
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
  
  	for (int i=0;i<rowNum;i++)
	    data[i] = B[i][1];
	return *this;
}
/*!
  \brief Operation A = x 
  \param x : a scalar value
*/
vpSubRowVector & vpSubRowVector::operator=(const double &x){
    	for (int i=0;i<rowNum;i++)
	    data[i] = x;
	return *this;
}
