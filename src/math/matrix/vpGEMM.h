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
 * Matrix generalized multiplication.
 *
 * Authors:
 * Laneurit Jean
 *
 *****************************************************************************/


#ifndef __VP_GEMM__
#define __VP_GEMM__

#include <visp/vpMatrix.h>
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>

const vpMatrix null(0,0);

/*!
  \brief Enumeration of the oeperation applied on matrix in vpGEMM function
  */
typedef enum {
  VP_GEMM_A_T=1,VP_GEMM_B_T=2,VP_GEMM_C_T=4,  /*!< GEMM method. */
} vpGEMMmethod;




template<int> inline void GEMMsize(const vpMatrix & A,const vpMatrix & B, int &Arows,  int &Acols, int &Brows,  int &Bcols){}

 template<> void inline GEMMsize<0>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getRows();
  Acols= A.getCols();
  Brows= B.getRows();
  Bcols= B.getCols();
}

 template<> inline void GEMMsize<1>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getCols();
  Acols= A.getRows();
  Brows= B.getRows();
  Bcols= B.getCols();
}
 template<> inline void GEMMsize<2>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getRows();
  Acols= A.getCols();
  Brows= B.getCols();
  Bcols= B.getRows();
}
 template<> inline void GEMMsize<3>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getCols();
  Acols= A.getRows();
  Brows= B.getCols();
  Bcols= B.getRows();
}

 template<> inline void GEMMsize<4>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getRows();
  Acols= A.getCols();
  Brows= B.getRows();
  Bcols= B.getCols();
}

 template<> inline void GEMMsize<5>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getCols();
  Acols= A.getRows();
  Brows= B.getRows();
  Bcols= B.getCols();
}

 template<> inline void GEMMsize<6>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getRows();
  Acols= A.getCols();
  Brows= B.getCols();
  Bcols= B.getRows();
}

 template<> inline void GEMMsize<7>(const vpMatrix & A,const vpMatrix & B, int &Arows, int &Acols,int &Brows, int &Bcols){
  Arows= A.getCols();
  Acols= A.getRows();
  Brows= B.getCols();
  Bcols= B.getRows();
}



 template<int> inline void GEMM1(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A, const vpMatrix & B, const double & alpha,vpMatrix &D){}

 template<> inline void GEMM1<0>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A, const vpMatrix & B, const double & alpha,vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[r][n]*B[n][c]*alpha;
      D[r][c]=sum;
    }
}

 template<> inline void GEMM1<1>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A, const vpMatrix & B, const double & alpha,vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[n][r]*B[n][c]*alpha;
      D[r][c]=sum;
    }
}

 template<> inline void GEMM1<2>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha,vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[r][n]*B[c][n]*alpha;
      D[r][c]=sum;
    }
}

 template<> inline void GEMM1<3>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha,vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[n][r]*B[c][n]*alpha;
      D[r][c]=sum;
    }
}

 template<int> inline void GEMM2(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){}

 template<> inline void GEMM2<0>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[r][n]*B[n][c]*alpha;
      D[r][c]=sum+C[r][c]*beta;
    } 
}

 template<> inline void GEMM2<1>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[n][r]*B[n][c]*alpha;
	D[r][c]=sum+C[r][c]*beta;
    }
}

 template<> inline void GEMM2<2>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[r][n]*B[c][n]*alpha;
      D[r][c]=sum+C[r][c]*beta;
    }
}

 template<> inline void GEMM2<3>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[n][r]*B[c][n]*alpha;
      D[r][c]=sum+C[r][c]*beta;
    } 
}


 template<> inline void GEMM2<4>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[r][n]*B[n][c]*alpha;
      D[r][c]=sum+C[c][r]*beta;
    }
}

 template<> inline void GEMM2<5>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[n][r]*B[n][c]*alpha;
	D[r][c]=sum+C[c][r]*beta;
    }
    
}

 template<> inline void GEMM2<6>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[r][n]*B[c][n]*alpha;
	D[r][c]=sum+C[c][r]*beta;
    } 
}

 template<> inline void GEMM2<7>(const int &Arows,const int &Brows, const int &Bcols, const vpMatrix & A,const vpMatrix & B, const double & alpha, const vpMatrix & C , const double &beta, vpMatrix &D){
  //vpMatrix &D = *dynamic_cast<double***>(Dptr);
  for(int r=0;r<Arows;r++)
    for(int c=0;c<Bcols;c++){
      double sum=0;
            for(int n=0;n<Brows;n++)
	sum+=A[n][r]*B[c][n]*alpha;
	D[r][c]=sum+C[c][r]*beta;
    }
}

 template<int T> inline  void vpTGEMM(const vpMatrix & A,const vpMatrix & B, const double & alpha ,const vpMatrix & C, const double & beta, vpMatrix & D){
  
  int Arows;
  int Acols;
  int Brows;
  int Bcols;
  
//   std::cout << T << std::endl;
  GEMMsize<T>(A,B,Arows,Acols,Brows,Bcols);
//   std::cout << Arows<<" " <<Acols << " "<< Brows << " "<< Bcols<<std::endl;
  
  try 
  {
    if ((Arows != D.getRows()) || (Bcols != D.getCols())) D.resize(Arows,Bcols);
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }
  
  if (Acols != Brows)
  {
    vpERROR_TRACE("\n\t\tvpMatrix mismatch size in vpGEMM") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,"\n\t\tvpMatrix mismatch size in vpGEMM")) ;
  }
  
  if(C.getRows()!=0 && C.getCols()!=0){

    if ((Arows != C.getRows()) || (Bcols != C.getCols()))
    {
      vpERROR_TRACE("\n\t\tvpMatrix mismatch size in vpGEMM") ;
      throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,"\n\t\tvpMatrix mismatch size in vpGEMM")) ;
    }
    
    
    GEMM2<T>(Arows,Brows,Bcols,A,B,alpha,C,beta,D);
  }else{
    GEMM1<T>(Arows,Brows,Bcols,A,B,alpha,D);
  }
  
}
/*!
  \brief This function performs generalized matrix multiplication: 
   D = alpha*op(A)*op(B) + beta*op(C), where op(X) is X or X^T 
   operation on A, B and C matrices is described by enumeration vpGEMMmethod
   for example to realised alpha*A^T*B^T+beta*C we need to called :
   vpGEMM(A,B,alpha,C,beta, VP_GEMM_A_T + VP_GEMM_B_T);
   If C is not used vpGEMM must be called  :
   vpGEMM(A,B,alpha,C, null,0);
   where null is a empty matrix
   
   \param A : a Matrix
   \param B : a Matrix
   \param alpha : a scalar
   \param C : a Matrix
   \param beta : a scalar
   \param ops : a scalar describing operation applied on the matrices
   
*/
     
inline void vpGEMM(const vpMatrix & A,const vpMatrix & B, const double & alpha ,const vpMatrix & C, const double & beta, vpMatrix & D, const int &ops=0){
  switch(ops){
    case 0 :
      vpTGEMM<0>( A, B,  alpha , C,  beta,  D);
      break;
    case 1 :
      vpTGEMM<1>( A, B,  alpha , C,  beta,  D);
      break;
    case 2 :
      vpTGEMM<2>( A, B,  alpha , C,  beta,  D);
      break;
    case 3 :
      vpTGEMM<3>( A, B,  alpha , C,  beta,  D);
      break;
    case 4 :
      vpTGEMM<4>( A, B,  alpha , C,  beta,  D);
      break;
    case 5 :
      vpTGEMM<5>( A, B,  alpha , C,  beta,  D);
      break;
    case 6 :
      vpTGEMM<6>( A, B,  alpha , C,  beta,  D);
      break;
    case 7 :
      vpTGEMM<7>( A, B,  alpha , C,  beta,  D);
      break;
    default:
      vpERROR_TRACE("\n\t\tvpMatrix mismatch operation in vpGEMM") ;
      throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,"\n\t\tvpMatrix mismatch operation in vpGEMM")) ;
      break;
  }
}

#endif
