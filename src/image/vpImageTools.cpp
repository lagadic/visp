#include<visp/vpImageTools.h>


void   vpImageTools::filter(const vpImage<unsigned char> &I,
			    vpImage<double>& If,
			    const vpMatrix& M)
{

  int  size = M.getRows() ;
  int  half_size = size/2 ;


  int i, j ;
  If.resize(I.getRows(),I.getCols()) ;


  If = 0 ;

  for (i=half_size ; i < I.getRows()-half_size ; i++)
  {
    for (j=half_size ; j < I.getCols()-half_size ; j++)
    {
      double   conv_x = 0 ;

      int a,b ;
      for(a = 0 ; a < size ; a++ )
        for(b = 0 ; b < size ; b++ )
	{
	  double val =  I[i-half_size+a][j-half_size+b] ;
	  conv_x += M[a][b] * val ;
	}
      If[i][j] = conv_x ;
    }
  }

}

void
vpImageTools::filter(const vpImage<double> &I,
		     vpImage<double>& Ix,
		     vpImage<double>& Iy,
		     const vpMatrix& M)
{

  int  size = M.getRows() ;
  int  half_size = size/2 ;


  int i, j ;
  Ix.resize(I.getRows(),I.getCols()) ;
  Iy.resize(I.getRows(),I.getCols()) ;


  Ix = 0 ;
  Iy = 0 ;
  for (i=half_size ; i < I.getRows()-half_size ; i++)
  {
    for (j=half_size ; j < I.getCols()-half_size ; j++)
    {
      double   conv_x = 0 ;
      double   conv_y = 0 ;
      int a,b ;
      for(a = 0 ; a < size ; a++ )
        for(b = 0 ; b < size ; b++ )
	{
	  double val =  I[i-half_size+a][j-half_size+b] ;
	  conv_x += M[a][b] * val ;
	  conv_y += M[b][a] * val  ;
	}
      Ix[i][j] = conv_x ;
      Iy[i][j] = conv_y ;
    }
  }

}
