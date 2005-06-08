#include<iostream>
#include<visp/vpRobust.h>
#include <fstream>

int
main()
{
  double sig = 1 ;

  double w ;
  ofstream f("/tmp/w.dat") ;
  double x = -10 ;
  while (x<10)
  {
    if (fabs(x/sig)<=(4.6851))
    {
      w = vpMath::sqr(1-vpMath::sqr(x/(sig*4.6851)));
    }
    else
    {
      w = 0;
    }
    f << x <<"  "<<w <<endl ;
    x+= 0.01 ;
  }
}
/*
  set data style line
set ylabel "weight"
set yr [0:1.19]
set xlabel "Normalized residuals"
plot 'w.dat' title "Tukey Estimator" lw 2, 1 title "Least-Squares" lw 2

set term post eps
set output "tukey.eps"
replot

*/
