#include <stream>
#include <visp/vpRobust.h>

int
main()
{
  double sig = 0.5 ;

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
