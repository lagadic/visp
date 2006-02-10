/*!
  \example test_matrix.cpp

  Test some vpMatrix functionalities.
*/



#include <visp/vpConfig.h>
#include <visp/vpDebug.h>


#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpTranslationVector.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpTwistMatrix.h>


int main()
{
  TRACE("--------------------------");
  TRACE("--- TEST vpTwistMatrix ---");
  TRACE("--------------------------");

  // Set the translation
  vpTranslationVector cte;
  cte[0] = 1.;
  cte[1] = 0.5;
  cte[2] = -1.;

  // Set the rotation
  vpRxyzVector cre;
  cre[0] =  M_PI/2.;
  cre[1] = -M_PI/2.;
  cre[2] = -M_PI/4.;

  // Build rotation matrix
  vpRotationMatrix cRe(cre);

  // Build the twist matrix
  vpTwistMatrix cVe(cte, cRe);

  TRACE("cVe twist matrix:");
  cVe.print (std::cout, 6);


  // Set a speed skew
  vpColVector ev(6);

  ev[0] = 1.;
  ev[1] = 0.1;
  ev[2] = -0.5;
  ev[3] = M_PI/180.;
  ev[4] = M_PI/18.;
  ev[5] = M_PI/10.;

  TRACE("ev colvector:");
  ev.print (std::cout, 6);

  // Set a speed skew
  vpColVector cv;

  cv = cVe * ev;

  TRACE("cv = cVe * ev:");
  cv.print (std::cout, 6);

}
