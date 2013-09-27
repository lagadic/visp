#include <iostream>
#include <visp/vpMath.h>
#include <visp/vpRotationMatrix.h>
#include <visp/vpThetaUVector.h>
#include <limits>

int main()
{
  try {
    vpThetaUVector tu;

    // Construct a rotation matrix from the theta U angles
    vpRotationMatrix R(vpMath::rad(0.),vpMath::rad(180)+100*std::numeric_limits<double>::epsilon(),0.);

    // Extract the theta U angles from a rotation matrix
    tu.buildFrom(R);

    // Since the rotation vector is 3 values column vector, the
    // transpose operation produce a row vector.
    vpRowVector tu_t = tu.t();

    // Print the transpose row vector
    std::cout << tu_t << std::endl;
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
