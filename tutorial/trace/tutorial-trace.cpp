/*! \example tutorial-trace.cpp */
//#define VP_TRACE        // Activate the trace mode
//#define VP_DEBUG        // Activate the debug mode
#define VP_DEBUG_MODE 2 // Activate debug level 1 and 2

#include <visp3/core/vpDebug.h>

int main()
{
  vpIN_FCT("main()"); // std::cout if VP_TRACE defined

  // Check the active debug levels set in VP_DEBUG_MODE
  std::cout << "Debug level 1 active: " << vpDEBUG_ENABLE(1) << std::endl;
  std::cout << "Debug level 2 active: " << vpDEBUG_ENABLE(2) << std::endl;
  std::cout << "Debug level 3 active: " << vpDEBUG_ENABLE(3) << std::endl;

  // C-like trace printings if VP_TRACE defined
  vpTRACE("C-like trace");              // std::cout
  vpTRACE(1, "C-like trace level 1");   // std::cout

  vpERROR_TRACE("C-like error trace");            // std::cerr
  vpERROR_TRACE(1, "C-like error trace level 1"); // std::cerr if VP_DEBUG_MODE value is >= 1

  // C-like debug printings if VP_DEBUG defined
  vpDEBUG_TRACE ("C-like debug trace"); // stdout
  vpDERROR_TRACE("C-like error trace"); // stderr

  vpDEBUG_TRACE (2, "C-like debug trace level 2"); // std::cout if VP_DEBUG_MODE value >= 2
  vpDERROR_TRACE(2, "C-like error trace level 2"); // std::cerr if VP_DEBUG_MODE value >= 2

  // C++-like trace printings if VP_TRACE defined
  vpCTRACE << "C++-like trace" << std::endl;       // std::cout
  vpCERROR << "C++-like error trace" << std::endl; // std::cerr

  // C++-like debug printings if VP_DEBUG defined
  vpCDEBUG(2) << "C++-like debug trace level 2" << std::endl; // std::cout if VP_DEBUG_MODE value >= 2

  vpOUT_FCT("main()"); // std::cout if VP_TRACE defined
}
