
#include <visp3/visual_features/vpFeatureLuminanceMapping.h>

#if defined(VISP_HAVE_CATCH2)

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>




SCENARIO("Learning a new basis for PCA", "[visual_features]")
{
  vpMatrix data(50, 100);
  vpLuminancePCA::learn(data, 32);
}
int main(int argc, char *argv[])
{
  Catch::Session session; // There must be exactly one instance
  session.applyCommandLine(argc, argv);

  int numFailed = session.run();
  return numFailed;
}

#else

int main()
{
  return EXIT_SUCCESS;
}
#endif
