
#include <visp3/visual_features/vpFeatureLuminanceMapping.h>

#if defined(VISP_HAVE_CATCH2)

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>




SCENARIO("Using PCA features", "[visual_features]")
{
  GIVEN("A matrix containing simple data")
  {
    unsigned int dataCols = 10;
    unsigned int trueComponents = dataCols / 2;
    vpMatrix data(40, dataCols, 0.0);
    std::cout << data.size() << std::endl;
    std::cout << data.getRows() << " " << data.getCols() << std::endl;
    for (unsigned c = 0; c < dataCols / 2; ++c) {
      unsigned start = c * data.getRows() / trueComponents;
      unsigned end = (c + 1) * (data.getRows() / trueComponents);
      std::cout << start << " " << end << std::endl;
      for (unsigned row = start; row < end; ++row) {
        data[row][c] = 1;
      }
    }

    std::cout << data << std::endl;

    WHEN("Learning PCA basis with too many components")
    {
      unsigned int k = data.getCols() + 1;
      THEN("An exception is thrown")
      {
        REQUIRE_THROWS(vpLuminancePCA::learn(data.transpose(), k));
      }
    }
    WHEN("Learning PCA basis")
    {
      unsigned int k = dataCols / 2;
      vpLuminancePCA pca = vpLuminancePCA::learn(data.transpose(), k);
      vpMatrix basis = pca.getBasis();
      THEN("Basis has correct dimensions")
      {
        REQUIRE(basis.getRows() == k);
        REQUIRE(basis.getCols() == dataCols);
      }
    }

  }
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
