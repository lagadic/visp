
#include <visp3/visual_features/vpFeatureLuminanceMapping.h>
#include <visp3/core/vpSubMatrix.h>

#include <visp3/core/vpUniRand.h>
#include <visp3/core/vpIoTools.h>
#if defined(VISP_HAVE_CATCH2)

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>


vpMatrix orthogonalBasis(unsigned n, unsigned seed)
{
  vpUniRand rand(seed);
  vpMatrix basis(n, n);
  vpColVector norms(n);

  //start with random basis
  for (unsigned int row = 0; row < n; ++row) {
    double norm = 0.0;
    for (unsigned int col = 0; col < n; ++col) {
      basis[row][col] = rand.uniform(-1.0, 1.0);
      norm += basis[row][col] * basis[row][col];
    }
    norm = 1.0 / sqrt(norm);
    for (unsigned int col = 0; col < n; ++col) {
      basis[row][col] *= norm;
    }

  }
  // Apply gram schmidt process
  norms[0] = basis.getRow(0).sumSquare();
  for (unsigned i = 1; i < n; ++i) {
    vpColVector uit = basis.getRow(i).t();

    for (unsigned j = 0; j < i; ++j) {
      vpRowVector vj = basis.getRow(j);
      vpRowVector res = vj * ((vj * uit) / (norms[j]));
      for (unsigned k = 0; k < n; ++k) {
        basis[i][k] -= res[k];
      }
    }
    norms[i] = basis.getRow(i).sumSquare();
  }
  for (unsigned int row = 0; row < n; ++row) {
    double norm = sqrt(norms[row]);

    for (unsigned int col = 0; col < n; ++col) {
      basis[row][col] /= norm;
    }
  }
  return basis;

}

SCENARIO("Using PCA features", "[visual_features]")
{
  GIVEN("A matrix containing simple data")
  {
    const unsigned numDataPoints = 200;
    const unsigned int dataDim = 50;
    const unsigned int trueComponents = 5;
    // Generate numDataPoints vectors in a "dataDim"-dimensional space.
    // The data is generated from "trueComponents" vectors, that are orthogonal
    const vpMatrix orthoFull = orthogonalBasis(dataDim, 42); // dataDim x dataDim
    const vpMatrix ortho(orthoFull, 0, 0, trueComponents, dataDim); // trueComponents X dataDim
    const vpMatrix coefficients(numDataPoints, trueComponents);
    vpUniRand rand(17);
    for (unsigned int i = 0; i < coefficients.getRows(); ++i) {
      for (unsigned int j = 0; j < coefficients.getCols(); ++j) {
        coefficients[i][j] = rand.uniform(-1.0, 1.0);
      }
    }

    vpMatrix data = coefficients * ortho;

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
      for (unsigned int k = 1; k <= trueComponents; ++k) {

        const vpLuminancePCA pca = vpLuminancePCA::learn(data.transpose(), k);
        const vpMatrix &basis = *pca.getBasis();

        THEN("Basis has correct dimensions")
        {
          REQUIRE(basis.getRows() == k);
          REQUIRE(basis.getCols() == dataDim);
        }
        THEN("The basis is orthonormal")
        {
          const vpMatrix Iapprox = basis * basis.t();
          vpMatrix I;
          I.eye(basis.getRows());
          bool matrixSame = true;
          for (unsigned int row = 0; row < I.getRows(); ++row) {
            for (unsigned int col = 0; col < I.getCols(); ++col) {
              if (fabs(I[row][col] - Iapprox[row][col]) > 1e-6) {
                matrixSame = false;
                break;
              }
            }
          }
          REQUIRE(matrixSame);
        }
        THEN("Mean vector has correct dimensions")
        {
          REQUIRE(pca.getMean()->getRows() == dataDim);
          REQUIRE(pca.getMean()->getCols() == 1);
        }

        THEN("Saving and loading pca leads to same basis and mean")
        {
          const std::string tempDir = vpIoTools::makeTempDirectory("visp_test_pca");
          const std::string basisFile = vpIoTools::createFilePath(tempDir, "basis.txt");
          const std::string meanFile = vpIoTools::createFilePath(tempDir, "mean.txt");
          const std::string varFile = vpIoTools::createFilePath(tempDir, "var.txt");


          pca.save(basisFile, meanFile, varFile);

          const vpLuminancePCA pca2 = vpLuminancePCA::load(basisFile, meanFile, varFile);
          const vpMatrix basisDiff = *pca.getBasis() - *pca2.getBasis();
          const vpColVector meanDiff = *pca.getMean() - *pca2.getMean();
          const vpColVector explainedVarDiff = pca.getExplainedVariance() - pca2.getExplainedVariance();
          bool basisSame = true;
          bool meanSame = true;
          bool explainedVarSame = true;

          for (unsigned int i = 0; i < basisDiff.getRows(); ++i) {
            for (unsigned int j = 0; j < basisDiff.getCols(); ++j) {
              if (fabs(basisDiff[i][j]) > 1e-10) {
                basisSame = false;
                break;
              }
            }
          }
          REQUIRE(basisSame);
          for (unsigned int i = 0; i < meanDiff.getRows(); ++i) {
            if (fabs(meanDiff[i]) > 1e-10) {
              meanSame = false;
              break;
            }
          }
          REQUIRE(meanSame);
          for (unsigned int i = 0; i < explainedVarDiff.getRows(); ++i) {
            if (fabs(explainedVarDiff[i]) > 1e-10) {
              explainedVarSame = false;
              break;
            }
          }
          REQUIRE(explainedVarSame);
        }

        THEN("Explained variance is below 1 and sorted in descending order")
        {
          const vpColVector var = pca.getExplainedVariance();
          REQUIRE(var.sum() < 1.0);
          for (int i = 1; i < (int)var.getRows() - 1; ++i) {
            REQUIRE(var[i] >= var[i + 1]);
          }
        }
        if (k == trueComponents) {
          WHEN("K is the true manifold dimensionality")
          {
            THEN("explained variance is close to 1")
            {
              REQUIRE(pca.getExplainedVariance().sum() > 0.99);
            }
          }
        }

        THEN("Projecting data has correct dimensions")
        {

        }
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
