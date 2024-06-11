
/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Performs various tests on the point class.
 */

/*!
 * \file testLuminanceMapping.cpp
 * \example testLuminanceMapping.cpp
 */

#include <visp3/visual_features/vpFeatureLuminanceMapping.h>

#include <visp3/core/vpSubMatrix.h>

#include <visp3/core/vpUniRand.h>
#include <visp3/core/vpIoTools.h>
#if defined(VISP_HAVE_CATCH2)

#define CATCH_CONFIG_RUNNER
#include <catch.hpp>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

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
    const unsigned h = 16, w = 16;
    const unsigned numDataPoints = 4;
    const unsigned int dataDim = h * w;
    const unsigned int trueComponents = 3;
    // Generate numDataPoints vectors in a "dataDim"-dimensional space.
    // The data is generated from "trueComponents" vectors, that are orthogonal
    const vpMatrix orthoFull = (orthogonalBasis(dataDim, 42) + vpMatrix(dataDim, dataDim, 1.0)) * 127.5; // dataDim x dataDim
    const vpMatrix ortho(orthoFull, 0, 0, trueComponents, dataDim); // trueComponents X dataDim
    const vpMatrix coefficients(numDataPoints, trueComponents);
    vpUniRand rand(17);
    for (unsigned int i = 0; i < coefficients.getRows(); ++i) {
      double sum = 0.0;
      for (unsigned int j = 0; j < coefficients.getCols(); ++j) {
        coefficients[i][j] = rand.uniform(0.0, 1.0);
        sum += coefficients[i][j] * coefficients[i][j];
      }
      const double inv_norm = 1.0 / sqrt(sum);
      for (unsigned int j = 0; j < coefficients.getCols(); ++j) {
        coefficients[i][j] *= inv_norm;
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
    WHEN("Learning with more images than pixels")
    {
      vpMatrix wrongData(20, 50);
      THEN("An exception is thrown")
      {
        REQUIRE_THROWS(vpLuminancePCA::learn(wrongData.transpose(), 32));
      }
    }
    WHEN("Learning PCA basis")
    {
      for (unsigned int k = 2; k <= trueComponents; ++k) {

        vpLuminancePCA pca = vpLuminancePCA::learn(data.transpose(), k);
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
        THEN("Modifying the basis size (number of inputs) by hand and saving")
        {
          const std::string tempDir = vpIoTools::makeTempDirectory("visp_test_pca_wrong");
          const std::string basisFile = vpIoTools::createFilePath(tempDir, "basis.txt");
          const std::string meanFile = vpIoTools::createFilePath(tempDir, "mean.txt");
          const std::string varFile = vpIoTools::createFilePath(tempDir, "var.txt");
          pca.getBasis()->resize(pca.getBasis()->getRows(), pca.getBasis()->getCols() - 1);
          REQUIRE_THROWS(pca.save(basisFile, meanFile, varFile));
        }
        THEN("Modifying the mean Columns by hand")
        {
          const std::string tempDir = vpIoTools::makeTempDirectory("visp_test_pca_wrong");
          const std::string basisFile = vpIoTools::createFilePath(tempDir, "basis.txt");
          const std::string meanFile = vpIoTools::createFilePath(tempDir, "mean.txt");
          const std::string varFile = vpIoTools::createFilePath(tempDir, "var.txt");

          std::shared_ptr<vpColVector> mean = pca.getMean();
          mean->resize(mean->getRows() + 1, false);
          REQUIRE_THROWS(pca.save(basisFile, meanFile, varFile));

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
              std::cout << meanDiff << std::endl;
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
            THEN("Inverse mapping leads back to the same data")
            {
              for (unsigned int i = 0; i < numDataPoints; ++i) {
                vpImage<unsigned char> I(h, w);
                for (unsigned int j = 0; j < data.getCols(); ++j) {
                  I.bitmap[j] = static_cast<unsigned char>(data[i][j]);
                }
                vpColVector s;
                pca.setBorder(0);
                pca.map(I, s);
                vpImage<unsigned char> Irec;
                pca.inverse(s, Irec);
                for (unsigned int j = 0; j < data.getCols(); ++j) {
                  REQUIRE(abs(static_cast<int>(I.bitmap[j]) - static_cast<int>(Irec.bitmap[j])) < 2);
                }
              }
            }
          }
        }

        THEN("Projecting data is correct")
        {
          {
            vpColVector s;
            pca.setBorder(0);
            vpImage<unsigned char> I(h, w);
            pca.map(I, s);
            REQUIRE(s.size() == pca.getProjectionSize());
          }
          {
            vpColVector s;
            const unsigned border = 3;
            pca.setBorder(border);
            REQUIRE(pca.getBorder() == border);
            vpImage<unsigned char> I(h + 2 * border, w + 2 * border);
            pca.map(I, s);
            REQUIRE(s.size() == pca.getProjectionSize());
          }
        }
      }
    }
  }
  WHEN("Saving unintialized PCA")
  {
    vpLuminancePCA pca;
    const std::string tempDir = vpIoTools::makeTempDirectory("visp_test_pca");
    const std::string basisFile = vpIoTools::createFilePath(tempDir, "basis.txt");
    const std::string meanFile = vpIoTools::createFilePath(tempDir, "mean.txt");
    const std::string varFile = vpIoTools::createFilePath(tempDir, "var.txt");
    THEN("an exception is thrown")
    {
      REQUIRE_THROWS(pca.save(basisFile, meanFile, varFile));
    }
  }
}

SCENARIO("Using DCT features", "[visual_features]")
{

  GIVEN("A matrix")
  {
    std::vector<std::tuple<vpMatrix, vpColVector, vpMatrix>> data = {
      {
        vpMatrix({
          {0.0, 1.0, 2.0},
          {3.0, 4.0, 5.0},
          {6.0, 7.0, 8.0}
        }),
        vpColVector(
          { 0.0, 1.0, 3.0, 6.0, 4.0, 2.0, 5.0, 7.0, 8.0 }
        ),
        vpMatrix({
          {0.0, 1.0, 5.0},
          {2.0, 4.0, 6.0},
          {3.0, 7.0, 8.0}
        })
      }
    };
    for (unsigned int i = 0; i < data.size(); ++i) {
      WHEN("Building the associated zigzag indexing matrix")
      {
        vpMatrix m = std::get<0>(data[i]);
        vpColVector contentAsZigzag = std::get<1>(data[i]);
        const vpMatrix mAfterWriterVec = std::get<2>(data[i]);
        vpLuminanceDCT::vpMatrixZigZagIndex zigzag;
        zigzag.init(m.getRows(), m.getCols());
        vpColVector s;
        THEN("Calling getValues with wrong matrix rows throws")
        {
          vpMatrix wrongM(m.getRows() + 1, m.getCols());
          REQUIRE_THROWS(zigzag.getValues(wrongM, 0, 2, s));
        }
        THEN("Calling getValues with wrong matrix cols throws")
        {
          vpMatrix wrongM(m.getRows(), m.getCols() + 1);
          REQUIRE_THROWS(zigzag.getValues(wrongM, 0, 2, s));
        }
        THEN("Calling getValues with wrong start and end arguments throws")
        {
          REQUIRE_THROWS(zigzag.getValues(m, 2, 1, s));
        }
        THEN("Calling getValues and querying all values returns correct result")
        {
          REQUIRE_NOTHROW(zigzag.getValues(m, 0, m.size(), s));
          REQUIRE(s == contentAsZigzag);
        }
        THEN("Calling getValues and querying a subset of the values is correct")
        {
          REQUIRE_NOTHROW(zigzag.getValues(m, 0, m.size() / 2, s));
          REQUIRE(s == contentAsZigzag.extract(0, m.size() / 2));
          REQUIRE_NOTHROW(zigzag.getValues(m, m.size() / 2, m.size(), s));
          REQUIRE(s == contentAsZigzag.extract(m.size() / 2, m.size() - m.size() / 2));
        }
        THEN("Calling setValues with wrong matrix rows throws")
        {
          vpMatrix wrongM(m.getRows() + 1, m.getCols());
          REQUIRE_THROWS(zigzag.setValues(contentAsZigzag, 0, wrongM));
        }
        THEN("Calling setValues with wrong matrix cols throws")
        {
          vpMatrix wrongM(m.getRows(), m.getCols() + 1);
          REQUIRE_THROWS(zigzag.setValues(contentAsZigzag, 0, wrongM));
        }

        THEN("Calling setValues with wrong start and vector size arguments throws")
        {
          REQUIRE_THROWS(zigzag.setValues(contentAsZigzag, m.size() - contentAsZigzag.size() + 1, m));
        }

        THEN("Calling setValues leads to expected result")
        {
          vpMatrix mWrite(m.getRows(), m.getCols());
          vpColVector powered = contentAsZigzag;
          for (unsigned i = 0; i < powered.size(); ++i) {
            powered[i] *= powered[i];
          }
          vpColVector poweredRead;
          REQUIRE_NOTHROW(zigzag.setValues(powered, 0, mWrite));
          REQUIRE_NOTHROW(zigzag.getValues(mWrite, 0, mWrite.size(), poweredRead));
          REQUIRE(powered == poweredRead);

          vpColVector indices = contentAsZigzag;
          for (unsigned i = 0; i < powered.size(); ++i) {
            indices[i] = static_cast<double>(i);
          }
          vpColVector indicesRead;
          REQUIRE_NOTHROW(zigzag.setValues(indices, 0, mWrite));
          REQUIRE(mWrite == mAfterWriterVec);

          vpMatrix m2(m.getRows(), m.getCols(), 0.0);
          zigzag.setValues(contentAsZigzag.extract(0, 3), 0, m2);

          vpColVector s2;
          zigzag.getValues(m2, 0, 3, s2);
          REQUIRE(s2 == contentAsZigzag.extract(0, 3));

        }

      }
    }


    GIVEN("A constant image")
    {
      vpImage<unsigned char> I(32, 64, 20);
      WHEN("Computing DCT")
      {
        vpLuminanceDCT dct(32);
        dct.setBorder(0);
        vpColVector s;
        dct.map(I, s);
        THEN("resulting feature vector has correct size")
        {
          REQUIRE(s.size() == 32);
        }
        THEN("The only non zero component is the first")
        {
          REQUIRE(s.sum() == Approx(s[0]).margin(1e-5));
        }

        vpImage<unsigned char> Ir;
        dct.inverse(s, Ir);
        REQUIRE((Ir.getRows() == I.getRows() && Ir.getCols() == I.getCols()));
        for (unsigned i = 0; i < I.getRows(); ++i) {
          for (unsigned j = 0; j < I.getCols(); ++j) {
            const int diff = abs(static_cast<int>(I[i][j]) - static_cast<int>(Ir[i][j]));
            REQUIRE(diff < 2);
            INFO("i = " + std::to_string(i) + ", j = " + std::to_string(j));
          }
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
