/*! \example tutorial-matlab.cpp
 *
 * Tutorial using ViSP and MATLAB
 * Determine column-wise sum of ViSP matrix using MATLAB Engine
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//! [Include]
#include <engine.h>
#include <matrix.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>
//! [Include]

int main()
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  // ViSP matrix containing input data
  //! [InputData]
  vpMatrix x(3, 3, 0);
  x[0][0] = 1;
  x[0][1] = 2;
  x[0][2] = 3;
  x[1][0] = 4;
  x[1][1] = 5;
  x[1][2] = 6;
  x[2][0] = 7;
  x[2][1] = 8;
  x[2][2] = 9;
  //! [InputData]
  int xCols = x.getCols();
  int xRows = x.getRows();

  //! [MATLABVariables]
  // MATLAB Engine
  Engine *ep;

  // MATLAB array to store input data to MATLAB
  mxArray *T = mxCreateDoubleMatrix(xRows, xCols, mxREAL);

  // MATLAB array to store output data from MATLAB
  mxArray *D = nullptr;
  //! [MATLABVariables]

  // Temporary variable to hold Output data
  double res[3];
  int resCols = 3;

  // Display input data to the user
  std::cout << "ViSP Input Matrix:" << std::endl;
  for (size_t i = 0; i < xRows; i++) {
    for (size_t j = 0; j < xCols; j++)
      std::cout << x.data[i * xCols + j] << " ";
    std::cout << std::endl;
  }

  // Start a MATLAB Engine process using engOpen
  //! [EngineOpen]
  if (!(ep = engOpen(""))) {
    fprintf(stderr, "\nCan't start MATLAB engine\n");
    return EXIT_FAILURE;
  }
  //! [EngineOpen]

  // Copy the contents of ViSP matrix to the MATLAB matrix variable T
  //! [CopyToVariable]
  memcpy((void *)mxGetPr(T), (void *)x.data, xRows * xCols * sizeof(double));
  //! [CopyToVariable]

  // Place the variable T into the MATLAB workspace
  //! [AddToWorkspace]
  engPutVariable(ep, "Tm", T);
  //! [AddToWorkspace]

  // Determine the sum of each column of input matrix x
  // ViSP matrix is row-major and MATLAB matrix is column-major, so transpose the matrix T before evaluation
  //! [EvalFunction]
  engEvalString(ep, "Dm = sum(Tm');");
  //! [EvalFunction]

  // Get the variable D from the MATLAB workspace
  //! [GetFromWorkspace]
  D = engGetVariable(ep, "Dm");
  //! [GetFromWorkspace]

  // Copy the contents of MATLAB variable D to local variable res
  //! [CopyFromVariable]
  memcpy((void *)res, (void *)mxGetPr(D), sizeof(res));
  //! [CopyFromVariable]

  // Display output data to the user
  std::cout << std::endl << "MATLAB Output Matrix (Column-wise sum):" << std::endl;
  for (size_t i = 0; i < resCols; i++)
    std::cout << res[i] << " ";
  std::cout << std::endl;

  // Wait until user exits
  std::cout << std::endl << "Hit return to quit\n" << std::endl;
  fgetc(stdin);

  // Free memory, close MATLAB Engine and Exit
  //! [Exit]
  mxDestroyArray(T);
  mxDestroyArray(D);
  engEvalString(ep, "close;");
  engClose(ep);
  //! [Exit]

  return EXIT_SUCCESS;
}
