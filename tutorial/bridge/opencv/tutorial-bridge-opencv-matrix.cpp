//! \example tutorial-bridge-opencv-matrix.cpp
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/io/vpImageIo.h>

#if defined(HAVE_OPENCV_IMGPROC)
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#endif

int main()
{
#if defined(VISP_HAVE_OPENCV) && defined(HAVE_OPENCV_IMGPROC)
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  {
    std::cout << "From OpenCV to ViSP conversion" << std::endl;
    //! [Create OpenCV matrix]
    cv::Mat M_cv = (cv::Mat_<double>(3, 4) << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12);
    std::cout << "M_cv: \n" << M_cv << std::endl;
    //! [Create OpenCV matrix]

    //! [Convert to ViSP matrix with deep copy]
    vpMatrix M(static_cast<unsigned int>(M_cv.rows), static_cast<unsigned int>(M_cv.cols));
    memcpy(M.data, M_cv.data, sizeof(double) * static_cast<size_t>(M_cv.rows * M_cv.cols));
    std::cout << "M: \n" << M << std::endl;
    //! [Convert to ViSP matrix with deep copy]
  }

  {
    std::cout << "From ViSP to OpenCV conversion" << std::endl;
    //! [Create ViSP matrix]
    vpMatrix M(3, 4, { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 });
    std::cout << "M: \n" << M << std::endl;
    //! [Create ViSP matrix]

    //! [Convert to OpenCV matrix with deep copy]
    cv::Mat tmp(static_cast<int>(M.getRows()), static_cast<int>(M.getCols()), CV_64F, static_cast<void *>(M.data));
    cv::Mat M_cv_deep = tmp.clone();
    std::cout << "M_cv_deep: \n" << M_cv_deep << std::endl;
    //! [Convert to OpenCV matrix with deep copy]

    //! [Convert to OpenCV matrix without deep copy]
    cv::Mat M_cv(static_cast<int>(M.getRows()), static_cast<int>(M.getCols()), CV_64F, static_cast<void *>(M.data));
    std::cout << "M_cv: \n" << M_cv << std::endl;
    //! [Convert to OpenCV matrix without deep copy]

    //! [Modify ViSP matrix]
    std::cout << "Set M = eye" << std::endl;
    M.eye();
    std::cout << "M: \n" << M << std::endl;
    std::cout << "M_cv: \n" << M_cv << std::endl;
    //! [Modify ViSP matrix]
}
#endif
}
