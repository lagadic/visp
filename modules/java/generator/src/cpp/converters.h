#include "visp3/visp_modules.h"
#include "visp3/core/vpMatrix.h"
//#include "visp3/core/vpRect.h"
//#include "visp3/core/vpPoint.h"
#include "visp3/core/vpException.h"

// TODO Check if need to add more test functions

void Mat_to_vector_double(const vp::vpMatrix& mat, std::vector<double>& v_double);
void vector_double_to_Mat(const std::vector<double>& v_double, vp::vpMatrix& mat);

/*
void Mat_to_vector_Rect(const vp::vpMatrix& mat, std::vector<vp::vpRect>& v_rect);
void vector_Rect_to_Mat(const std::vector<vp::vpRect>& v_rect, vp::vpMatrix& mat);

// TODO Do we need to add ImagePoint to vpMatrix conversion
void Mat_to_vector_Point(vp::vpMatrix& mat, std::vector<vp::vpPoint>& v_point);
void vector_Point_to_Mat(std::vector<vp::vpPoint>& v_point, vp::vpMatrix& mat);

void Mat_to_vector_Mat(vp::vpMatrix& mat, std::vector<vp::vpMatrix>& v_mat);
void vector_Mat_to_Mat(std::vector<vp::vpMatrix>& v_mat, vp::vpMatrix& mat);

void Mat_to_vector_vector_Point(vp::vpMatrix& mat, std::vector< std::vector< vp::vpPoint > >& vv_pt);
void vector_vector_Point_to_Mat(std::vector< std::vector< vp::vpPoint > >& vv_pt, vp::vpMatrix& mat);
*/
