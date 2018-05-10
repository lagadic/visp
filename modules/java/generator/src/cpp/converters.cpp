#define LOG_TAG "org.visp.utils.Converters"
#include "common.h"

using namespace vp;
// CHECK_MAT is defined in visp/modules/java/include/visp/java.hpp

//vector_double

void Mat_to_vector_double(const vpMatrix& mat, std::vector<double>& v_double){
    v_double.clear();
    v_double.assign(mat.data,mat.data + mat.getCols()*mat.getRows());
}

void vector_double_to_Mat(const std::vector<double>& v_double, vpMatrix& mat){
    if (mat.size() == v_double.size())
        memcpy(mat.data,v_double.data(), sizeof(double)* v_double.size());
}

/*
//vector_Rect

void Mat_to_vector_Rect(vpMatrix& mat, std::vector<Rect>& v_rect){
    v_rect.clear();
    v_rect = (std::vector<Rect>) mat;
}

void vector_Rect_to_Mat(std::vector<Rect>& v_rect, vpMatrix& mat)
{
    mat = Mat(v_rect, true);
}

//vector_Point
void Mat_to_vector_Point(vpMatrix& mat, std::vector<Point>& v_point)
{
    v_point.clear();
    CHECK_MAT(mat.cols==1);
    v_point = (std::vector<Point>) mat;
}

void vector_Point_to_Mat(std::vector<Point>& v_point, vpMatrix& mat)
{
    mat = Mat(v_point, true);
}

//vector_Mat
void Mat_to_vector_Mat(cv::vpMatrix& mat, std::vector<cv::Mat>& v_mat)
{
    v_mat.clear();
    if(mat.cols == 1)
    {
        v_mat.reserve(mat.rows);
        for(int i=0; i<mat.rows; i++)
        {
            Vec<int, 2> a = mat.at< Vec<int, 2> >(i, 0);
            long long addr = (((long long)a[0])<<32) | (a[1]&0xffffffff);
            vpMatrix& m = *( (Mat*) addr );
            v_mat.push_back(m);
        }
    } else {
        LOGD("Mat_to_vector_Mat() FAILED: mat.cols == 1");
    }
}


void vector_Mat_to_Mat(std::vector<cv::Mat>& v_mat, cv::vpMatrix& mat)
{
    int count = (int)v_mat.size();
    mat.create(count, 1, CV_32SC2);
    for(int i=0; i<count; i++)
    {
        long long addr = (long long) new Mat(v_mat[i]);
        mat.at< Vec<int, 2> >(i, 0) = Vec<int, 2>(addr>>32, addr&0xffffffff);
    }
}

void Mat_to_vector_vector_Point(vpMatrix& mat, std::vector< std::vector< Point > >& vv_pt)
{
    std::vector<Mat> vm;
    vm.reserve( mat.rows );
    Mat_to_vector_Mat(mat, vm);
    for(size_t i=0; i<vm.size(); i++)
    {
        std::vector<Point> vpt;
        Mat_to_vector_Point(vm[i], vpt);
        vv_pt.push_back(vpt);
    }
}

void vector_vector_Point_to_Mat(std::vector< std::vector< Point > >& vv_pt, vpMatrix& mat)
{
    std::vector<Mat> vm;
    vm.reserve( vv_pt.size() );
    for(size_t i=0; i<vv_pt.size(); i++)
    {
        Mat m;
        vector_Point_to_Mat(vv_pt[i], m);
        vm.push_back(m);
    }
    vector_Mat_to_Mat(vm, mat);
}
*/
