#include <string>
#include <jni.h>
#include "visp3/core/vpColVector.h"
#include "visp3/core/vpRowVector.h"
#include "visp3/core/vpMatrix.h"
using namespace std;

extern "C" { 

//
//   vpColVector(int n, double val)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_10 (JNIEnv*, jclass, jint, jdouble);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_10
  (JNIEnv* env, jclass , jint n, jdouble val)
{
        
        vpColVector* _retval_ = new vpColVector( (int)n, (double)val );
        return (jlong) _retval_;
}



//
//   vpColVector(vector_double v)
//

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_11 (JNIEnv *, jclass, jdoubleArray);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_11
  (JNIEnv* env, jclass , jdoubleArray v_list)
{
		jboolean isCopy1;
		int n = env->GetArrayLength(v_list);
		double* arr = env->GetDoubleArrayElements(v_list,&isCopy1);
        vector< double > v(arr, arr + n);
        vpColVector* _retval_ = new vpColVector( v );

		if (isCopy1 == JNI_TRUE)
       		env -> ReleaseDoubleArrayElements(v_list, arr, JNI_ABORT);
        return (jlong) _retval_;
}



//
//   vpColVector(vector_float v)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_12 (JNIEnv *, jclass, jfloatArray);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_12
  (JNIEnv* env, jclass , jfloatArray v_list)
{
		jboolean isCopy1;
        int n = env->GetArrayLength(v_list);
		float* arr = env->GetFloatArrayElements(v_list,&isCopy1);
        vector< float > v(arr, arr + n);
        vpColVector* _retval_ = new vpColVector( v );

		if (isCopy1 == JNI_TRUE)
       		env -> ReleaseFloatArrayElements(v_list, arr, JNI_ABORT);
        return (jlong) _retval_;
}



//
//   vpColVector(vpColVector v, int r, int nrows)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_13 (JNIEnv*, jclass, jlong, jint, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_13
  (JNIEnv* env, jclass , jlong v_nativeObj, jint r, jint nrows)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        vpColVector* _retval_ = new vpColVector( v, (int)r, (int)nrows );
        return (jlong) _retval_;
}



//
//   vpColVector(vpColVector v)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_14 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_14
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        vpColVector* _retval_ = new vpColVector( v );
        return (jlong) _retval_;
}



//
//   vpColVector(vpColVector v)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_15 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_15
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        vpColVector* _retval_ = new vpColVector( v );
        return (jlong) _retval_;
}



//
//   vpColVector(vpMatrix M, int j)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_16 (JNIEnv*, jclass, jlong, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_16
  (JNIEnv* env, jclass , jlong M_nativeObj, jint j)
{
        vpMatrix& M = *((vpMatrix*)M_nativeObj);
        vpColVector* _retval_ = new vpColVector( M, (int)j );
        return (jlong) _retval_;
}



//
//   vpColVector(vpMatrix M)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_17 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_17
  (JNIEnv* env, jclass , jlong M_nativeObj)
{
        vpMatrix& M = *((vpMatrix*)M_nativeObj);
        vpColVector* _retval_ = new vpColVector( M );
        return (jlong) _retval_;
}



//
//   vpColVector()
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_18 (JNIEnv*, jclass);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_VpColVector_18
  (JNIEnv* env, jclass )
{
        
        vpColVector* _retval_ = new vpColVector(  );
        return (jlong) _retval_;
}



//
// static double dotProd(vpColVector a, vpColVector b)
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_dotProd_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_dotProd_10
  (JNIEnv* env, jclass , jlong a_nativeObj, jlong b_nativeObj)
{
        vpColVector& a = *((vpColVector*)a_nativeObj);
        vpColVector& b = *((vpColVector*)b_nativeObj);
        double _retval_ = vpColVector::dotProd( a, b );
        return _retval_;
}



//
//  double euclideanNorm()
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_euclideanNorm_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_euclideanNorm_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        double _retval_ = me->euclideanNorm(  );
        return _retval_;
}



//
//  double infinityNorm()
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_infinityNorm_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_infinityNorm_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        double _retval_ = me->infinityNorm(  );
        return _retval_;
}



//
// static double mean(vpColVector v)
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_mean_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_mean_10
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        double _retval_ = vpColVector::mean( v );
        return _retval_;
}



//
// static double median(vpColVector v)
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_median_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_median_10
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        double _retval_ = vpColVector::median( v );
        return _retval_;
}



//
// static double stdev(vpColVector v, bool useBesselCorrection = false)
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_stdev_10 (JNIEnv*, jclass, jlong, jboolean);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_stdev_10
  (JNIEnv* env, jclass , jlong v_nativeObj, jboolean useBesselCorrection)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        double _retval_ = vpColVector::stdev( v, (bool)useBesselCorrection );
        return _retval_;
}





JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_stdev_11 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_stdev_11
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        double _retval_ = vpColVector::stdev( v );
        return _retval_;
}



//
//  double sum()
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_sum_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_sum_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        double _retval_ = me->sum(  );
        return _retval_;
}



//
//  double sumSquare()
//



JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_sumSquare_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpColVector_sumSquare_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        double _retval_ = me->sumSquare(  );
        return _retval_;
}



//
//  void clear()
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_clear_10 (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_clear_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->clear();
        return;
}



//
//  void deg2rad()
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_deg2rad_10 (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_deg2rad_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->deg2rad();
        return;
}



//
//  void init(vpColVector v, int r, int nrows)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_init_10 (JNIEnv*, jclass, jlong, jlong, jint, jint);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_init_10
  (JNIEnv* env, jclass , jlong self, jlong v_nativeObj, jint r, jint nrows)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector& v = *((vpColVector*)v_nativeObj);
        me->init( v, (int)r, (int)nrows );
        return;
}


//
//  void insert(int i, vpColVector v)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_insert_10 (JNIEnv*, jclass, jlong, jint, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_insert_10
  (JNIEnv* env, jclass , jlong self, jint i, jlong v_nativeObj)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector& v = *((vpColVector*)v_nativeObj);
        me->insert( (int)i, v );
        return;
}


//
//  void insert(vpColVector v, int r, int c = 0)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_insert_12 (JNIEnv*, jclass, jlong, jlong, jint, jint);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_insert_12
  (JNIEnv* env, jclass , jlong self, jlong v_nativeObj, jint r, jint c)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector& v = *((vpColVector*)v_nativeObj);
        me->insert( v, (int)r, (int)c );
        return;
}



//
//  void rad2deg()
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_rad2deg_10 (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_rad2deg_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->rad2deg();
        return;
}



//
//  void reshape(vpMatrix M, int nrows, int ncols)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_reshape_10 (JNIEnv*, jclass, jlong, jlong, jint, jint);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_reshape_10
  (JNIEnv* env, jclass , jlong self, jlong M_nativeObj, jint nrows, jint ncols)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpMatrix& M = *((vpMatrix*)M_nativeObj);
        me->reshape( M, (int)nrows, (int)ncols );
        return;
}



//
//  void resize(int i, bool flagNullify = true)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_resize_10 (JNIEnv*, jclass, jlong, jint, jboolean);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_resize_10
  (JNIEnv* env, jclass , jlong self, jint i, jboolean flagNullify)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->resize( (int)i, (bool)flagNullify );
        return;
}





JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_resize_11 (JNIEnv*, jclass, jlong, jint);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_resize_11
  (JNIEnv* env, jclass , jlong self, jint i)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->resize( (int)i );
        return;
}



//
//  void resize(int nrows, int ncols, bool flagNullify)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_resize_12 (JNIEnv*, jclass, jlong, jint, jint, jboolean);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_resize_12
  (JNIEnv* env, jclass , jlong self, jint nrows, jint ncols, jboolean flagNullify)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->resize( (int)nrows, (int)ncols, (bool)flagNullify );
        return;
}



//
//  void setIdentity(double val = 1.0)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_setIdentity_10 (JNIEnv*, jclass, jlong, jdouble);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_setIdentity_10
  (JNIEnv* env, jclass , jlong self, jdouble val)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->setIdentity( (double)val );
        return;
}





JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_setIdentity_11 (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_setIdentity_11
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->setIdentity(  );
        return;
}



//
//  void stack(double d)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stack_10 (JNIEnv*, jclass, jlong, jdouble);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stack_10
  (JNIEnv* env, jclass , jlong self, jdouble d)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        me->stack( (double)d );
        return;
}



//
// static void stack(vpColVector A, vpColVector B, vpColVector C)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stack_11 (JNIEnv*, jclass, jlong, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stack_11
  (JNIEnv* env, jclass , jlong A_nativeObj, jlong B_nativeObj, jlong C_nativeObj)
{
        vpColVector& A = *((vpColVector*)A_nativeObj);
        vpColVector& B = *((vpColVector*)B_nativeObj);
        vpColVector& C = *((vpColVector*)C_nativeObj);
        vpColVector::stack( A, B, C );
        return;
}



//
//  void stack(vpColVector v)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stack_12 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stack_12
  (JNIEnv* env, jclass , jlong self, jlong v_nativeObj)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector& v = *((vpColVector*)v_nativeObj);
        me->stack( v );
        return;
}



//
// static void stackMatrices(vpColVector A, vpColVector B, vpColVector C)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stackMatrices_10 (JNIEnv*, jclass, jlong, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stackMatrices_10
  (JNIEnv* env, jclass , jlong A_nativeObj, jlong B_nativeObj, jlong C_nativeObj)
{
        vpColVector& A = *((vpColVector*)A_nativeObj);
        vpColVector& B = *((vpColVector*)B_nativeObj);
        vpColVector& C = *((vpColVector*)C_nativeObj);
        vpColVector::stackMatrices( A, B, C );
        return;
}



//
//  void stackMatrices(vpColVector r)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stackMatrices_11 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_stackMatrices_11
  (JNIEnv* env, jclass , jlong self, jlong r_nativeObj)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector& r = *((vpColVector*)r_nativeObj);
        me->stackMatrices( r );
        return;
}



//
//  void transpose(vpRowVector v)
//



JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_transpose_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpColVector_transpose_10
  (JNIEnv* env, jclass , jlong self, jlong v_nativeObj)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpRowVector& v = *((vpRowVector*)v_nativeObj);
        me->transpose( v );
        return;
}



//
// static vpColVector cross(vpColVector a, vpColVector b)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_cross_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_cross_10
  (JNIEnv* env, jclass , jlong a_nativeObj, jlong b_nativeObj)
{
        vpColVector& a = *((vpColVector*)a_nativeObj);
        vpColVector& b = *((vpColVector*)b_nativeObj);
        vpColVector _retval_ = vpColVector::cross( a, b );
        return (jlong) new vpColVector(_retval_);
}



//
// static vpColVector crossProd(vpColVector a, vpColVector b)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_crossProd_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_crossProd_10
  (JNIEnv* env, jclass , jlong a_nativeObj, jlong b_nativeObj)
{
        vpColVector& a = *((vpColVector*)a_nativeObj);
        vpColVector& b = *((vpColVector*)b_nativeObj);
        vpColVector _retval_ = vpColVector::crossProd( a, b );
        return (jlong) new vpColVector(_retval_);
}



//
//  vpColVector extract(int r, int colsize)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_extract_10 (JNIEnv*, jclass, jlong, jint, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_extract_10
  (JNIEnv* env, jclass , jlong self, jint r, jint colsize)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector _retval_ = me->extract( (int)r, (int)colsize );
        return (jlong) new vpColVector(_retval_);
}



//
//  vpColVector hadamard(vpColVector v)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_hadamard_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_hadamard_10
  (JNIEnv* env, jclass , jlong self, jlong v_nativeObj)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector& v = *((vpColVector*)v_nativeObj);
        vpColVector _retval_ = me->hadamard( v );
        return (jlong) new vpColVector(_retval_);
}



//
// static vpColVector invSort(vpColVector v)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_invSort_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_invSort_10
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        vpColVector _retval_ = vpColVector::invSort( v );
        return (jlong) new vpColVector(_retval_);
}



//
//  vpColVector normalize(vpColVector x)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_normalize_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_normalize_10
  (JNIEnv* env, jclass , jlong self, jlong x_nativeObj)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector& x = *((vpColVector*)x_nativeObj);
        vpColVector _retval_ = me->normalize( x );
        return (jlong) new vpColVector(_retval_);
}



//
//  vpColVector normalize()
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_normalize_11 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_normalize_11
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector _retval_ = me->normalize(  );
        return (jlong) new vpColVector(_retval_);
}



//
//  vpColVector rows(int first_row, int last_row)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_rows_10 (JNIEnv*, jclass, jlong, jint, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_rows_10
  (JNIEnv* env, jclass , jlong self, jint first_row, jint last_row)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpColVector _retval_ = me->rows( (int)first_row, (int)last_row );
        return (jlong) new vpColVector(_retval_);
}



//
// static vpColVector sort(vpColVector v)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_sort_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_sort_10
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        vpColVector _retval_ = vpColVector::sort( v );
        return (jlong) new vpColVector(_retval_);
}



//
// static vpColVector stack(vpColVector A, vpColVector B)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_stack_13 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_stack_13
  (JNIEnv* env, jclass , jlong A_nativeObj, jlong B_nativeObj)
{
        vpColVector& A = *((vpColVector*)A_nativeObj);
        vpColVector& B = *((vpColVector*)B_nativeObj);
        vpColVector _retval_ = vpColVector::stack( A, B );
        return (jlong) new vpColVector(_retval_);
}



//
// static vpColVector stackMatrices(vpColVector A, vpColVector B)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_stackMatrices_12 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_stackMatrices_12
  (JNIEnv* env, jclass , jlong A_nativeObj, jlong B_nativeObj)
{
        vpColVector& A = *((vpColVector*)A_nativeObj);
        vpColVector& B = *((vpColVector*)B_nativeObj);
        vpColVector _retval_ = vpColVector::stackMatrices( A, B );
        return (jlong) new vpColVector(_retval_);
}



//
//  vpMatrix reshape(int nrows, int ncols)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_reshape_11 (JNIEnv*, jclass, jlong, jint, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_reshape_11
  (JNIEnv* env, jclass , jlong self, jint nrows, jint ncols)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpMatrix _retval_ = me->reshape( (int)nrows, (int)ncols );
        return (jlong) new vpMatrix(_retval_);
}



//
// static vpMatrix skew(vpColVector v)
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_skew_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_skew_10
  (JNIEnv* env, jclass , jlong v_nativeObj)
{
        vpColVector& v = *((vpColVector*)v_nativeObj);
        vpMatrix _retval_ = vpColVector::skew( v );
        return (jlong) new vpMatrix(_retval_);
}



//
//  vpRowVector t()
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_t_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_t_10
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpRowVector _retval_ = me->t(  );
        return (jlong) new vpRowVector(_retval_);
}



//
//  vpRowVector transpose()
//



JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_transpose_11 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpColVector_transpose_11
  (JNIEnv* env, jclass , jlong self)
{
        vpColVector* me = (vpColVector*) self; //TODO: check for NULL
        vpRowVector _retval_ = me->transpose(  );
        return (jlong) new vpRowVector(_retval_);
}

}
