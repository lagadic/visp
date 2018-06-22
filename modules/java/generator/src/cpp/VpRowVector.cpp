#include <string>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRowVector.h>

extern "C" {

#if !defined(__ppc__)
// to suppress warning from jni.h on OS X
#  define TARGET_RT_MAC_CFM 0
#endif
#include <jni.h>


//
//   vpRowVector(int n, double val)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_10 (JNIEnv*, jclass, jint, jdouble);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_10
(JNIEnv* env, jclass , jint n, jdouble val)
{

  vpRowVector* _retval_ = new vpRowVector( (int)n, (double)val );
  return (jlong) _retval_;
}

//
//   vpRowVector(vector_double v)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_11 (JNIEnv*, jclass, jdoubleArray);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_11
(JNIEnv* env, jclass , jdoubleArray v_list)
{
  jboolean isCopy1;
  int n = env->GetArrayLength(v_list);
  double* arr = env->GetDoubleArrayElements(v_list,&isCopy1);
  std::vector< double > v(arr, arr + n);
  vpRowVector* _retval_ = new vpRowVector( v );

  if (isCopy1 == JNI_TRUE)
    env -> ReleaseDoubleArrayElements(v_list, arr, JNI_ABORT);
  return (jlong) _retval_;
}

//
//   vpRowVector(vector_float v)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_12 (JNIEnv*, jclass, jfloatArray);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_12
(JNIEnv* env, jclass , jfloatArray v_list)
{
  jboolean isCopy1;
  int n = env->GetArrayLength(v_list);
  float* arr = env->GetFloatArrayElements(v_list,&isCopy1);
  std::vector< float > v(arr, arr + n);
  vpRowVector* _retval_ = new vpRowVector( v );

  if (isCopy1 == JNI_TRUE)
    env -> ReleaseFloatArrayElements(v_list, arr, JNI_ABORT);
  return (jlong) _retval_;
}

//
//   vpRowVector(vpMatrix M, int i)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_13 (JNIEnv*, jclass, jlong, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_13
(JNIEnv* env, jclass , jlong M_nativeObj, jint i)
{
  vpMatrix& M = *((vpMatrix*)M_nativeObj);
  vpRowVector* _retval_ = new vpRowVector( M, (int)i );
  return (jlong) _retval_;
}

//
//   vpRowVector(vpMatrix M)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_14 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_14
(JNIEnv* env, jclass , jlong M_nativeObj)
{
  vpMatrix& M = *((vpMatrix*)M_nativeObj);
  vpRowVector* _retval_ = new vpRowVector( M );
  return (jlong) _retval_;
}

//
//   vpRowVector(vpRowVector v, int c, int ncols)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_15 (JNIEnv*, jclass, jlong, jint, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_15
(JNIEnv* env, jclass , jlong v_nativeObj, jint c, jint ncols)
{
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  vpRowVector* _retval_ = new vpRowVector( v, (int)c, (int)ncols );
  return (jlong) _retval_;
}

//
//   vpRowVector(vpRowVector v)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_16 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_16
(JNIEnv* env, jclass , jlong v_nativeObj)
{
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  vpRowVector* _retval_ = new vpRowVector( v );
  return (jlong) _retval_;
}

//
//   vpRowVector()
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_17 (JNIEnv*, jclass);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_VpRowVector_17
(JNIEnv* env, jclass )
{

  vpRowVector* _retval_ = new vpRowVector(  );
  return (jlong) _retval_;
}

//
//  double euclideanNorm()
//
JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_euclideanNorm_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_euclideanNorm_10
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  double _retval_ = me->euclideanNorm(  );
  return _retval_;
}

//
// static double mean(vpRowVector v)
//
JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_mean_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_mean_10
(JNIEnv* env, jclass , jlong v_nativeObj)
{
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  double _retval_ = vpRowVector::mean( v );
  return _retval_;
}

//
// static double median(vpRowVector v)
//
JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_median_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_median_10
(JNIEnv* env, jclass , jlong v_nativeObj)
{
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  double _retval_ = vpRowVector::median( v );
  return _retval_;
}

//
// static double stdev(vpRowVector v, bool useBesselCorrection = false)
//
JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_stdev_10 (JNIEnv*, jclass, jlong, jboolean);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_stdev_10
(JNIEnv* env, jclass , jlong v_nativeObj, jboolean useBesselCorrection)
{
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  double _retval_ = vpRowVector::stdev( v, (bool)useBesselCorrection );
  return _retval_;
}

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_stdev_11 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_stdev_11
(JNIEnv* env, jclass , jlong v_nativeObj)
{
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  double _retval_ = vpRowVector::stdev( v );
  return _retval_;
}

//
//  double sum()
//
JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_sum_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_sum_10
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  double _retval_ = me->sum(  );
  return _retval_;
}

//
//  double sumSquare()
//
JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_sumSquare_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jdouble JNICALL Java_org_visp_core_VpRowVector_sumSquare_10
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  double _retval_ = me->sumSquare(  );
  return _retval_;
}

//
//  void clear()
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_clear_10 (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_clear_10
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  me->clear(  );
  return;
}

//
//  void deg2rad()
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_deg2rad_10 (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_deg2rad_10
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  me->deg2rad(  );
  return;
}

//
//  void init(vpRowVector v, int c, int ncols)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_init_10 (JNIEnv*, jclass, jlong, jlong, jint, jint);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_init_10
(JNIEnv* env, jclass , jlong self, jlong v_nativeObj, jint c, jint ncols)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  me->init( v, (int)c, (int)ncols );
  return;
}

//
//  void insert(int i, vpRowVector v)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_insert_10 (JNIEnv*, jclass, jlong, jint, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_insert_10
(JNIEnv* env, jclass , jlong self, jint i, jlong v_nativeObj)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  me->insert( (int)i, v );
  return;
}

//
//  void rad2deg()
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_rad2deg_10 (JNIEnv*, jclass, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_rad2deg_10
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  me->rad2deg(  );
  return;
}

//
//  void reshape(vpMatrix M, int nrows, int ncols)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_reshape_10 (JNIEnv*, jclass, jlong, jlong, jint, jint);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_reshape_10
(JNIEnv* env, jclass , jlong self, jlong M_nativeObj, jint nrows, jint ncols)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpMatrix& M = *((vpMatrix*)M_nativeObj);
  me->reshape( M, (int)nrows, (int)ncols );
  return;
}

//
//  void resize(int i, bool flagNullify = true)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_resize_10 (JNIEnv*, jclass, jlong, jint, jboolean);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_resize_10
(JNIEnv* env, jclass , jlong self, jint i, jboolean flagNullify)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  me->resize( (int)i, (bool)flagNullify );
  return;
}

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_resize_11 (JNIEnv*, jclass, jlong, jint);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_resize_11
(JNIEnv* env, jclass , jlong self, jint i)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  me->resize( (int)i );
  return;
}

//
//  void resize(int nrows, int ncols, bool flagNullify)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_resize_12 (JNIEnv*, jclass, jlong, jint, jint, jboolean);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_resize_12
(JNIEnv* env, jclass , jlong self, jint nrows, jint ncols, jboolean flagNullify)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  me->resize( (int)nrows, (int)ncols, (bool)flagNullify );
  return;
}

//
//  void stack(double d)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_stack_10 (JNIEnv*, jclass, jlong, jdouble);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_stack_10
(JNIEnv* env, jclass , jlong self, jdouble d)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  me->stack( (double)d );
  return;
}

//
// static void stack(vpRowVector A, vpRowVector B, vpRowVector C)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_stack_11 (JNIEnv*, jclass, jlong, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_stack_11
(JNIEnv* env, jclass , jlong A_nativeObj, jlong B_nativeObj, jlong C_nativeObj)
{
  vpRowVector& A = *((vpRowVector*)A_nativeObj);
  vpRowVector& B = *((vpRowVector*)B_nativeObj);
  vpRowVector& C = *((vpRowVector*)C_nativeObj);
  vpRowVector::stack( A, B, C );
  return;
}

//
//  void stack(vpRowVector v)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_stack_12 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_stack_12
(JNIEnv* env, jclass , jlong self, jlong v_nativeObj)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpRowVector& v = *((vpRowVector*)v_nativeObj);
  me->stack( v );
  return;
}

//
//  void transpose(vpColVector v)
//
JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_transpose_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT void JNICALL Java_org_visp_core_VpRowVector_transpose_10
(JNIEnv* env, jclass , jlong self, jlong v_nativeObj)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpColVector& v = *((vpColVector*)v_nativeObj);
  me->transpose( v );
  return;
}

//
//  vpColVector t()
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_t_10 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_t_10
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpColVector _retval_ = me->t(  );
  return (jlong) new vpColVector(_retval_);
}

//
//  vpColVector transpose()
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_transpose_11 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_transpose_11
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpColVector _retval_ = me->transpose(  );
  return (jlong) new vpColVector(_retval_);
}

//
//  vpMatrix reshape(int nrows, int ncols)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_reshape_11 (JNIEnv*, jclass, jlong, jint, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_reshape_11
(JNIEnv* env, jclass , jlong self, jint nrows, jint ncols)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpMatrix _retval_ = me->reshape( (int)nrows, (int)ncols );
  return (jlong) new vpMatrix(_retval_);
}

//
//  vpRowVector extract(int c, int rowsize)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_extract_10 (JNIEnv*, jclass, jlong, jint, jint);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_extract_10
(JNIEnv* env, jclass , jlong self, jint c, jint rowsize)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpRowVector _retval_ = me->extract( (int)c, (int)rowsize );
  return (jlong) new vpRowVector(_retval_);
}

//
//  vpRowVector normalize(vpRowVector x)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_normalize_10 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_normalize_10
(JNIEnv* env, jclass , jlong self, jlong x_nativeObj)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpRowVector& x = *((vpRowVector*)x_nativeObj);
  vpRowVector _retval_ = me->normalize( x );
  return (jlong) new vpRowVector(_retval_);
}

//
//  vpRowVector normalize()
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_normalize_11 (JNIEnv*, jclass, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_normalize_11
(JNIEnv* env, jclass , jlong self)
{
  vpRowVector* me = (vpRowVector*) self; //TODO: check for NULL
  vpRowVector _retval_ = me->normalize(  );
  return (jlong) new vpRowVector(_retval_);
}

//
// static vpRowVector stack(vpRowVector A, vpRowVector B)
//
JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_stack_13 (JNIEnv*, jclass, jlong, jlong);

JNIEXPORT jlong JNICALL Java_org_visp_core_VpRowVector_stack_13
(JNIEnv* env, jclass , jlong A_nativeObj, jlong B_nativeObj)
{
  vpRowVector& A = *((vpRowVector*)A_nativeObj);
  vpRowVector& B = *((vpRowVector*)B_nativeObj);
  vpRowVector _retval_ = vpRowVector::stack( A, B );
  return (jlong) new vpRowVector(_retval_);
}

} // extern "C"
