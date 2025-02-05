#include <visp3/imgproc/vpContours.h>
#include <visp3/imgproc/vpImgproc.h>
using namespace std;

extern "C" {

#if !defined(__ppc__)
// to suppress warning from jni.h on OS X
#define TARGET_RT_MAC_CFM 0
#endif
#include <jni.h>

//
//   vpContour()
//

  JNIEXPORT jlong JNICALL Java_org_visp_imgproc_VpContour_VpContour1(JNIEnv *, jclass);

  JNIEXPORT jlong JNICALL Java_org_visp_imgproc_VpContour_VpContour1(JNIEnv *env, jclass)
  {
    (void)env;
    visp::vpContour *_retval_ = new visp::vpContour();
    return (jlong)_retval_;
  }

  //
  //   vpContour(vpContourType type)
  //

  JNIEXPORT jlong JNICALL Java_org_visp_imgproc_VpContour_VpContour2(JNIEnv *, jclass, jint);

  JNIEXPORT jlong JNICALL Java_org_visp_imgproc_VpContour_VpContour2(JNIEnv *env, jclass, jint type)
  {
    (void)env;
    if (type == 0)
      return (jlong) new visp::vpContour(visp::CONTOUR_OUTER);
    else
      return (jlong) new visp::vpContour(visp::CONTOUR_HOLE);
  }

  //
  //   vpContour(vpContour contour)
  //

  JNIEXPORT jlong JNICALL Java_org_visp_imgproc_VpContour_VpContour3(JNIEnv *, jclass, jlong);

  JNIEXPORT jlong JNICALL Java_org_visp_imgproc_VpContour_VpContour3(JNIEnv *env, jclass, jlong address)
  {
    (void)env;
    visp::vpContour *other = (visp::vpContour *)address;
    visp::vpContour *_retval_ = new visp::vpContour(*other);
    return (jlong)_retval_;
  }

  //
  //   void setParent(vpContour *parent)
  //

  JNIEXPORT void JNICALL Java_org_visp_imgproc_VpContour_n_1setParent(JNIEnv *, jclass, jlong, jlong);

  JNIEXPORT void JNICALL Java_org_visp_imgproc_VpContour_n_1setParent(JNIEnv *env, jclass, jlong address_self,
                                                                      jlong address_parent)
  {
    (void)env;
    visp::vpContour *self = (visp::vpContour *)address_self;
    visp::vpContour *parent = (visp::vpContour *)address_parent;
    self->setParent(parent);
  }

  //
  //  native support for java finalize()
  //  static void vpContour::delete( __int64 self )
  //

  JNIEXPORT void JNICALL Java_org_visp_imgproc_VpContour_delete(JNIEnv *, jclass, jlong);

  JNIEXPORT void JNICALL Java_org_visp_imgproc_VpContour_delete(JNIEnv *, jclass, jlong self)
  {
    delete (visp::vpContour *)self;
  }

} // extern "C"
