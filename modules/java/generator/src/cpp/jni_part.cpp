// This file is part of ViSP project.
#include "common.h"

#ifdef HAVE_VISP_FEATURES2D
#  include "visp/features2d.hpp"
#endif

#ifdef HAVE_VISP_VIDEO
#  include "visp/video.hpp"
#endif

#ifdef HAVE_VISP_CONTRIB
#  include "visp/contrib.hpp"
#endif

// The file just gets JNI version. No relation to ViSP project's source code

extern "C" {

JNIEXPORT jint JNICALL
JNI_OnLoad(JavaVM* vm, void* )
{
    JNIEnv* env;
    if (vm->GetEnv((void**) &env, JNI_VERSION_1_6) != JNI_OK)
        return -1;

    /* get class with (*env)->FindClass */
    /* register methods with (*env)->RegisterNatives */

    return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL
JNI_OnUnload(JavaVM*, void*)
{
  //do nothing
}

} // extern "C"
