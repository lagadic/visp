#include <jni.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include "visp3/core/vpImage.h"
typedef unsigned char u_char;


// Java Method:    vpImage(Class type)

JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2
  (JNIEnv *env, jclass, jstring type);
JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2
  (JNIEnv *env, jclass, jstring type){
	const char* cName = env -> GetStringUTFChars(type, NULL);
	
	if (!strcmp(cName,"class java.lang.Byte"))
		return (jlong) new vpImage<u_char>();
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
}


// Java Method:    vpImage(Class type, int r, int c)
JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2II
  (JNIEnv *env, jclass, jstring type, jint r, jint c);
JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2II
  (JNIEnv *env, jclass, jstring type, jint r, jint c){
	const char* cName = env -> GetStringUTFChars(type, NULL);
	
	if (!strcmp(cName,"class java.lang.Byte"))
		return (jlong) new vpImage<u_char>(r,c);
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
	
}

// Java Method:    vpImage(Class type, int r, int c, String val)
// I could have Object or jobject val. But casting to u_char or int is easier in strings than in objects
JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2IILjava_lang_Object_2
  (JNIEnv *env, jclass, jstring type, jint r, jint c, jstring value);
JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2IILjava_lang_Object_2
  (JNIEnv *env, jclass, jstring type, jint r, jint c, jstring value){
	const char* cName = env -> GetStringUTFChars(type, NULL);
	const char* val = env -> GetStringUTFChars(value, NULL);
	
	if (!strcmp(cName,"class java.lang.Byte")){
		u_char d;
		std::stringstream s(val);
		s >> d;
		return (jlong) new vpImage<u_char>(r,c, d);
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
	
}

// Java Method:    vpImage(Class<T> type, byte[] array, int height, int width, boolean copyData)

JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2_3BIIZ
  (JNIEnv *env, jclass, jstring type, jbyteArray arr, jint h, jint w, jboolean copyData);
JNIEXPORT jlong JNICALL Java_org_visp_core_vpImage_n_1vpImage__Ljava_lang_String_2_3BIIZ
  (JNIEnv *env, jclass, jstring type, jbyteArray arr, jint h, jint w, jboolean copyData){
	const char* cName = env -> GetStringUTFChars(type, NULL);
	jbyte *array = env->GetByteArrayElements(arr, NULL);
	
	if (!strcmp(cName,"class java.lang.Byte")){
		return (jlong) new vpImage<u_char>((u_char *const) array, (const unsigned int) h, (const unsigned int) w, copyData);
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
	
	// be memory friendly
	env->ReleaseByteArrayElements(arr, array, 0);
}

// Java Method:    getCols(Class type)
JNIEXPORT jint JNICALL Java_org_visp_core_vpImage_n_1cols
  (JNIEnv *env, jclass, jstring type, jlong address);
JNIEXPORT jint JNICALL Java_org_visp_core_vpImage_n_1cols
  (JNIEnv *env, jclass, jstring type, jlong address){
	const char* cName = env -> GetStringUTFChars(type, NULL);
			
	if (!strcmp(cName,"class java.lang.Byte")){
		vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
		return me->getCols();
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
}

// Java Method:    getRows(Class type)
JNIEXPORT jint JNICALL Java_org_visp_core_vpImage_n_1rows
  (JNIEnv *env, jclass, jstring type, jlong address);
JNIEXPORT jint JNICALL Java_org_visp_core_vpImage_n_1rows
  (JNIEnv *env, jclass, jstring type, jlong address){
	const char* cName = env -> GetStringUTFChars(type, NULL);
		
	if (!strcmp(cName,"class java.lang.Byte")){
		vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
		return me->getRows();
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
}

// Java Method:    dump()
JNIEXPORT jstring JNICALL Java_org_visp_core_vpImage_n_1dump
  (JNIEnv *env, jclass, jstring type, jlong address);
JNIEXPORT jstring JNICALL Java_org_visp_core_vpImage_n_1dump
  (JNIEnv *env, jclass, jstring type, jlong address){
	const char* cName = env -> GetStringUTFChars(type, NULL);
		
	if (!strcmp(cName,"class java.lang.Byte")){
		vpImage<u_char>* me = (vpImage<u_char>*) address; //TODO: check for NULL
		std::stringstream ss;
		ss << *me;
		return env->NewStringUTF(ss.str().c_str());
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
}
