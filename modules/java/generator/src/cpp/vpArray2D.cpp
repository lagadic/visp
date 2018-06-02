#include <jni.h>
#include <iostream>
#include <cstring>
#include <sstream>
#include "visp3/core/vpArray2D.h"


// Java Method:    vpArray2D(Class type)
JNIEXPORT jlong JNICALL Java_org_visp_core_vpArray2D_n_1vpArray2D__Ljava_lang_String_2
  (JNIEnv *env, jclass, jstring type);
JNIEXPORT jlong JNICALL Java_org_visp_core_vpArray2D_n_1vpArray2D__Ljava_lang_String_2
  (JNIEnv *env, jclass, jstring type){
	const char* cName = env -> GetStringUTFChars(type, NULL);
	
	if (!strcmp(cName,"class java.lang.Double"))
		return (jlong) new vpArray2D<double>();
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
}


// Java Method:    vpArray2D(Class type, int r, int c)
JNIEXPORT jlong JNICALL Java_org_visp_core_vpArray2D_n_1vpArray2D__Ljava_lang_String_2II
  (JNIEnv *env, jclass, jstring type, jint r, jint c);
JNIEXPORT jlong JNICALL Java_org_visp_core_vpArray2D_n_1vpArray2D__Ljava_lang_String_2II
  (JNIEnv *env, jclass, jstring type, jint r, jint c){
	const char* cName = env -> GetStringUTFChars(type, NULL);
	
	if (!strcmp(cName,"class java.lang.Double"))
		return (jlong) new vpArray2D<double>(r,c);
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
	
}

// Java Method:    vpArray2D(Class type, int r, int c, String val)
// I could have Object or jobject val. But casting to double or int is easier in strings than in objects
JNIEXPORT jlong JNICALL Java_org_visp_core_vpArray2D_n_1vpArray2D__Ljava_lang_String_2IILjava_lang_Object_2
  (JNIEnv *env, jclass, jstring type, jint r, jint c, jstring value){
	const char* cName = env -> GetStringUTFChars(type, NULL);
	const char* val = env -> GetStringUTFChars(value, NULL);
	
	if (!strcmp(cName,"class java.lang.Double")){
		double d;
		std::stringstream s(val);
		s >> d;
		return (jlong) new vpArray2D<double>(r,c, d);
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
	
}

// Java Method:    getCols(Class type)
JNIEXPORT jint JNICALL Java_org_visp_core_vpArray2D_n_1cols
  (JNIEnv *env, jclass, jstring type, jlong address);
JNIEXPORT jint JNICALL Java_org_visp_core_vpArray2D_n_1cols
  (JNIEnv *env, jclass, jstring type, jlong address){
	const char* cName = env -> GetStringUTFChars(type, NULL);
			
	if (!strcmp(cName,"class java.lang.Double")){
		vpArray2D<double>* me = (vpArray2D<double>*) address; //TODO: check for NULL
		return me->getCols();
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
}

// Java Method:    getRows(Class type)
JNIEXPORT jint JNICALL Java_org_visp_core_vpArray2D_n_1rows
  (JNIEnv *env, jclass, jstring type, jlong address);
JNIEXPORT jint JNICALL Java_org_visp_core_vpArray2D_n_1rows
  (JNIEnv *env, jclass, jstring type, jlong address){
	const char* cName = env -> GetStringUTFChars(type, NULL);
		
	if (!strcmp(cName,"class java.lang.Double")){
		vpArray2D<double>* me = (vpArray2D<double>*) address; //TODO: check for NULL
		return me->getRows();
	}
	else
		throw std::runtime_error(std::string(cName)+ ": No support added for this class");
}
