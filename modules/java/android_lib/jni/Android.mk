LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)

include ViSP.mk

LOCAL_MODULE    := visp_java

MY_PREFIX := $(LOCAL_PATH)
MY_SOURCES := $(wildcard $(MY_PREFIX)/*.cpp)
LOCAL_SRC_FILES := $(MY_SOURCES:$(MY_PREFIX)%=%)

LOCAL_LDLIBS +=  -llog -ldl -ljnigraphics

include $(BUILD_SHARED_LIBRARY)
