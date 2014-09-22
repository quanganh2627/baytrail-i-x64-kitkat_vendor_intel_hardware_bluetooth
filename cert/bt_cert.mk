ifeq ($(CONFIG_USE_INTEL_CERT_BINARIES),true)

LOCAL_PATH := $(call my-dir)

# Bluetooth.apk
include $(CLEAR_VARS)
LOCAL_MODULE := Bluetooth
LOCAL_MODULE_TAGS := optional
#LOCAL_OVERRIDES_PACKAGES := Bluetooth
LOCAL_SRC_FILES := $(LOCAL_MODULE).apk
LOCAL_MODULE_CLASS := APPS
LOCAL_MODULE_SUFFIX := $(COMMON_ANDROID_PACKAGE_SUFFIX)
LOCAL_CERTIFICATE := platform
LOCAL_REQUIRED_MODULES := libbluetooth_jni bluetooth.default
include $(BUILD_PREBUILT)

# libbluetooth_jni.so
include $(CLEAR_VARS)
LOCAL_MODULE := libbluetooth_jni
LOCAL_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_SRC_FILES := $(LOCAL_MODULE).so
LOCAL_BUILT_MODULE_STEM := $(LOCAL_MODULE).so
include $(BUILD_PREBUILT)

# bluetooth.default.so
include $(CLEAR_VARS)
LOCAL_MODULE := bluetooth.default
LOCAL_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SRC_FILES := $(LOCAL_MODULE).so
LOCAL_BUILT_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_SHARED_LIBRARIES := \
    libcutils \
    liblog \
    libpower \
    libbt-hci \
    libbt-utils
LOCAL_REQUIRED_MODULES := libbt-hci libbt-vendor bt_stack.conf bt_did.conf auto_pair_devlist.conf
include $(BUILD_PREBUILT)

# libbt-hci.so
include $(CLEAR_VARS)
LOCAL_MODULE := libbt-hci
LOCAL_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_SRC_FILES := $(LOCAL_MODULE).so
LOCAL_BUILT_MODULE_STEM := $(LOCAL_MODULE).so
ifeq ($(BLUETOOTH_HCI_USE_USB),true)
LOCAL_SHARED_LIBRARIES := \
        libusb
endif
LOCAL_SHARED_LIBRARIES += \
        libcutils \
        liblog \
        libdl \
        libbt-utils
include $(BUILD_PREBUILT)

#libbt-utils
include $(CLEAR_VARS)
LOCAL_MODULE := libbt-utils
LOCAL_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_SRC_FILES := $(LOCAL_MODULE).so
LOCAL_BUILT_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_SHARED_LIBRARIES := libcutils liblog libc
include $(BUILD_PREBUILT)

# audio.a2dp.default.so
include $(CLEAR_VARS)
LOCAL_MODULE := audio.a2dp.default
LOCAL_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SRC_FILES := $(LOCAL_MODULE).so
LOCAL_BUILT_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_SHARED_LIBRARIES := libcutils liblog libpower
include $(BUILD_PREBUILT)

# audio.hsp.default.so
include $(CLEAR_VARS)
LOCAL_MODULE := audio.hsp.default
LOCAL_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_MODULE_TAGS := optional
LOCAL_MODULE_CLASS := SHARED_LIBRARIES
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw
LOCAL_SRC_FILES := $(LOCAL_MODULE).so
LOCAL_BUILT_MODULE_STEM := $(LOCAL_MODULE).so
LOCAL_SHARED_LIBRARIES := libcutils libaudioutils
include $(BUILD_PREBUILT)

endif # CONFIG_USE_INTEL_CERT_BINARIES
