ifeq ($(CONFIG_USE_INTEL_CERT_CORE_STACK), true)
PRODUCT_COPY_FILES += \
        vendor/intel/hardware/bluetooth/cert/bluedroid/bluetooth.default.so:system/lib/hw/bluetooth.default.so \
        vendor/intel/hardware/bluetooth/cert/bluedroid/audio.a2dp.default.so:system/lib/hw/audio.a2dp.default.so \
        vendor/intel/hardware/bluetooth/cert/bluedroid/audio.hsp.default.so:system/lib/hw/audio.hsp.default.so \
        vendor/intel/hardware/bluetooth/cert/bluedroid/libbt-hci.so:system/lib/libbt-hci.so
endif

