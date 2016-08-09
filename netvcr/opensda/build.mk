
OPENSDASRC += \
                 $(OPENSDA)/bootloader/Common/src/version.c \
                 $(OPENSDA)/bootloader/Common/src/usbd_user_msc.c \
                 $(OPENSDA)/bootloader/hal/TARGET_Freescale/TARGET_MK20DX/read_uid.c \

OPENSDAINC += $(OPENSDA)/bootloader/Common/inc \
              $(OPENSDA)/shared/cmsis/TARGET_Freescale/TARGET_MK20DX \
              $(OPENSDA)/shared/rtos \
              $(OPENSDA)/shared/USBStack/INC \

