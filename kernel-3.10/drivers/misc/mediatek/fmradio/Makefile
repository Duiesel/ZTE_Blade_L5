# fm support
include $(srctree)/drivers/misc/mediatek/Makefile.custom
ifeq ($(CONFIG_MTK_FM_SUPPORT), y)

subdir-ccflags-y += -D MTK_WCN_REMOVE_KERNEL_MODULE
subdir-ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/include

#define build_speical_fm_driver

# MT6628 FM driver
#ifeq ($(CONFIG_MTK_FM_CHIP), MT6628_FM)

$(warning feature_option=$(CONFIG_MTK_FM_CHIP))
ifeq ($(findstring MT6628_FM,$(CONFIG_MTK_FM_CHIP)),MT6628_FM)
$(warning MT6628_FM)
	FM_CHIP := mt6628
	FM_CHIP_PATH := $(FM_CHIP)/pub/$(FM_CHIP)
	ccflags-y += -I$(src)/inc \
				-I$(src)/$(FM_CHIP)/inc \
				-I$(src)/../connectivity/combo/common/include \
				-I$(src)/../connectivity/combo/common/linux/include \
				-I$(src)/../connectivity/common_detect
					
obj-$(CONFIG_MTK_FM) += mtk_fm_drv.o
	mt6628-objs	:= core/fm_module.o \
				core/fm_main.o \
				core/fm_config.o \
				core/fm_rds_parser.o \
				core/fm_patch.o \
				core/fm_utils.o \
				core/fm_link.o \
				core/fm_eint.o  \
				$(FM_CHIP_PATH)_fm_lib.o \
				$(FM_CHIP_PATH)_fm_rds.o \
				$(FM_CHIP_PATH)_fm_cmd.o \
				$(FM_CHIP_PATH)_fm_config.o
	mtk_fm_drv-objs += $(mt6628-objs)
$(warning $(mtk_fm_drv-objs))
	#obj-$(CONFIG_MTK_FM) += private/
endif

# MT6620 FM driver
#ifeq ($(CONFIG_MTK_FM_CHIP), MT6620_FM)

ifeq ($(findstring MT6620_FM,$(CONFIG_MTK_FM_CHIP)),MT6620_FM)
$(warning MT6620_FM)
	FM_CHIP := mt6620
	FM_CHIP_PATH := $(FM_CHIP)/pub/$(FM_CHIP)
	ccflags-y += -I$(src)/inc \
					-I$(src)/$(FM_CHIP)/inc \
					-I$(src)/../connectivity/combo/common/include \
					-I$(src)/../connectivity/combo/common/linux/include \
					-I$(src)/../connectivity/common_detect

	obj-$(CONFIG_MTK_FM) += mtk_fm_drv.o
	mt6620-objs	:= core/fm_module.o \
					core/fm_main.o \
					core/fm_config.o \
					core/fm_rds_parser.o \
					core/fm_patch.o \
					core/fm_utils.o \
					core/fm_link.o \
					core/fm_eint.o \
					$(FM_CHIP_PATH)_fm_lib.o \
					$(FM_CHIP_PATH)_fm_rds.o \
					$(FM_CHIP_PATH)_fm_cmd.o \
					$(FM_CHIP_PATH)_fm_config.o
	mtk_fm_drv-objs += $(mt6620-objs)
$(warning $(mtk_fm_drv-objs))
	#obj-$(CONFIG_MTK_FM) += private/
endif
#$(foreach each_chip, $(CONFIG_MTK_FM_CHIP),$(eval $(call build_speical_fm_driver,$(each_chip))))
#endef

# MT6627 FM driver

ifeq ($(findstring MT6627_FM,$(CONFIG_MTK_FM_CHIP)),MT6627_FM)
$(warning MT6627_FM)
	FM_CHIP := mt6627
	FM_CHIP_PATH := $(FM_CHIP)/pub/$(FM_CHIP)
	ccflags-y += -DMT6627_FM
	ccflags-y += -I$(src)/inc \
					-I$(src)/$(FM_CHIP)/inc \
					-I$(src)/../connectivity/conn_soc/common/include \
					-I$(src)/../connectivity/conn_soc/common/linux/include \
					-I$(src)/../connectivity/common_detect

	obj-$(CONFIG_MTK_FM) += mtk_fm_drv.o
	mt6627-objs	:= core/fm_module.o \
					core/fm_main.o \
					core/fm_config.o \
					core/fm_rds_parser.o \
					core/fm_patch.o \
					core/fm_utils.o \
					core/fm_link.o \
					core/fm_eint.o \
					$(FM_CHIP_PATH)_fm_lib.o \
					$(FM_CHIP_PATH)_fm_rds.o \
					$(FM_CHIP_PATH)_fm_cmd.o \
					$(FM_CHIP_PATH)_fm_config.o
	mtk_fm_drv-objs += $(mt6627-objs)
$(warning $(mtk_fm_drv-objs))
	#obj-$(CONFIG_MTK_FM) += private/
endif

# MT6625 FM driver

ifeq ($(findstring MT6625_FM,$(CONFIG_MTK_FM_CHIP)),MT6625_FM)
$(warning MT6627_FM)
	FM_CHIP := mt6627
	FM_CHIP_PATH := $(FM_CHIP)/pub/$(FM_CHIP)
	ccflags-y += -DMT6627_FM
	ccflags-y += -DMT6625_FM
	ccflags-y += -I$(src)/inc \
					-I$(src)/$(FM_CHIP)/inc \
					-I$(src)/../connectivity/conn_soc/common/include \
					-I$(src)/../connectivity/conn_soc/common/linux/include \
					-I$(src)/../connectivity/common_detect

	obj-$(CONFIG_MTK_FM) += mtk_fm_drv.o
	mt6627-objs	:= core/fm_module.o \
					core/fm_main.o \
					core/fm_config.o \
					core/fm_rds_parser.o \
					core/fm_patch.o \
					core/fm_utils.o \
					core/fm_link.o \
					core/fm_eint.o \
					$(FM_CHIP_PATH)_fm_lib.o \
					$(FM_CHIP_PATH)_fm_rds.o \
					$(FM_CHIP_PATH)_fm_cmd.o \
					$(FM_CHIP_PATH)_fm_config.o
	mtk_fm_drv-objs += $(mt6627-objs)
$(warning $(mtk_fm_drv-objs))
	#obj-$(CONFIG_MTK_FM) += private/
endif

# MT6580 FM driver

ifeq ($(findstring MT6580_FM,$(CONFIG_MTK_FM_CHIP)),MT6580_FM)
$(warning MT6580_FM)
	FM_CHIP := mt6580
	FM_CHIP_PATH := $(FM_CHIP)/pub/$(FM_CHIP)
	ccflags-y += -DMT6580_FM
	ccflags-y += -I$(src)/inc \
					-I$(src)/$(FM_CHIP)/inc \
					-I$(src)/../connectivity/conn_soc/common/include \
					-I$(src)/../connectivity/conn_soc/common/linux/include \
					-I$(src)/../connectivity/common_detect

	obj-$(CONFIG_MTK_FM) += mtk_fm_drv.o
	mt6580-objs	:= core/fm_module.o \
					core/fm_main.o \
					core/fm_config.o \
					core/fm_rds_parser.o \
					core/fm_patch.o \
					core/fm_utils.o \
					core/fm_link.o \
					core/fm_eint.o \
					$(FM_CHIP_PATH)_fm_lib.o \
					$(FM_CHIP_PATH)_fm_rds.o \
					$(FM_CHIP_PATH)_fm_cmd.o \
					$(FM_CHIP_PATH)_fm_config.o
	mtk_fm_drv-objs += $(mt6580-objs)
$(warning $(mtk_fm_drv-objs))
	#obj-$(CONFIG_MTK_FM) += private/
endif

# MT6630 FM driver
ifeq ($(findstring MT6630_FM,$(CONFIG_MTK_FM_CHIP)),MT6630_FM)
$(warning MT6630_FM)
	FM_CHIP := mt6630
	FM_CHIP_PATH := $(FM_CHIP)/pub/$(FM_CHIP)
    ccflags-y += -DMT6630_FM
	ccflags-y += -I$(src)/inc \
					-I$(src)/$(FM_CHIP)/inc \
					-I$(src)/../connectivity/combo/common/include \
					-I$(src)/../connectivity/combo/common/linux/include \
					-I$(src)/../connectivity/common_detect

	obj-$(CONFIG_MTK_FM) += mtk_fm_drv.o
	mt6630-objs	:= core/fm_module.o \
					core/fm_main.o \
					core/fm_config.o \
					core/fm_rds_parser.o \
					core/fm_patch.o \
					core/fm_utils.o \
					core/fm_link.o \
					core/fm_eint.o \
					$(FM_CHIP_PATH)_fm_lib.o \
					$(FM_CHIP_PATH)_fm_rds.o \
					$(FM_CHIP_PATH)_fm_cmd.o \
					$(FM_CHIP_PATH)_fm_config.o
	mtk_fm_drv-objs += $(mt6630-objs)
$(warning $(mtk_fm_drv-objs))
	#obj-$(CONFIG_MTK_FM) += private/
endif

# MT6626 FM driver 
ifeq ($(CONFIG_MTK_FM_CHIP), mt6626)
FM_CHIP := mt6626
FM_CHIP_PATH := $(FM_CHIP)/pub/$(FM_CHIP)
ccflags-y := -I$(src)/inc \
				-I$(src)/$(FM_CHIP)/inc \
				-I$(src)/cust/$(FM_CHIP) \
				-I$(src)/private/inc

obj-$(CONFIG_MTK_FM) += mtk_fm_drv.o
mtk_fm_drv-objs	:= core/fm_module.o \
				core/fm_main.o \
				core/fm_config.o \
				core/fm_patch.o \
				core/fm_rds_parser.o \
				core/fm_utils.o \
				core/fm_link.o \
				$(FM_CHIP_PATH)_fm_lib.o \
				$(FM_CHIP_PATH)_fm_rds.o \
				$(FM_CHIP_PATH)_fm_link.o \
				$(FM_CHIP_PATH)_fm_eint.o
endif

#for include WMT header
subdir-ccflags-y += -DWMT_PLAT_ALPS=1

endif

obj-y += dummy.o

