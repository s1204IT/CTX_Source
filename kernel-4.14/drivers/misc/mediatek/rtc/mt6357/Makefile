#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

ccflags-y += -I$(srctree)/drivers/misc/mediatek/pmic/
ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6357) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6357
ccflags-$(CONFIG_MTK_PMIC_CHIP_MT6390) += -I$(srctree)/drivers/misc/mediatek/pmic/include/mt6357

obj-$(CONFIG_MTK_RTC) := mtk_rtc_hal.o

