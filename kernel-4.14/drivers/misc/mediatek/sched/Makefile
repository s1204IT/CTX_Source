#
# Copyright (C) 2017 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#

LINUXINCLUDE += -include $(srctree)/kernel/sched/sched.h

# For RT monitor
obj-$(CONFIG_MTK_RT_THROTTLE_MON) += rt_monitor.o

# RQ stats for TLP estimation
obj-$(CONFIG_MTK_SCHED_RQAVG_US) += rq_stats.o
obj-$(CONFIG_MTK_SCHED_RQAVG_KS) += sched_avg.o

# For CPU topology to user space
obj-y += sched_ctl.o

# sysfs interface for user to get EAS related info
obj-y += eas_plus.o

ccflags-y += -DMET_USER_EVENT_SUPPORT
