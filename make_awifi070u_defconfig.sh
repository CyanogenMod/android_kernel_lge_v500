#!/bin/sh

#[START] make awifi070u-perf_defconfig based on awifi-perf_defconfig and remove some features that not used in awifi070u . gunmin.lee@lge.com
cat kernel/arch/arm/configs/awifi-perf_defconfig | grep -v "CONFIG_ANDROID_SW_IRRC=y" > kernel/arch/arm/configs/awifi070u-perf_defconfig
#[END] make awifi070u-perf_defconfig based on awifi-perf_defconfig and remove some features that not used in awifi070u . gunmin.lee@lge.com

#[START] append featues in awifi070u-perf_defconfig
cat kernel/arch/arm/configs/awifi070u_append_features >> kernel/arch/arm/configs/awifi070u-perf_defconfig

#[END] append featues in awifi070u-perf_defconfig
