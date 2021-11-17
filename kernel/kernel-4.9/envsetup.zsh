#!/bin/zsh

CROSS_GCC_PATH=$HOME/buildtool/l4t_gcc
cur_dir=$(pwd)

if [ -d "${CROSS_GCC_PATH}" ]; then
	echo "--ENV : cross gccis exist, env is ok"
else
	echo "--ENV : cross gcc not exist ; now download cross tools";
	wget http://releases.linaro.org/components/toolchain/binaries/7.3-2018.05/aarch64-linux-gnu/gcc-linaro-7.3.1-2018.05-x86_64_aarch64-linux-gnu.tar.xz -o gcc.tar.xz;
	tar -xf gcc.tar.xz -C $HOME/buildtool/l4t_gcc
fi
#set ENV
export LOCALVERSION=-tegra
export TEGRA_KERNEL_OUT=$cur_dir/okernel
export CROSS_COMPILE=$CROSS_GCC_PATH/bin/aarch64-linux-gnu-
mkdir -p okernel
