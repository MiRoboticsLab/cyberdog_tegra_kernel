#!/bin/sh

CROSS_GCC_PATH=$HOME/buildtool/l4t_gcc
COMPILER="l4t_gcc_7.3.1.tgz"
cur_dir=$(pwd)

if [ -d "${CROSS_GCC_PATH}" ]; then
	echo "--ENV : cross gcc is exist, env is ok"
else
	echo "--ENV : cross gcc not exist ; now download cross tools";
	wget https://cdn.cnbj2m.fds.api.mi-img.com/cyberdog-package/build/${COMPILER}
	mkdir -p $HOME/buildtool/l4t_gcc
	tar -xf ${COMPILER} -C $HOME/buildtool/
	ls $CROSS_GCC_PATH
fi
#set ENV
export LOCALVERSION=-tegra
export TEGRA_KERNEL_OUT=$cur_dir/okernel
export CROSS_COMPILE=$CROSS_GCC_PATH/bin/aarch64-linux-gnu-
mkdir -p okernel
