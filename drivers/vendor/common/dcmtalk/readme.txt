dcm模块编译
===========

# 设置交叉编译器路径
--------------------
export PATH=/data/android/zt_A12/prebuilts/clang/host/linux-x86/clang-r416183b/bin:$PATH

# 修改Makefile中的kernel路径：
-----------------------------
KDIR   := /data/android/code/linux-5.10.160

# 设置参数，交叉编译
--------------------
make CROSS_COMPILE=aarch64-linux-gnu- LLVM=1 LLVM_IAS=1 ARCH=arm64

