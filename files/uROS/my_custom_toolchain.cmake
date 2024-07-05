set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_ASM_COMPILER_WORKS TRUE)
set(CMAKE_SYSTEM_PROCESSOR microblaze_0)

# SET HERE THE PATH TO YOUR C99 AND C++ COMPILERS
set(CMAKE_C_COMPILER /tools/Xilinx/Vitis/2022.2/gnu/microblaze/lin/bin/mb-gcc)
set(CMAKE_CXX_COMPILER /tools/Xilinx/Vitis/2022.2/gnu/microblaze/lin/bin/mb-g++)

set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

# SET HERE YOUR BUILDING FLAGS
set(FLAGS "-Wall -O0 -g3 -c -fmessage-length=0 -mlittle-endian -mxl-pattern-compare -mcpu=v11.0 -mno-xl-reorder -mno-xl-soft-mul -mxl-barrel-shift -mxl-pattern-compare" CACHE STRING "" FORCE)


set(CMAKE_C_FLAGS_INIT "-std=c11 ${FLAGS} -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-std=c++11 ${FLAGS} -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

set(__LITTLE_ENDIAN__ 0)

