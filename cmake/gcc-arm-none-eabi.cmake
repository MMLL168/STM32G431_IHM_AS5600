set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

if(WIN32)
    set(_ARM_GCC_HINTS
        "C:/ST/STM32CubeIDE_1.19.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.13.3.rel1.win32_1.0.0.202411081344/tools/bin"
        "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/14.2 rel1/bin"
        "C:/Program Files (x86)/Arm GNU Toolchain arm-none-eabi/13.3 rel1/bin"
        "C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin"
    )
endif()

find_program(ARM_NONE_EABI_GCC
    NAMES arm-none-eabi-gcc arm-none-eabi-gcc.exe
    HINTS ${_ARM_GCC_HINTS}
)

if(NOT ARM_NONE_EABI_GCC)
    message(FATAL_ERROR "arm-none-eabi-gcc was not found. Install Arm GNU Toolchain or add it to PATH.")
endif()

get_filename_component(TOOLCHAIN_BIN_DIR "${ARM_NONE_EABI_GCC}" DIRECTORY)

find_program(ARM_NONE_EABI_GXX
    NAMES arm-none-eabi-g++ arm-none-eabi-g++.exe
    HINTS "${TOOLCHAIN_BIN_DIR}"
    NO_DEFAULT_PATH
)
find_program(ARM_NONE_EABI_OBJCOPY
    NAMES arm-none-eabi-objcopy arm-none-eabi-objcopy.exe
    HINTS "${TOOLCHAIN_BIN_DIR}"
    NO_DEFAULT_PATH
)
find_program(ARM_NONE_EABI_SIZE
    NAMES arm-none-eabi-size arm-none-eabi-size.exe
    HINTS "${TOOLCHAIN_BIN_DIR}"
    NO_DEFAULT_PATH
)

set(CMAKE_C_COMPILER                "${ARM_NONE_EABI_GCC}")
set(CMAKE_ASM_COMPILER              ${CMAKE_C_COMPILER})
set(CMAKE_CXX_COMPILER              "${ARM_NONE_EABI_GXX}")
set(CMAKE_LINKER                    "${ARM_NONE_EABI_GXX}")
set(CMAKE_OBJCOPY                   "${ARM_NONE_EABI_OBJCOPY}")
set(CMAKE_SIZE                      "${ARM_NONE_EABI_SIZE}")

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_C_COMPILER_WORKS TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_WORKS TRUE)
set(CMAKE_ASM_COMPILER_FORCED TRUE)
set(CMAKE_ASM_COMPILER_WORKS TRUE)

# MCU specific flags
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections -fstack-usage")

# The cyclomatic-complexity parameter must be defined for the Cyclomatic complexity feature in STM32CubeIDE to work.
# However, most GCC toolchains do not support this option, which causes a compilation error; for this reason, the feature is disabled by default.
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fcyclomatic-complexity")

# This project runs a 30 kHz motor-control loop on STM32G431.
# `-O0` is too slow for the FOC ISR and triggers MC_DURATION / FOC Duration faults,
# so keep debug symbols while using an optimization level suitable for real-time control.
set(CMAKE_C_FLAGS_DEBUG "-O2 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O2 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32G431XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
