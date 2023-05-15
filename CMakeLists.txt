#此文件从模板自动生成! 请勿更改!
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
cmake_minimum_required(VERSION 3.23)
#set(CMAKE_C_COMPILER /path/to/your/compiler)
#set(CMAKE_CXX_COMPILER /path/to/your/compiler)
#set(CMAKE_ASM_COMPILER /path/to/your/compiler)

# specify cross-compilers and tools
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER  arm-none-eabi-gcc)
#set(CMAKE_C_COMPILER /home/pansy/arm-gnu/arm-gnu/bin/arm-none-eabi-gcc)
#set(CMAKE_CXX_COMPILER /home/pansy/arm-gnu/arm-gnu/bin/arm-none-eabi-g++)
#set(CMAKE_ASM_COMPILER  /home/pansy/arm-gnu/arm-gnu/bin/arm-none-eabi-gcc)
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(SIZE arm-none-eabi-size)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# project settings
project(Engineering_A_2023_C_Board C CXX ASM)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

#Uncomment for hardware floating point
add_compile_definitions(ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING)
add_compile_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)
add_link_options(-mfloat-abi=hard -mfpu=fpv4-sp-d16)

#Uncomment for software floating point
#add_compile_options(-mfloat-abi=soft)

add_compile_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_compile_options(-ffunction-sections -fdata-sections -fno-common -fmessage-length=0)

# uncomment to mitigate c++17 absolute addresses warnings
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-register")

# Enable assembler files preprocessing
add_compile_options($<$<COMPILE_LANGUAGE:ASM>:-x$<SEMICOLON>assembler-with-cpp>)

add_definitions(-D__FPU_PRESENT=1U)
add_definitions(-D__FPU_USED=1U)
if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()
#Bsp/Inc ACE-ECF/Bsp/Inc
include_directories(
        Task/Inc
        Control/Inc
        ACE-ECF/Algorithm/inc
        Bsp/Inc
        Core/Inc
        Middlewares/ST/ARM/DSP/Inc
        Middlewares/ST/STM32_USB_Device_Library/
        Middlewares/Third_Party/FreeRTOS/Source/include
#        Middlewares/Third_Party/FreeRTOS/Source
        Middlewares/Third_Party/FreeRTOS/Source/portable/RVDS/ARM_CM4F
        Core/Inc Drivers/STM32F4xx_HAL_Driver/Inc Drivers/STM32F4xx_HAL_Driver/Inc/Legacy Middlewares/Third_Party/FreeRTOS/Source/include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F Drivers/CMSIS/Device/ST/STM32F4xx/Include Drivers/CMSIS/Include USB_DEVICE/App USB_DEVICE/Target Middlewares/ST/STM32_USB_Device_Library/Core/Inc Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc)

add_definitions(-DUSE_HAL_DRIVER -DSTM32F407xx)

file(GLOB_RECURSE SOURCES "USB_DEVICE/*.*" "startup/*.*" "Middlewares/*.*" "Drivers/*.*" "Core/*.*"
        "Bsp/Src/*.*"
        "Task/Src/*.*"
        "Control/Src/*.*"
#        "Algorithm/Src/*.*"
#        "ACE-ECF/Bsp/Src/*.*"
        "ACE-ECF/Algorithm/src/*.*")

set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32F407IGHx_FLASH.ld Control/Src/AutoAction.c Control/Inc/AutoAction.h Bsp/Src/bsp_pwm.c Bsp/Inc/bsp_pwm.h Bsp/Src/bsp_Power_Detect.c Bsp/Inc/bsp_Power_Detect.h ACE-ECF/Bsp/Src/Ina226.c ACE-ECF/Bsp/Src/Ina226.c ACE-ECF/Bsp/Inc/Ina226.h ACE-ECF/Bsp/Inc/Ina226.h BoardConfig.h)

add_link_options(-Wl,-gc-sections,--print-memory-usage,-Map=${PROJECT_BINARY_DIR}/${PROJECT_NAME}.map)
add_link_options(-mcpu=cortex-m4 -mthumb -mthumb-interwork)
add_link_options(-T ${LINKER_SCRIPT})

add_executable(${PROJECT_NAME}.elf ${SOURCES} ${LINKER_SCRIPT})

set(HEX_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${PROJECT_NAME}.bin)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -Oihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${HEX_FILE}
        COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}.elf> ${BIN_FILE}
        COMMENT "Building ${HEX_FILE}
Building ${BIN_FILE}")
