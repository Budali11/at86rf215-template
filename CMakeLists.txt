cmake_minimum_required(VERSION 3.10)

# Project name and version
project(sam4l-reb215 VERSION 1.0 LANGUAGES C)

# export complie options
set(CMAKE_EXPORT_COMPILE_COMMANDS true)

# Specify the target device
set(DEVICE atsam4lc8c4)

# Specify the toolchain file
# Note: You need to create a toolchain file named `arm-none-eabi-toolchain.cmake`
# set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arm-none-eabi-toolchain.cmake)

# Define the cross-compiler tools
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross-compiler
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# Specify the cross-archiver
set(CMAKE_AR arm-none-eabi-ar)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_SIZE arm-none-eabi-size)

# Specify other tools
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Specify the standard to use for C
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(DFP_dir /home/budali11/workbench/atmel/SAM4L_DFP/1.1.61/sam4l8)
set(CMSIS_dir /home/budali11/workbench/atmel/CMSIS/5.4.0)
set(asf_dir /home/budali11/workbench/atmel/xdk-asf-3.52.0)
set(openocd_root /usr/local/share/openocd)
set(compiler_root /home/budali11/arm-none-eabi-13.3.rel1)

# Specify source files
set(SOURCES
  src/main.c
  src/at86rf215.c
  ${asf_dir}/sam/utils/cmsis/sam4l/source/templates/exceptions.c
  ${asf_dir}/sam/utils/cmsis/sam4l/source/templates/gcc/startup_sam4l.c
  ${asf_dir}/sam/utils/cmsis/sam4l/source/templates/system_sam4l.c
  ${asf_dir}/sam/boards/sam4l8_xplained_pro/init.c
  ${asf_dir}/sam/drivers/usart/usart.c
  ${asf_dir}/sam/drivers/spi/spi.c
  ${asf_dir}/sam/drivers/flashcalw/flashcalw.c
  ${asf_dir}/sam/drivers/bpm/bpm.c
  ${asf_dir}/sam/drivers/wdt/wdt_sam4l.c
  ${asf_dir}/sam/drivers/gpio/gpio.c
  ${asf_dir}/sam/utils/syscalls/gcc/syscalls.c
  ${asf_dir}/common/services/clock/sam4l/sysclk.c
  ${asf_dir}/common/services/clock/sam4l/dfll.c
  ${asf_dir}/common/services/clock/sam4l/osc.c
  ${asf_dir}/common/services/clock/sam4l/pll.c
  ${asf_dir}/common/utils/interrupt/interrupt_sam_nvic.c
  ${asf_dir}/common/utils/stdio/read.c
  ${asf_dir}/common/utils/stdio/write.c
)


# Specify header files
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${compiler_root}/arm-none-eabi/include
  ${CMSIS_dir}/CMSIS/Core/Include
  ${asf_dir}/common/boards
  ${asf_dir}/common/services/clock
  ${asf_dir}/common/services/clock/sam4l
  ${asf_dir}/common/services/ioport
  ${asf_dir}/common/services/ioport/sam
  ${asf_dir}/common/services/serial
  ${asf_dir}/common/services/serial/sam_uart
  ${asf_dir}/common/utils
  ${asf_dir}/common/utils/interrupt
  ${asf_dir}/sam/utils
  ${asf_dir}/sam/utils/cmsis
  ${asf_dir}/sam/utils/cmsis/sam4l/include
  ${asf_dir}/sam/utils/cmsis/sam4l/source/templates
  ${asf_dir}/sam/utils/cmsis/sam4l/include/component
  ${asf_dir}/sam/utils/cmsis/sam4l/include/instance
  ${asf_dir}/sam/utils/cmsis/sam4l/include/pio
  ${asf_dir}/sam/utils/fpu
  ${asf_dir}/sam/utils/header_files
  ${asf_dir}/sam/utils/preprocessor
  ${asf_dir}/sam/boards
  ${asf_dir}/sam/boards/sam4l8_xplained_pro
  ${asf_dir}/sam/boards/sam4l8_xplained_pro/board_config
  ${asf_dir}/sam/drivers/bpm
  ${asf_dir}/sam/drivers/usart
  ${asf_dir}/sam/drivers/spi
  ${asf_dir}/sam/drivers/flashcalw
  ${asf_dir}/sam/drivers/wdt
  ${asf_dir}/sam/drivers/gpio
)

# Create the executable
add_executable(${PROJECT_NAME}.elf ${SOURCES})

# Linker script
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/sam4lc8c_flash.ld)

# Specify linker flags
set(CMAKE_EXE_LINKER_FLAGS
    "${CMAKE_EXE_LINKER_FLAGS} -T${LINKER_SCRIPT} -Wl,-Map=${PROJECT_NAME}.map"
)

# Specify compiler flags
set(CMAKE_C_FLAGS
    "${CMAKE_C_FLAGS} -march=armv7-m -mthumb -O0 -g"
)

# add definitions
add_compile_definitions(
  __SAM4LC8C__
  BOARD=SAM4L8_XPLAINED_PRO
)

# Post-build commands
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
)

add_custom_target(download
  COMMAND sudo openocd -s ${openocd_root}/scripts -f ${openocd_root}/scripts/board/atmel_sam4l8_xplained_pro.cfg -c init -c halt -c "flash write_image erase ${PROJECT_NAME}.elf" -c reset -c shutdown
)
add_custom_target(connect
  COMMAND sudo openocd -s ${openocd_root}/scripts -f ${openocd_root}/scripts/board/atmel_sam4l8_xplained_pro.cfg 
)
add_custom_target(gdb
  COMMAND arm-none-eabi-gdb ${PROJECT_NAME}.elf
)

# Toolchain definitions
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)

