cmd_src/modules/src/estimator/estimator_complementary.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/estimator/.estimator_complementary.o.d    -I/home/darren/Documents/crazyflie-firmware/src/modules/src/estimator -Isrc/modules/src/estimator -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/darren/Documents/crazyflie-firmware/vendor/libdw1000/inc   -I/home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/darren/Documents/crazyflie-firmware/src/config   -I/home/darren/Documents/crazyflie-firmware/src/platform/interface   -I/home/darren/Documents/crazyflie-firmware/src/deck/interface   -I/home/darren/Documents/crazyflie-firmware/src/deck/drivers/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/bosch/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/esp32/interface   -I/home/darren/Documents/crazyflie-firmware/src/hal/interface   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/cpx   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/controller   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/estimator   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/kve   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/tdoa   -I/home/darren/Documents/crazyflie-firmware/src/lib/FatFS   -I/home/darren/Documents/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/vl53l1   -I/home/darren/Documents/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/darren/Documents/crazyflie-tof-nn/firmware/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/home/darren/Documents/crazyflie-tof-nn/firmware/src   -I/home/darren/Documents/crazyflie-tof-nn/firmware/src/tof_api -Wno-error   -c -o src/modules/src/estimator/estimator_complementary.o /home/darren/Documents/crazyflie-firmware/src/modules/src/estimator/estimator_complementary.c

source_src/modules/src/estimator/estimator_complementary.o := /home/darren/Documents/crazyflie-firmware/src/modules/src/estimator/estimator_complementary.c

deps_src/modules/src/estimator/estimator_complementary.o := \
  /home/darren/Documents/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /home/darren/Documents/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /home/darren/Documents/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /home/darren/Documents/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/darren/Documents/crazyflie-firmware/src/config/trace.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/darren/Documents/crazyflie-firmware/src/utils/interface/cfassert.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/stabilizer.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/darren/Documents/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/estimator/estimator_complementary.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/sensfusion6.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/estimator/position_estimator.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/sensors.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/static_mem.h \

src/modules/src/estimator/estimator_complementary.o: $(deps_src/modules/src/estimator/estimator_complementary.o)

$(deps_src/modules/src/estimator/estimator_complementary.o):
