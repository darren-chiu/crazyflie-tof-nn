cmd_src/hal/src/sensors.o := arm-none-eabi-gcc -Wp,-MD,src/hal/src/.sensors.o.d    -I/home/darren/Documents/crazyflie-firmware/src/hal/src -Isrc/hal/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/darren/Documents/crazyflie-firmware/vendor/libdw1000/inc   -I/home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/darren/Documents/crazyflie-firmware/src/config   -I/home/darren/Documents/crazyflie-firmware/src/platform/interface   -I/home/darren/Documents/crazyflie-firmware/src/deck/interface   -I/home/darren/Documents/crazyflie-firmware/src/deck/drivers/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/bosch/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/esp32/interface   -I/home/darren/Documents/crazyflie-firmware/src/hal/interface   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/cpx   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/controller   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/estimator   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/kve   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/tdoa   -I/home/darren/Documents/crazyflie-firmware/src/lib/FatFS   -I/home/darren/Documents/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/vl53l1   -I/home/darren/Documents/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/darren/Documents/crazyflie-tof-nn/firmware/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/home/darren/Documents/crazyflie-tof-nn/firmware/src   -I/home/darren/Documents/crazyflie-tof-nn/firmware/src/tof_api -Wno-error   -c -o src/hal/src/sensors.o /home/darren/Documents/crazyflie-firmware/src/hal/src/sensors.c

source_src/hal/src/sensors.o := /home/darren/Documents/crazyflie-firmware/src/hal/src/sensors.c

deps_src/hal/src/sensors.o := \
    $(wildcard include/config/sensors/bmi088/bmp388.h) \
    $(wildcard include/config/sensors/bmi088/spi.h) \
    $(wildcard include/config/sensors/mpu9250/lps25h.h) \
    $(wildcard include/config/sensors/bosch.h) \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/sensors.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdbool.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/imu_types.h \
  /home/darren/Documents/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /home/darren/Documents/crazyflie-firmware/src/platform/interface/platform.h \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/motors.h \
    $(wildcard include/config/motors/esc/protocol/oneshot125.h) \
    $(wildcard include/config/motors/esc/protocol/oneshot42.h) \
    $(wildcard include/config/motors/esc/protocol/dshot.h) \
    $(wildcard include/config/motors/dshot/pwm/150khz.h) \
    $(wildcard include/config/motors/dshot/pwm/300khz.h) \
    $(wildcard include/config/motors/dshot/pwm/600khz.h) \
  /home/darren/Documents/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /home/darren/Documents/crazyflie-firmware/src/config/trace.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/usec_time.h \
  /home/darren/Documents/crazyflie-firmware/src/config/stm32fxxx.h \
  /home/darren/Documents/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
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
  /home/darren/Documents/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/console.h \
  /home/darren/Documents/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdarg.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/sensors_bmi088_bmp388.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/sensors.h \
  /home/darren/Documents/crazyflie-firmware/src/hal/interface/sensors_mpu9250_lps25h.h \

src/hal/src/sensors.o: $(deps_src/hal/src/sensors.o)

$(deps_src/hal/src/sensors.o):
