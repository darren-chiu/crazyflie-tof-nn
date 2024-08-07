cmd_src/drivers/src/vl53l0x.o := arm-none-eabi-gcc -Wp,-MD,src/drivers/src/.vl53l0x.o.d    -I/home/darren/Documents/crazyflie-firmware/src/drivers/src -Isrc/drivers/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/home/darren/Documents/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/home/darren/Documents/crazyflie-firmware/vendor/libdw1000/inc   -I/home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include   -I/home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/home/darren/Documents/crazyflie-firmware/src/config   -I/home/darren/Documents/crazyflie-firmware/src/platform/interface   -I/home/darren/Documents/crazyflie-firmware/src/deck/interface   -I/home/darren/Documents/crazyflie-firmware/src/deck/drivers/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/bosch/interface   -I/home/darren/Documents/crazyflie-firmware/src/drivers/esp32/interface   -I/home/darren/Documents/crazyflie-firmware/src/hal/interface   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/kalman_core   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/lighthouse   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/outlierfilter   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/cpx   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/p2pDTR   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/controller   -I/home/darren/Documents/crazyflie-firmware/src/modules/interface/estimator   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/kve   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/lighthouse   -I/home/darren/Documents/crazyflie-firmware/src/utils/interface/tdoa   -I/home/darren/Documents/crazyflie-firmware/src/lib/FatFS   -I/home/darren/Documents/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/home/darren/Documents/crazyflie-firmware/src/lib/vl53l1   -I/home/darren/Documents/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/home/darren/Documents/crazyflie-tof-nn/firmware/build/include/generated -fno-delete-null-pointer-checks -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/home/darren/Documents/crazyflie-tof-nn/firmware/src   -I/home/darren/Documents/crazyflie-tof-nn/firmware/src/tof_api -Wno-error   -c -o src/drivers/src/vl53l0x.o /home/darren/Documents/crazyflie-firmware/src/drivers/src/vl53l0x.c

source_src/drivers/src/vl53l0x.o := /home/darren/Documents/crazyflie-firmware/src/drivers/src/vl53l0x.c

deps_src/drivers/src/vl53l0x.o := \
    $(wildcard include/config/pad/scl/sda//extsup/hv.h) \
    $(wildcard include/config/control.h) \
    $(wildcard include/config/spad/enables/ref/0.h) \
    $(wildcard include/config/ref/en/start/select.h) \
    $(wildcard include/config/gpio.h) \
    $(wildcard include/config/min/count/rate/rtn/limit.h) \
    $(wildcard include/config/timeout/macrop/hi.h) \
    $(wildcard include/config/valid/phase/high.h) \
    $(wildcard include/config/valid/phase/low.h) \
    $(wildcard include/config/vcsel/period.h) \
    $(wildcard include/config/timeout/macrop.h) \
    $(wildcard include/config/vcsel/width.h) \
    $(wildcard include/config/timeout.h) \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stddef.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdint.h \
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
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /home/darren/Documents/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /home/darren/Documents/crazyflie-firmware/src/config/config.h \
  /home/darren/Documents/crazyflie-firmware/src/modules/interface/console.h \
  /home/darren/Documents/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/lib/gcc/arm-none-eabi/10.3.1/include/stdarg.h \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /home/darren/Documents/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
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
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/vl53l0x.h \
    $(wildcard include/config/min/snr.h) \
    $(wildcard include/config/sigma/thresh/hi.h) \
    $(wildcard include/config/sigma/thresh/lo.h) \
    $(wildcard include/config/timeout/macrop/lo.h) \
    $(wildcard include/config/initial/phase/select.h) \
    $(wildcard include/config/readout/ctrl.h) \
    $(wildcard include/config/spad/enables/ref/1.h) \
    $(wildcard include/config/spad/enables/ref/2.h) \
    $(wildcard include/config/spad/enables/ref/3.h) \
    $(wildcard include/config/spad/enables/ref/4.h) \
    $(wildcard include/config/spad/enables/ref/5.h) \
  /home/darren/Documents/crazyflie-firmware/src/drivers/interface/i2cdev.h \

src/drivers/src/vl53l0x.o: $(deps_src/drivers/src/vl53l0x.o)

$(deps_src/drivers/src/vl53l0x.o):
