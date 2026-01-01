################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../tiny-sha256/sha256.c 

OBJS += \
./tiny-sha256/sha256.o 

C_DEPS += \
./tiny-sha256/sha256.d 


# Each subdirectory must supply rules for building sources it contributes
tiny-sha256/%.o tiny-sha256/%.su tiny-sha256/%.cyclo: ../tiny-sha256/%.c tiny-sha256/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DSTM32G030xx -DUSE_FULL_LL_DRIVER -DHSE_VALUE=8000000 -DHSE_STARTUP_TIMEOUT=100 -DLSE_STARTUP_TIMEOUT=5000 -DLSE_VALUE=32768 -DEXTERNAL_CLOCK_VALUE=12288000 -DHSI_VALUE=16000000 -DLSI_VALUE=32000 -DVDD_VALUE=3300 -DPREFETCH_ENABLE=1 -DINSTRUCTION_CACHE_ENABLE=1 -DDATA_CACHE_ENABLE=1 -c -I../Core/Inc -I"C:/ST/STProjects/DEMSAY/DEM_BOOTLOADER_STM/tiny-aes" -I"C:/ST/STProjects/DEMSAY/DEM_BOOTLOADER_STM/ssd1306_LL" -I../Drivers/STM32G0xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32G0xx/Include -I../Drivers/CMSIS/Include -I"C:/ST/STProjects/DEMSAY/DEM_BOOTLOADER_STM/tiny-sha256" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-tiny-2d-sha256

clean-tiny-2d-sha256:
	-$(RM) ./tiny-sha256/sha256.cyclo ./tiny-sha256/sha256.d ./tiny-sha256/sha256.o ./tiny-sha256/sha256.su

.PHONY: clean-tiny-2d-sha256

