################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../LoRa/Src/lora.c 

OBJS += \
./LoRa/Src/lora.o 

C_DEPS += \
./LoRa/Src/lora.d 


# Each subdirectory must supply rules for building sources it contributes
LoRa/Src/%.o LoRa/Src/%.su LoRa/Src/%.cyclo: ../LoRa/Src/%.c LoRa/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F405xx -c -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -I"C:/Users/Usuario/Downloads/LoraStm32/LoRa/Inc" -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-LoRa-2f-Src

clean-LoRa-2f-Src:
	-$(RM) ./LoRa/Src/lora.cyclo ./LoRa/Src/lora.d ./LoRa/Src/lora.o ./LoRa/Src/lora.su

.PHONY: clean-LoRa-2f-Src

