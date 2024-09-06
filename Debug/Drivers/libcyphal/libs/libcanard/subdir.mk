################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/libcyphal/libs/libcanard/canard.c 

C_DEPS += \
./Drivers/libcyphal/libs/libcanard/canard.d 

OBJS += \
./Drivers/libcyphal/libs/libcanard/canard.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/libcyphal/libs/libcanard/%.o Drivers/libcyphal/libs/libcanard/%.su Drivers/libcyphal/libs/libcanard/%.cyclo: ../Drivers/libcyphal/libs/libcanard/%.c Drivers/libcyphal/libs/libcanard/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Drivers/libcyphal -I../Drivers/libvoltbro -I../Drivers/libcyphal/libs -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-libcyphal-2f-libs-2f-libcanard

clean-Drivers-2f-libcyphal-2f-libs-2f-libcanard:
	-$(RM) ./Drivers/libcyphal/libs/libcanard/canard.cyclo ./Drivers/libcyphal/libs/libcanard/canard.d ./Drivers/libcyphal/libs/libcanard/canard.o ./Drivers/libcyphal/libs/libcanard/canard.su

.PHONY: clean-Drivers-2f-libcyphal-2f-libs-2f-libcanard

