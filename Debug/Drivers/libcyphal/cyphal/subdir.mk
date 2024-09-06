################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/libcyphal/cyphal/cyphal.cpp \
../Drivers/libcyphal/cyphal/definitions.cpp 

OBJS += \
./Drivers/libcyphal/cyphal/cyphal.o \
./Drivers/libcyphal/cyphal/definitions.o 

CPP_DEPS += \
./Drivers/libcyphal/cyphal/cyphal.d \
./Drivers/libcyphal/cyphal/definitions.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/libcyphal/cyphal/%.o Drivers/libcyphal/cyphal/%.su Drivers/libcyphal/cyphal/%.cyclo: ../Drivers/libcyphal/cyphal/%.cpp Drivers/libcyphal/cyphal/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Drivers/libcyphal/libs -I../Drivers/libvoltbro -I../Drivers/libcyphal -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-libcyphal-2f-cyphal

clean-Drivers-2f-libcyphal-2f-cyphal:
	-$(RM) ./Drivers/libcyphal/cyphal/cyphal.cyclo ./Drivers/libcyphal/cyphal/cyphal.d ./Drivers/libcyphal/cyphal/cyphal.o ./Drivers/libcyphal/cyphal/cyphal.su ./Drivers/libcyphal/cyphal/definitions.cyclo ./Drivers/libcyphal/cyphal/definitions.d ./Drivers/libcyphal/cyphal/definitions.o ./Drivers/libcyphal/cyphal/definitions.su

.PHONY: clean-Drivers-2f-libcyphal-2f-cyphal

