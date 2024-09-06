################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/libcyphal/cyphal/allocators/sys/sys_allocator.cpp 

OBJS += \
./Drivers/libcyphal/cyphal/allocators/sys/sys_allocator.o 

CPP_DEPS += \
./Drivers/libcyphal/cyphal/allocators/sys/sys_allocator.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/libcyphal/cyphal/allocators/sys/%.o Drivers/libcyphal/cyphal/allocators/sys/%.su Drivers/libcyphal/cyphal/allocators/sys/%.cyclo: ../Drivers/libcyphal/cyphal/allocators/sys/%.cpp Drivers/libcyphal/cyphal/allocators/sys/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Drivers/libcyphal/libs -I../Drivers/libvoltbro -I../Drivers/libcyphal -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-libcyphal-2f-cyphal-2f-allocators-2f-sys

clean-Drivers-2f-libcyphal-2f-cyphal-2f-allocators-2f-sys:
	-$(RM) ./Drivers/libcyphal/cyphal/allocators/sys/sys_allocator.cyclo ./Drivers/libcyphal/cyphal/allocators/sys/sys_allocator.d ./Drivers/libcyphal/cyphal/allocators/sys/sys_allocator.o ./Drivers/libcyphal/cyphal/allocators/sys/sys_allocator.su

.PHONY: clean-Drivers-2f-libcyphal-2f-cyphal-2f-allocators-2f-sys

