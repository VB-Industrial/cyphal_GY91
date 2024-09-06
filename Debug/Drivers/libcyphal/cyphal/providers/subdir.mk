################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Drivers/libcyphal/cyphal/providers/FDCAN_generic.cpp \
../Drivers/libcyphal/cyphal/providers/G4CAN.cpp \
../Drivers/libcyphal/cyphal/providers/LinuxCAN.cpp \
../Drivers/libcyphal/cyphal/providers/provider.cpp 

OBJS += \
./Drivers/libcyphal/cyphal/providers/FDCAN_generic.o \
./Drivers/libcyphal/cyphal/providers/G4CAN.o \
./Drivers/libcyphal/cyphal/providers/LinuxCAN.o \
./Drivers/libcyphal/cyphal/providers/provider.o 

CPP_DEPS += \
./Drivers/libcyphal/cyphal/providers/FDCAN_generic.d \
./Drivers/libcyphal/cyphal/providers/G4CAN.d \
./Drivers/libcyphal/cyphal/providers/LinuxCAN.d \
./Drivers/libcyphal/cyphal/providers/provider.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/libcyphal/cyphal/providers/%.o Drivers/libcyphal/cyphal/providers/%.su Drivers/libcyphal/cyphal/providers/%.cyclo: ../Drivers/libcyphal/cyphal/providers/%.cpp Drivers/libcyphal/cyphal/providers/subdir.mk
	arm-none-eabi-g++ "$<" -mcpu=cortex-m4 -std=gnu++17 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G474xx -c -I../Drivers/libcyphal/libs -I../Drivers/libvoltbro -I../Drivers/libcyphal -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-use-cxa-atexit -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-libcyphal-2f-cyphal-2f-providers

clean-Drivers-2f-libcyphal-2f-cyphal-2f-providers:
	-$(RM) ./Drivers/libcyphal/cyphal/providers/FDCAN_generic.cyclo ./Drivers/libcyphal/cyphal/providers/FDCAN_generic.d ./Drivers/libcyphal/cyphal/providers/FDCAN_generic.o ./Drivers/libcyphal/cyphal/providers/FDCAN_generic.su ./Drivers/libcyphal/cyphal/providers/G4CAN.cyclo ./Drivers/libcyphal/cyphal/providers/G4CAN.d ./Drivers/libcyphal/cyphal/providers/G4CAN.o ./Drivers/libcyphal/cyphal/providers/G4CAN.su ./Drivers/libcyphal/cyphal/providers/LinuxCAN.cyclo ./Drivers/libcyphal/cyphal/providers/LinuxCAN.d ./Drivers/libcyphal/cyphal/providers/LinuxCAN.o ./Drivers/libcyphal/cyphal/providers/LinuxCAN.su ./Drivers/libcyphal/cyphal/providers/provider.cyclo ./Drivers/libcyphal/cyphal/providers/provider.d ./Drivers/libcyphal/cyphal/providers/provider.o ./Drivers/libcyphal/cyphal/providers/provider.su

.PHONY: clean-Drivers-2f-libcyphal-2f-cyphal-2f-providers

