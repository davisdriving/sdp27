################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (14.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../STM32CubeIDE/Application/User/Core/syscalls.c \
../STM32CubeIDE/Application/User/Core/sysmem.c 

OBJS += \
./STM32CubeIDE/Application/User/Core/syscalls.o \
./STM32CubeIDE/Application/User/Core/sysmem.o 

C_DEPS += \
./STM32CubeIDE/Application/User/Core/syscalls.d \
./STM32CubeIDE/Application/User/Core/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
STM32CubeIDE/Application/User/Core/%.o STM32CubeIDE/Application/User/Core/%.su STM32CubeIDE/Application/User/Core/%.cyclo: ../STM32CubeIDE/Application/User/Core/%.c STM32CubeIDE/Application/User/Core/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../USB_Device/App -I../USB_Device/Target -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -ftls-model=local-exec -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-STM32CubeIDE-2f-Application-2f-User-2f-Core

clean-STM32CubeIDE-2f-Application-2f-User-2f-Core:
	-$(RM) ./STM32CubeIDE/Application/User/Core/syscalls.cyclo ./STM32CubeIDE/Application/User/Core/syscalls.d ./STM32CubeIDE/Application/User/Core/syscalls.o ./STM32CubeIDE/Application/User/Core/syscalls.su ./STM32CubeIDE/Application/User/Core/sysmem.cyclo ./STM32CubeIDE/Application/User/Core/sysmem.d ./STM32CubeIDE/Application/User/Core/sysmem.o ./STM32CubeIDE/Application/User/Core/sysmem.su

.PHONY: clean-STM32CubeIDE-2f-Application-2f-User-2f-Core

