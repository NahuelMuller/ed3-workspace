################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/dspcode/CommonTables/arm_common_tables.c 

OBJS += \
./src/dspcode/CommonTables/arm_common_tables.o 

C_DEPS += \
./src/dspcode/CommonTables/arm_common_tables.d 


# Each subdirectory must supply rules for building sources it contributes
src/dspcode/CommonTables/%.o: ../src/dspcode/CommonTables/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNDEBUG -D__CODE_RED -DARM_MATH_CM4 -D__FPU_PRESENT=1 -I../inc -I../inc_cmsis -O2 -g3 -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fsingle-precision-constant -fno-strict-aliasing -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


