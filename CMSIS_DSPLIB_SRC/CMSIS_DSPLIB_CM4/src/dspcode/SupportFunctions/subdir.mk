################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/dspcode/SupportFunctions/arm_copy_f32.c \
../src/dspcode/SupportFunctions/arm_copy_q15.c \
../src/dspcode/SupportFunctions/arm_copy_q31.c \
../src/dspcode/SupportFunctions/arm_copy_q7.c \
../src/dspcode/SupportFunctions/arm_fill_f32.c \
../src/dspcode/SupportFunctions/arm_fill_q15.c \
../src/dspcode/SupportFunctions/arm_fill_q31.c \
../src/dspcode/SupportFunctions/arm_fill_q7.c \
../src/dspcode/SupportFunctions/arm_float_to_q15.c \
../src/dspcode/SupportFunctions/arm_float_to_q31.c \
../src/dspcode/SupportFunctions/arm_float_to_q7.c \
../src/dspcode/SupportFunctions/arm_q15_to_float.c \
../src/dspcode/SupportFunctions/arm_q15_to_q31.c \
../src/dspcode/SupportFunctions/arm_q15_to_q7.c \
../src/dspcode/SupportFunctions/arm_q31_to_float.c \
../src/dspcode/SupportFunctions/arm_q31_to_q15.c \
../src/dspcode/SupportFunctions/arm_q31_to_q7.c \
../src/dspcode/SupportFunctions/arm_q7_to_float.c \
../src/dspcode/SupportFunctions/arm_q7_to_q15.c \
../src/dspcode/SupportFunctions/arm_q7_to_q31.c 

OBJS += \
./src/dspcode/SupportFunctions/arm_copy_f32.o \
./src/dspcode/SupportFunctions/arm_copy_q15.o \
./src/dspcode/SupportFunctions/arm_copy_q31.o \
./src/dspcode/SupportFunctions/arm_copy_q7.o \
./src/dspcode/SupportFunctions/arm_fill_f32.o \
./src/dspcode/SupportFunctions/arm_fill_q15.o \
./src/dspcode/SupportFunctions/arm_fill_q31.o \
./src/dspcode/SupportFunctions/arm_fill_q7.o \
./src/dspcode/SupportFunctions/arm_float_to_q15.o \
./src/dspcode/SupportFunctions/arm_float_to_q31.o \
./src/dspcode/SupportFunctions/arm_float_to_q7.o \
./src/dspcode/SupportFunctions/arm_q15_to_float.o \
./src/dspcode/SupportFunctions/arm_q15_to_q31.o \
./src/dspcode/SupportFunctions/arm_q15_to_q7.o \
./src/dspcode/SupportFunctions/arm_q31_to_float.o \
./src/dspcode/SupportFunctions/arm_q31_to_q15.o \
./src/dspcode/SupportFunctions/arm_q31_to_q7.o \
./src/dspcode/SupportFunctions/arm_q7_to_float.o \
./src/dspcode/SupportFunctions/arm_q7_to_q15.o \
./src/dspcode/SupportFunctions/arm_q7_to_q31.o 

C_DEPS += \
./src/dspcode/SupportFunctions/arm_copy_f32.d \
./src/dspcode/SupportFunctions/arm_copy_q15.d \
./src/dspcode/SupportFunctions/arm_copy_q31.d \
./src/dspcode/SupportFunctions/arm_copy_q7.d \
./src/dspcode/SupportFunctions/arm_fill_f32.d \
./src/dspcode/SupportFunctions/arm_fill_q15.d \
./src/dspcode/SupportFunctions/arm_fill_q31.d \
./src/dspcode/SupportFunctions/arm_fill_q7.d \
./src/dspcode/SupportFunctions/arm_float_to_q15.d \
./src/dspcode/SupportFunctions/arm_float_to_q31.d \
./src/dspcode/SupportFunctions/arm_float_to_q7.d \
./src/dspcode/SupportFunctions/arm_q15_to_float.d \
./src/dspcode/SupportFunctions/arm_q15_to_q31.d \
./src/dspcode/SupportFunctions/arm_q15_to_q7.d \
./src/dspcode/SupportFunctions/arm_q31_to_float.d \
./src/dspcode/SupportFunctions/arm_q31_to_q15.d \
./src/dspcode/SupportFunctions/arm_q31_to_q7.d \
./src/dspcode/SupportFunctions/arm_q7_to_float.d \
./src/dspcode/SupportFunctions/arm_q7_to_q15.d \
./src/dspcode/SupportFunctions/arm_q7_to_q31.d 


# Each subdirectory must supply rules for building sources it contributes
src/dspcode/SupportFunctions/%.o: ../src/dspcode/SupportFunctions/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNDEBUG -D__CODE_RED -DARM_MATH_CM4 -D__FPU_PRESENT=1 -I../inc -I../inc_cmsis -O2 -g3 -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fsingle-precision-constant -fno-strict-aliasing -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


