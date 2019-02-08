################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/dspcode/StatisticsFunctions/arm_max_f32.c \
../src/dspcode/StatisticsFunctions/arm_max_q15.c \
../src/dspcode/StatisticsFunctions/arm_max_q31.c \
../src/dspcode/StatisticsFunctions/arm_max_q7.c \
../src/dspcode/StatisticsFunctions/arm_mean_f32.c \
../src/dspcode/StatisticsFunctions/arm_mean_q15.c \
../src/dspcode/StatisticsFunctions/arm_mean_q31.c \
../src/dspcode/StatisticsFunctions/arm_mean_q7.c \
../src/dspcode/StatisticsFunctions/arm_min_f32.c \
../src/dspcode/StatisticsFunctions/arm_min_q15.c \
../src/dspcode/StatisticsFunctions/arm_min_q31.c \
../src/dspcode/StatisticsFunctions/arm_min_q7.c \
../src/dspcode/StatisticsFunctions/arm_power_f32.c \
../src/dspcode/StatisticsFunctions/arm_power_q15.c \
../src/dspcode/StatisticsFunctions/arm_power_q31.c \
../src/dspcode/StatisticsFunctions/arm_power_q7.c \
../src/dspcode/StatisticsFunctions/arm_rms_f32.c \
../src/dspcode/StatisticsFunctions/arm_rms_q15.c \
../src/dspcode/StatisticsFunctions/arm_rms_q31.c \
../src/dspcode/StatisticsFunctions/arm_std_f32.c \
../src/dspcode/StatisticsFunctions/arm_std_q15.c \
../src/dspcode/StatisticsFunctions/arm_std_q31.c \
../src/dspcode/StatisticsFunctions/arm_var_f32.c \
../src/dspcode/StatisticsFunctions/arm_var_q15.c \
../src/dspcode/StatisticsFunctions/arm_var_q31.c 

OBJS += \
./src/dspcode/StatisticsFunctions/arm_max_f32.o \
./src/dspcode/StatisticsFunctions/arm_max_q15.o \
./src/dspcode/StatisticsFunctions/arm_max_q31.o \
./src/dspcode/StatisticsFunctions/arm_max_q7.o \
./src/dspcode/StatisticsFunctions/arm_mean_f32.o \
./src/dspcode/StatisticsFunctions/arm_mean_q15.o \
./src/dspcode/StatisticsFunctions/arm_mean_q31.o \
./src/dspcode/StatisticsFunctions/arm_mean_q7.o \
./src/dspcode/StatisticsFunctions/arm_min_f32.o \
./src/dspcode/StatisticsFunctions/arm_min_q15.o \
./src/dspcode/StatisticsFunctions/arm_min_q31.o \
./src/dspcode/StatisticsFunctions/arm_min_q7.o \
./src/dspcode/StatisticsFunctions/arm_power_f32.o \
./src/dspcode/StatisticsFunctions/arm_power_q15.o \
./src/dspcode/StatisticsFunctions/arm_power_q31.o \
./src/dspcode/StatisticsFunctions/arm_power_q7.o \
./src/dspcode/StatisticsFunctions/arm_rms_f32.o \
./src/dspcode/StatisticsFunctions/arm_rms_q15.o \
./src/dspcode/StatisticsFunctions/arm_rms_q31.o \
./src/dspcode/StatisticsFunctions/arm_std_f32.o \
./src/dspcode/StatisticsFunctions/arm_std_q15.o \
./src/dspcode/StatisticsFunctions/arm_std_q31.o \
./src/dspcode/StatisticsFunctions/arm_var_f32.o \
./src/dspcode/StatisticsFunctions/arm_var_q15.o \
./src/dspcode/StatisticsFunctions/arm_var_q31.o 

C_DEPS += \
./src/dspcode/StatisticsFunctions/arm_max_f32.d \
./src/dspcode/StatisticsFunctions/arm_max_q15.d \
./src/dspcode/StatisticsFunctions/arm_max_q31.d \
./src/dspcode/StatisticsFunctions/arm_max_q7.d \
./src/dspcode/StatisticsFunctions/arm_mean_f32.d \
./src/dspcode/StatisticsFunctions/arm_mean_q15.d \
./src/dspcode/StatisticsFunctions/arm_mean_q31.d \
./src/dspcode/StatisticsFunctions/arm_mean_q7.d \
./src/dspcode/StatisticsFunctions/arm_min_f32.d \
./src/dspcode/StatisticsFunctions/arm_min_q15.d \
./src/dspcode/StatisticsFunctions/arm_min_q31.d \
./src/dspcode/StatisticsFunctions/arm_min_q7.d \
./src/dspcode/StatisticsFunctions/arm_power_f32.d \
./src/dspcode/StatisticsFunctions/arm_power_q15.d \
./src/dspcode/StatisticsFunctions/arm_power_q31.d \
./src/dspcode/StatisticsFunctions/arm_power_q7.d \
./src/dspcode/StatisticsFunctions/arm_rms_f32.d \
./src/dspcode/StatisticsFunctions/arm_rms_q15.d \
./src/dspcode/StatisticsFunctions/arm_rms_q31.d \
./src/dspcode/StatisticsFunctions/arm_std_f32.d \
./src/dspcode/StatisticsFunctions/arm_std_q15.d \
./src/dspcode/StatisticsFunctions/arm_std_q31.d \
./src/dspcode/StatisticsFunctions/arm_var_f32.d \
./src/dspcode/StatisticsFunctions/arm_var_q15.d \
./src/dspcode/StatisticsFunctions/arm_var_q31.d 


# Each subdirectory must supply rules for building sources it contributes
src/dspcode/StatisticsFunctions/%.o: ../src/dspcode/StatisticsFunctions/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNDEBUG -D__CODE_RED -DARM_MATH_CM4 -D__FPU_PRESENT=1 -I../inc -I../inc_cmsis -O2 -g3 -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fsingle-precision-constant -fno-strict-aliasing -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


