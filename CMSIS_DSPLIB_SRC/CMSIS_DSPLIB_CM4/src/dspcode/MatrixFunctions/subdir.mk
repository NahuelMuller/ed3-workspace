################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/dspcode/MatrixFunctions/arm_mat_add_f32.c \
../src/dspcode/MatrixFunctions/arm_mat_add_q15.c \
../src/dspcode/MatrixFunctions/arm_mat_add_q31.c \
../src/dspcode/MatrixFunctions/arm_mat_init_f32.c \
../src/dspcode/MatrixFunctions/arm_mat_init_q15.c \
../src/dspcode/MatrixFunctions/arm_mat_init_q31.c \
../src/dspcode/MatrixFunctions/arm_mat_inverse_f32.c \
../src/dspcode/MatrixFunctions/arm_mat_mult_f32.c \
../src/dspcode/MatrixFunctions/arm_mat_mult_fast_q15.c \
../src/dspcode/MatrixFunctions/arm_mat_mult_fast_q31.c \
../src/dspcode/MatrixFunctions/arm_mat_mult_q15.c \
../src/dspcode/MatrixFunctions/arm_mat_mult_q31.c \
../src/dspcode/MatrixFunctions/arm_mat_scale_f32.c \
../src/dspcode/MatrixFunctions/arm_mat_scale_q15.c \
../src/dspcode/MatrixFunctions/arm_mat_scale_q31.c \
../src/dspcode/MatrixFunctions/arm_mat_sub_f32.c \
../src/dspcode/MatrixFunctions/arm_mat_sub_q15.c \
../src/dspcode/MatrixFunctions/arm_mat_sub_q31.c \
../src/dspcode/MatrixFunctions/arm_mat_trans_f32.c \
../src/dspcode/MatrixFunctions/arm_mat_trans_q15.c \
../src/dspcode/MatrixFunctions/arm_mat_trans_q31.c 

OBJS += \
./src/dspcode/MatrixFunctions/arm_mat_add_f32.o \
./src/dspcode/MatrixFunctions/arm_mat_add_q15.o \
./src/dspcode/MatrixFunctions/arm_mat_add_q31.o \
./src/dspcode/MatrixFunctions/arm_mat_init_f32.o \
./src/dspcode/MatrixFunctions/arm_mat_init_q15.o \
./src/dspcode/MatrixFunctions/arm_mat_init_q31.o \
./src/dspcode/MatrixFunctions/arm_mat_inverse_f32.o \
./src/dspcode/MatrixFunctions/arm_mat_mult_f32.o \
./src/dspcode/MatrixFunctions/arm_mat_mult_fast_q15.o \
./src/dspcode/MatrixFunctions/arm_mat_mult_fast_q31.o \
./src/dspcode/MatrixFunctions/arm_mat_mult_q15.o \
./src/dspcode/MatrixFunctions/arm_mat_mult_q31.o \
./src/dspcode/MatrixFunctions/arm_mat_scale_f32.o \
./src/dspcode/MatrixFunctions/arm_mat_scale_q15.o \
./src/dspcode/MatrixFunctions/arm_mat_scale_q31.o \
./src/dspcode/MatrixFunctions/arm_mat_sub_f32.o \
./src/dspcode/MatrixFunctions/arm_mat_sub_q15.o \
./src/dspcode/MatrixFunctions/arm_mat_sub_q31.o \
./src/dspcode/MatrixFunctions/arm_mat_trans_f32.o \
./src/dspcode/MatrixFunctions/arm_mat_trans_q15.o \
./src/dspcode/MatrixFunctions/arm_mat_trans_q31.o 

C_DEPS += \
./src/dspcode/MatrixFunctions/arm_mat_add_f32.d \
./src/dspcode/MatrixFunctions/arm_mat_add_q15.d \
./src/dspcode/MatrixFunctions/arm_mat_add_q31.d \
./src/dspcode/MatrixFunctions/arm_mat_init_f32.d \
./src/dspcode/MatrixFunctions/arm_mat_init_q15.d \
./src/dspcode/MatrixFunctions/arm_mat_init_q31.d \
./src/dspcode/MatrixFunctions/arm_mat_inverse_f32.d \
./src/dspcode/MatrixFunctions/arm_mat_mult_f32.d \
./src/dspcode/MatrixFunctions/arm_mat_mult_fast_q15.d \
./src/dspcode/MatrixFunctions/arm_mat_mult_fast_q31.d \
./src/dspcode/MatrixFunctions/arm_mat_mult_q15.d \
./src/dspcode/MatrixFunctions/arm_mat_mult_q31.d \
./src/dspcode/MatrixFunctions/arm_mat_scale_f32.d \
./src/dspcode/MatrixFunctions/arm_mat_scale_q15.d \
./src/dspcode/MatrixFunctions/arm_mat_scale_q31.d \
./src/dspcode/MatrixFunctions/arm_mat_sub_f32.d \
./src/dspcode/MatrixFunctions/arm_mat_sub_q15.d \
./src/dspcode/MatrixFunctions/arm_mat_sub_q31.d \
./src/dspcode/MatrixFunctions/arm_mat_trans_f32.d \
./src/dspcode/MatrixFunctions/arm_mat_trans_q15.d \
./src/dspcode/MatrixFunctions/arm_mat_trans_q31.d 


# Each subdirectory must supply rules for building sources it contributes
src/dspcode/MatrixFunctions/%.o: ../src/dspcode/MatrixFunctions/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU C Compiler'
	arm-none-eabi-gcc -D__REDLIB__ -DNDEBUG -D__CODE_RED -DARM_MATH_CM4 -D__FPU_PRESENT=1 -I../inc -I../inc_cmsis -O2 -g3 -c -fmessage-length=0 -fno-builtin -ffunction-sections -fdata-sections -fsingle-precision-constant -fno-strict-aliasing -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -mthumb -D__REDLIB__ -specs=redlib.specs -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


