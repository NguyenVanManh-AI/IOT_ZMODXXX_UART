################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ra/fsp/src/r_iic_master/r_iic_master.c 

C_DEPS += \
./ra/fsp/src/r_iic_master/r_iic_master.d 

OBJS += \
./ra/fsp/src/r_iic_master/r_iic_master.o 

SREC += \
zmod4410.srec 

MAP += \
zmod4410.map 


# Each subdirectory must supply rules for building sources it contributes
ra/fsp/src/r_iic_master/%.o: ../ra/fsp/src/r_iic_master/%.c
	$(file > $@.in,-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16 -O2 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-strict-aliasing -Wunused -Wuninitialized -Wall -Wextra -Wmissing-declarations -Wconversion -Wpointer-arith -Wshadow -Wlogical-op -Waggregate-return -Wfloat-equal -g -D_RENESAS_RA_ -D_RA_CORE=CM33 -D_RA_ORDINAL=1 -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/src" -I"." -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/ra/fsp/inc" -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/ra/fsp/inc/api" -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/ra/fsp/inc/instances" -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/ra/arm/CMSIS_6/CMSIS/Core/Include" -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/ra_gen" -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/ra_cfg/fsp_cfg/bsp" -I"C:/Users/ADMIN/e2_studio/workspace/zmod4410/ra_cfg/fsp_cfg" -std=c99 -Wno-stringop-overflow -Wno-format-truncation --param=min-pagesize=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -c -o "$@" -x c "$<")
	@echo Building file: $< && arm-none-eabi-gcc @"$@.in"

