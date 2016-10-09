################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Thirdparty/g2o/g2o/types/types_sba.cpp \
../Thirdparty/g2o/g2o/types/types_seven_dof_expmap.cpp \
../Thirdparty/g2o/g2o/types/types_six_dof_expmap.cpp 

OBJS += \
./Thirdparty/g2o/g2o/types/types_sba.o \
./Thirdparty/g2o/g2o/types/types_seven_dof_expmap.o \
./Thirdparty/g2o/g2o/types/types_six_dof_expmap.o 

CPP_DEPS += \
./Thirdparty/g2o/g2o/types/types_sba.d \
./Thirdparty/g2o/g2o/types/types_seven_dof_expmap.d \
./Thirdparty/g2o/g2o/types/types_six_dof_expmap.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/g2o/g2o/types/%.o: ../Thirdparty/g2o/g2o/types/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -DCOMPILEDWITHC11 -D__cplusplus=201103L -DArchivoBowBinario -I"/home/alejandro/Desarrollo eclipse/os1/include" -I/usr/local/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


