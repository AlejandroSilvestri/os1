################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Thirdparty/g2o/g2o/stuff/os_specific.c 

CPP_SRCS += \
../Thirdparty/g2o/g2o/stuff/property.cpp \
../Thirdparty/g2o/g2o/stuff/string_tools.cpp \
../Thirdparty/g2o/g2o/stuff/timeutil.cpp 

OBJS += \
./Thirdparty/g2o/g2o/stuff/os_specific.o \
./Thirdparty/g2o/g2o/stuff/property.o \
./Thirdparty/g2o/g2o/stuff/string_tools.o \
./Thirdparty/g2o/g2o/stuff/timeutil.o 

C_DEPS += \
./Thirdparty/g2o/g2o/stuff/os_specific.d 

CPP_DEPS += \
./Thirdparty/g2o/g2o/stuff/property.d \
./Thirdparty/g2o/g2o/stuff/string_tools.d \
./Thirdparty/g2o/g2o/stuff/timeutil.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/g2o/g2o/stuff/%.o: ../Thirdparty/g2o/g2o/stuff/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: Cross GCC Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

Thirdparty/g2o/g2o/stuff/%.o: ../Thirdparty/g2o/g2o/stuff/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -DCOMPILEDWITHC11 -D__cplusplus=201103L -DArchivoBowBinario -I"/home/alejandro/Desarrollo eclipse/os1/include" -I/usr/local/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


