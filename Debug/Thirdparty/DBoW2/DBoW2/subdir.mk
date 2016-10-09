################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Thirdparty/DBoW2/DBoW2/BowVector.cpp \
../Thirdparty/DBoW2/DBoW2/FORB.cpp \
../Thirdparty/DBoW2/DBoW2/FeatureVector.cpp \
../Thirdparty/DBoW2/DBoW2/ScoringObject.cpp 

OBJS += \
./Thirdparty/DBoW2/DBoW2/BowVector.o \
./Thirdparty/DBoW2/DBoW2/FORB.o \
./Thirdparty/DBoW2/DBoW2/FeatureVector.o \
./Thirdparty/DBoW2/DBoW2/ScoringObject.o 

CPP_DEPS += \
./Thirdparty/DBoW2/DBoW2/BowVector.d \
./Thirdparty/DBoW2/DBoW2/FORB.d \
./Thirdparty/DBoW2/DBoW2/FeatureVector.d \
./Thirdparty/DBoW2/DBoW2/ScoringObject.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/DBoW2/DBoW2/%.o: ../Thirdparty/DBoW2/DBoW2/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -DCOMPILEDWITHC11 -D__cplusplus=201103L -DArchivoBowBinario -I"/home/alejandro/Desarrollo eclipse/os1/include" -I/usr/local/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


