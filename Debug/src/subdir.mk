################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../src/Converter.cc \
../src/Frame.cc \
../src/FrameDrawer.cc \
../src/Initializer.cc \
../src/KeyFrame.cc \
../src/KeyFrameDatabase.cc \
../src/LocalMapping.cc \
../src/LoopClosing.cc \
../src/Map.cc \
../src/MapDrawer.cc \
../src/MapPoint.cc \
../src/ORBextractor.cc \
../src/ORBmatcher.cc \
../src/Optimizer.cc \
../src/PnPsolver.cc \
../src/Sim3Solver.cc \
../src/System.cc \
../src/Tracking.cc \
../src/Viewer.cc \
../src/main.cc 

CC_DEPS += \
./src/Converter.d \
./src/Frame.d \
./src/FrameDrawer.d \
./src/Initializer.d \
./src/KeyFrame.d \
./src/KeyFrameDatabase.d \
./src/LocalMapping.d \
./src/LoopClosing.d \
./src/Map.d \
./src/MapDrawer.d \
./src/MapPoint.d \
./src/ORBextractor.d \
./src/ORBmatcher.d \
./src/Optimizer.d \
./src/PnPsolver.d \
./src/Sim3Solver.d \
./src/System.d \
./src/Tracking.d \
./src/Viewer.d \
./src/main.d 

OBJS += \
./src/Converter.o \
./src/Frame.o \
./src/FrameDrawer.o \
./src/Initializer.o \
./src/KeyFrame.o \
./src/KeyFrameDatabase.o \
./src/LocalMapping.o \
./src/LoopClosing.o \
./src/Map.o \
./src/MapDrawer.o \
./src/MapPoint.o \
./src/ORBextractor.o \
./src/ORBmatcher.o \
./src/Optimizer.o \
./src/PnPsolver.o \
./src/Sim3Solver.o \
./src/System.o \
./src/Tracking.o \
./src/Viewer.o \
./src/main.o 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -DCOMPILEDWITHC11 -D__cplusplus=201103L -DArchivoBowBinario -I"/home/alejandro/Desarrollo eclipse/os1/include" -I/usr/local/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


