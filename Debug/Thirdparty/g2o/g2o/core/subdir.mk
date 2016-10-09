################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Thirdparty/g2o/g2o/core/batch_stats.cpp \
../Thirdparty/g2o/g2o/core/cache.cpp \
../Thirdparty/g2o/g2o/core/estimate_propagator.cpp \
../Thirdparty/g2o/g2o/core/factory.cpp \
../Thirdparty/g2o/g2o/core/hyper_dijkstra.cpp \
../Thirdparty/g2o/g2o/core/hyper_graph.cpp \
../Thirdparty/g2o/g2o/core/hyper_graph_action.cpp \
../Thirdparty/g2o/g2o/core/jacobian_workspace.cpp \
../Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.cpp \
../Thirdparty/g2o/g2o/core/matrix_structure.cpp \
../Thirdparty/g2o/g2o/core/optimizable_graph.cpp \
../Thirdparty/g2o/g2o/core/optimization_algorithm.cpp \
../Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.cpp \
../Thirdparty/g2o/g2o/core/optimization_algorithm_factory.cpp \
../Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.cpp \
../Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.cpp \
../Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.cpp \
../Thirdparty/g2o/g2o/core/parameter.cpp \
../Thirdparty/g2o/g2o/core/parameter_container.cpp \
../Thirdparty/g2o/g2o/core/robust_kernel.cpp \
../Thirdparty/g2o/g2o/core/robust_kernel_factory.cpp \
../Thirdparty/g2o/g2o/core/robust_kernel_impl.cpp \
../Thirdparty/g2o/g2o/core/solver.cpp \
../Thirdparty/g2o/g2o/core/sparse_optimizer.cpp 

OBJS += \
./Thirdparty/g2o/g2o/core/batch_stats.o \
./Thirdparty/g2o/g2o/core/cache.o \
./Thirdparty/g2o/g2o/core/estimate_propagator.o \
./Thirdparty/g2o/g2o/core/factory.o \
./Thirdparty/g2o/g2o/core/hyper_dijkstra.o \
./Thirdparty/g2o/g2o/core/hyper_graph.o \
./Thirdparty/g2o/g2o/core/hyper_graph_action.o \
./Thirdparty/g2o/g2o/core/jacobian_workspace.o \
./Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.o \
./Thirdparty/g2o/g2o/core/matrix_structure.o \
./Thirdparty/g2o/g2o/core/optimizable_graph.o \
./Thirdparty/g2o/g2o/core/optimization_algorithm.o \
./Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.o \
./Thirdparty/g2o/g2o/core/optimization_algorithm_factory.o \
./Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.o \
./Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.o \
./Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.o \
./Thirdparty/g2o/g2o/core/parameter.o \
./Thirdparty/g2o/g2o/core/parameter_container.o \
./Thirdparty/g2o/g2o/core/robust_kernel.o \
./Thirdparty/g2o/g2o/core/robust_kernel_factory.o \
./Thirdparty/g2o/g2o/core/robust_kernel_impl.o \
./Thirdparty/g2o/g2o/core/solver.o \
./Thirdparty/g2o/g2o/core/sparse_optimizer.o 

CPP_DEPS += \
./Thirdparty/g2o/g2o/core/batch_stats.d \
./Thirdparty/g2o/g2o/core/cache.d \
./Thirdparty/g2o/g2o/core/estimate_propagator.d \
./Thirdparty/g2o/g2o/core/factory.d \
./Thirdparty/g2o/g2o/core/hyper_dijkstra.d \
./Thirdparty/g2o/g2o/core/hyper_graph.d \
./Thirdparty/g2o/g2o/core/hyper_graph_action.d \
./Thirdparty/g2o/g2o/core/jacobian_workspace.d \
./Thirdparty/g2o/g2o/core/marginal_covariance_cholesky.d \
./Thirdparty/g2o/g2o/core/matrix_structure.d \
./Thirdparty/g2o/g2o/core/optimizable_graph.d \
./Thirdparty/g2o/g2o/core/optimization_algorithm.d \
./Thirdparty/g2o/g2o/core/optimization_algorithm_dogleg.d \
./Thirdparty/g2o/g2o/core/optimization_algorithm_factory.d \
./Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.d \
./Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.d \
./Thirdparty/g2o/g2o/core/optimization_algorithm_with_hessian.d \
./Thirdparty/g2o/g2o/core/parameter.d \
./Thirdparty/g2o/g2o/core/parameter_container.d \
./Thirdparty/g2o/g2o/core/robust_kernel.d \
./Thirdparty/g2o/g2o/core/robust_kernel_factory.d \
./Thirdparty/g2o/g2o/core/robust_kernel_impl.d \
./Thirdparty/g2o/g2o/core/solver.d \
./Thirdparty/g2o/g2o/core/sparse_optimizer.d 


# Each subdirectory must supply rules for building sources it contributes
Thirdparty/g2o/g2o/core/%.o: ../Thirdparty/g2o/g2o/core/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: Cross G++ Compiler'
	g++ -std=c++11 -DCOMPILEDWITHC11 -D__cplusplus=201103L -DArchivoBowBinario -I"/home/alejandro/Desarrollo eclipse/os1/include" -I/usr/local/include/eigen3 -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


