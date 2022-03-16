project(mpc_car_practice)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++17 -O3)
add_definitions(-w)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/mpc.cpp src/main.cpp)
#for cppad examples
set(cppad_exp_1 examples/cppad_example_1.cpp)
set(cppad_exp_2 examples/cppad_ipopt_example_2.cpp)
#for mpc
include_directories(include)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
#third party
include_directories(third_party/gnuplot-iostream)
include_directories(third_party/Eigen-3.3)

add_subdirectory(third_party/gnuplot-iostream)

# Determine if it is a macos
if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/opt/openssl/lib)
#    link_directories(/usr/local/Cellar/libuv/1*/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc_practice ${sources})
target_link_libraries(mpc_practice ipopt z ssl uv uWS gnuplot_iostream util)

#add_executable(cppad_exp_1 ${cppad_exp_1})
#target_link_libraries(cppad_exp_1)
#
#add_executable(cppad_exp_2 ${cppad_exp_2})
#target_link_libraries(cppad_exp_2 ipopt)

