project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/control/PID.cpp src/tune/Twiddle.cpp src/tune/CarTwiddle.cpp )

include_directories(/usr/local/include libs)
link_directories(/usr/local/lib)

if (PLOT_WITH_MATPLOT)
    add_definitions(-DPLOT_WITH_MATPLOT=1)
    find_package(PythonLibs 2.7)
endif(PLOT_WITH_MATPLOT)

if (VERBOSE_OUT)
    add_definitions(-DVERBOSE_OUT=1)
endif(VERBOSE_OUT)

if (STABILIZE_MOTION)
    add_definitions(-DSTABILIZE_MOTION=1)
    add_definitions(-DUSE_MOVING_AVERAGE=1)
endif(STABILIZE_MOTION)

if (USE_MOVING_AVERAGE)
    add_definitions(-DUSE_MOVING_AVERAGE=1)
endif(USE_MOVING_AVERAGE)

if (CLAMP_STEERING_DELTA)
    add_definitions(-DCLAMP_STEERING_DELTA=1)
endif(CLAMP_STEERING_DELTA)

if (USE_MEAN_TURN)
    add_definitions(-DUSE_MEAN_TURN=1)
endif(USE_MEAN_TURN)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
    find_path(LIBUV_INCLUDE_DIR uv.h PATH_SUFFIXES libuv )
    find_library(LIBUV NAMES uv libuv)
    include_directories(${Libuv_INCLUDE_DIRS})
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(${LIBUV_LIBRARIES})
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(pid ${sources} src/pid_main.cpp )
target_link_libraries(pid z ssl uv uWS)

add_executable(twiddle ${sources} src/twiddle_main.cpp )
if (PLOT_WITH_MATPLOT)
    target_include_directories(twiddle PRIVATE ${PYTHON_INCLUDE_DIRS})
    target_include_directories(twiddle PRIVATE /usr/local/lib/python2.7/site-packages/numpy/core/include)
    target_link_libraries(twiddle ${PYTHON_LIBRARIES} )
endif(PLOT_WITH_MATPLOT)
