project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/control/PID.cpp src/tune/Twiddle.cpp src/tune/CarTwiddle.cpp )

include_directories(/usr/local/include libs)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
    add_definitions(-DPLOT_WITH_MATPLOT=1)
    find_package(PythonLibs 2.7)
    include_directories(/usr/local/opt/openssl/include)
    link_directories(/usr/local/opt/openssl/lib)
    link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(pid ${sources} src/main.cpp )
target_link_libraries(pid z ssl uv uWS)

add_executable(tune ${sources} src/tune.cpp )
#if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
    #target_include_directories(tune PRIVATE ${PYTHON_INCLUDE_DIRS})
    #target_include_directories(tune PRIVATE /usr/local/lib/python2.7/site-packages/numpy/core/include)
    #target_link_libraries(tune ${PYTHON_LIBRARIES} )
#endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

