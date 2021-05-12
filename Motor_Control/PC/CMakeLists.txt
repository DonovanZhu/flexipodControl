cmake_minimum_required(VERSION 3.14)
project(Titan LANGUAGES C CXX) # use CXX, CUDA by default (since CUDA is a language, don't need cuda_add_executable)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CMAKE_CXX_EXTENSIONS OFF) ## on g++ this ensures: -std=c++11 and not -std=gnu++11

# https://cliutils.gitlab.io/modern-cmake/chapters/packages/CUDA.html
if(NOT DEFINED CMAKE_CUDA_STANDARD)
    set(CMAKE_CUDA_STANDARD 17) # set cuda cmake standard to c++17
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
endif()
set(CMAKE_CXX_STANDARD 17) # set C++ standard to C++17


# https://github.com/microsoft/vcpkg/blob/master/docs/users/integration.md#using-an-environment-variable-instead-of-a-command-line-option
# SET CMAKE_TOOLCHAIN_FILE
# you can add VCPKG_ROOT= your/vcpkg/root/folder as environment variable
if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
      CACHE STRING "")
endif()
	
find_package(msgpack CONFIG REQUIRED)

add_executable(send 
    send.cpp)
target_link_libraries(send PRIVATE msgpackc-cxx)

add_executable(recv 
    recv.cpp)
target_link_libraries(recv PRIVATE msgpackc-cxx)