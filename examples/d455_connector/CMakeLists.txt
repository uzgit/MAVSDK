cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(stream)

add_executable(stream
    stream.cpp
)

find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Threads REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries(stream
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    realsense2
    ${CMAKE_THREAD_LIBS_INIT}
)

if(NOT MSVC)
    add_compile_options(battery PRIVATE -Wall -Wextra)
else()
    add_compile_options(battery PRIVATE -WX -W2)
endif()

