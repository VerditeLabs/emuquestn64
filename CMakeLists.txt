cmake_minimum_required(VERSION 3.24)
project(emuquest C)

set(CMAKE_C_STANDARD 23)



add_executable(emuquest main.c)

if(APPLE)
    target_sources(emuquest PUBLIC sokol.m)
    target_link_libraries(emuquest "-framework Metal -framework Cocoa -framework MetalKit -framework Quartz -framework AudioToolbox")
else()
    target_sources(emuquest PUBLIC sokol.c)
endif()

target_include_directories(emuquest PUBLIC thirdparty/Nuklear thirdparty/sokol thirdparty/sokol/util)