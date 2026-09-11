cmake_minimum_required(VERSION 3.10)
project(rdes)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -fPIC")

add_library(rdes SHARED src/rdes.c)
target_include_directories(rdes PUBLIC src)
