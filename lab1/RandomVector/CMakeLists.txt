cmake_minimum_required(VERSION 3.10)

project(
  VNAV_Lab1_RandomVector
  VERSION 1.0
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
  random_vector_lib
  random_vector.cpp
  random_vector.h
)

target_compile_options(
  random_vector_lib
  PRIVATE
  -Wall
  -Wextra
  -pedantic
)

add_executable(
  random_vector
  main.cpp
)

target_link_libraries(
  random_vector
  PRIVATE
  random_vector_lib
)
