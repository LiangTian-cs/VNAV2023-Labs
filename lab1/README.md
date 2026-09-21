# MIT VNAV 2023 - Lab 1

This directory contains the Lab 1 exercises:

- Shell Exercise 1
  - `dante.txt`
  - `exercise1.txt`

- Shell Exercise 2
  - `fortunes.txt`

- C++ Warm-up
  - `cpp-warmup.txt`

- RandomVector
  - `RandomVector/random_vector.h`
  - `RandomVector/random_vector.cpp`
  - `RandomVector/main.cpp`
  - `RandomVector/CMakeLists.txt`

## Compile RandomVector exactly in the style requested by MIT

```bash
cd RandomVector

g++ -std=c++11 -Wall -pedantic \
  -o random_vector \
  main.cpp random_vector.cpp

./random_vector

d RandomVector

cmake -S . -B build
cmake --build build -j

./build/random_vector

