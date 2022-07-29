cmake_minimum_required(VERSION 3.1...3.23)
project(Charger_Runner)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE Debug)

add_executable(res
    main.cpp
    solution.h
    solution.cpp
    network.cpp
    network.h
    geo_utils/SphericalUtil.h
)
