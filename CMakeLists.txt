cmake_minimum_required(VERSION 3.1...3.23)
project(Charger_Runner)

SET(CMAKE_CXX_STANDARD 14)
SET(CMAKE_BUILD_TYPE Release)

add_executable(res
    main.cpp
    solution.h
    solution.cpp
    utm/utm.h
    utm/utm.cpp
    utm/datum.h
    utm/datum.cpp
    network.cpp
    network.h
)
