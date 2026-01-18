cmake_minimum_required(VERSION 3.27)

project(PMSM_MOTOR_SIM VERSION 0.1)


add_executable(MOTORSIM)

include_directories(include)


target_sources(MOTORSIM PRIVATE
src/main.cpp
# src/params.cpp
src/controller.cpp
src/mcb.cpp
src/motorModel.cpp
src/inverter.cpp
)
