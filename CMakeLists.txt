# TestAPEN Autopilot - Top-level CMakeLists.txt
# Fork of TestAPEN using ESP-NOW instead of CAN bus

cmake_minimum_required(VERSION 3.16)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(testapen)
