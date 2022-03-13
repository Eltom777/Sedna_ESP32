cmake_minimum_required(VERSION 3.16.0)
list(APPEND EXTRA_COMPONENT_DIRS "./common/components/esp-google-iot")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Sedna_ESP32)
