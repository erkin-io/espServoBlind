# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Version embedded in the app image (esp_app_desc); keep in step with FW_VERSION_STR
set(PROJECT_VER "1.1.0")
project(erkinIO_esp32_servo_blind)
