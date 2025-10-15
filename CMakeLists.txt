cmake_minimum_required(VERSION 3.16 FATAL_ERROR)

# Set project metadata
project(CANopenNode VERSION 2.0.375 LANGUAGES C;CXX)

#
# Compiler configuration
#
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED YES)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

#
# Project configuration options
#
option(BUILD_EXAMPLES           "Build examples" OFF)
option(BUILD_TOOLS              "Build tools" OFF)
option(USE_DEPRECATED_MACROS    "Enable deprecated preprocessor defines and macros" ON)

# Display chosen options
message(STATUS "C Standard: ${CMAKE_C_STANDARD}")
message(STATUS "C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "Build Examples: ${BUILD_EXAMPLES}")
message(STATUS "Build Tools: ${BUILD_TOOLS}")
message(STATUS "Use Deprecated Macros: ${USE_DEPRECATED_MACROS}")

add_library(${PROJECT_NAME} STATIC)

file(GLOB_RECURSE 301_SRC ${PROJECT_SOURCE_DIR}/301/*.c)
file(GLOB_RECURSE 303_SRC ${PROJECT_SOURCE_DIR}/303/*.c)
file(GLOB_RECURSE 304_SRC ${PROJECT_SOURCE_DIR}/304/*.c)
file(GLOB_RECURSE 305_SRC ${PROJECT_SOURCE_DIR}/305/*.c)
file(GLOB_RECURSE 309_SRC ${PROJECT_SOURCE_DIR}/309/*.c)
file(GLOB_RECURSE EXTRA_SRC ${PROJECT_SOURCE_DIR}/extra/*.c)
file(GLOB_RECURSE HANDMADE_SRC ${PROJECT_SOURCE_DIR}/handmade/*.c)

target_sources(${PROJECT_NAME} PRIVATE ${301_SRC} ${303_SRC} ${304_SRC} ${305_SRC} ${309_SRC} ${EXTRA_SRC} ${HANDMADE_SRC})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/handmade
    INTERFACE
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>/handmade)

    target_compile_definitions(${PROJECT_NAME} PUBLIC CANOPENNODE_STATIC_USE)
