cmake_minimum_required(VERSION 3.5)

project(utils)

set(CMAKE_CXX_STANDARD 11)

set(CRL_TARGET_NAME ${PROJECT_NAME})

file(
        GLOB CRL_SOURCES #
        "include/crl-basic/utils/*.h" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# dependencies
list(
        APPEND CRL_TARGET_DEPENDENCIES #
        "eigen" #
        "nlohmann_json::nlohmann_json" #
)

# target include dirs
list(
        APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# target link libs
list(
        APPEND CRL_TARGET_LINK_LIBS #
        PUBLIC "eigen" #
        PUBLIC "nlohmann_json::nlohmann_json" #
)

# compile definitions
list(
        APPEND CRL_COMPILE_DEFINITIONS #
        PUBLIC "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
)

# create target
create_crl_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}" #
)

set(CRL_TEST_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/test/trajectory.cpp" #
)

# create test
create_crl_test(
        test_${CRL_TARGET_NAME}
        "${CRL_TEST_SOURCES}" #
        "${CRL_TARGET_NAME}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}" #
)
