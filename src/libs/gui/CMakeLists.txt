cmake_minimum_required(VERSION 3.5)

project(gui)

set(CRL_TARGET_NAME ${PROJECT_NAME})

set(CRL_SHADER_FOLDER
        "\"${CMAKE_CURRENT_SOURCE_DIR}/shaders\""
        CACHE STRING ""
        )

file(
        GLOB
        CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/include/crl-basic/gui/*.h" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
        "${CMAKE_CURRENT_SOURCE_DIR}/imgui_widgets/include/imgui_widgets/*.h" #
        "${CMAKE_CURRENT_SOURCE_DIR}/imgui_widgets/*.cpp" #
)

# dependencies
list(
        APPEND
        CRL_TARGET_DEPENDENCIES #
        "glm" #
        "imgui" #
        "stb_image" #
        "stl_reader" #
        "nlohmann_json::nlohmann_json" #
        "tinyobjloader" #
        "eigen" #
        "crl::utils" #
)

# target include dirs
list(
        APPEND CRL_TARGET_INCLUDE_DIRS #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include" #
        PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/imgui_widgets/include" #
)

# target link libs
list(
        APPEND
        CRL_TARGET_LINK_LIBS #
        PUBLIC "glm" #
        PUBLIC "imgui" #
        PUBLIC "stb_image" #
        PUBLIC "stl_reader" #
        PUBLIC "nlohmann_json::nlohmann_json" #
        PUBLIC "tinyobjloader" #
        PUBLIC "eigen" #
        PUBLIC "crl::utils" #
)

# compile definitions
list(
        APPEND
        CRL_COMPILE_DEFINITIONS #
        PUBLIC "CRL_DATA_FOLDER=${CRL_DATA_FOLDER}" #
        PUBLIC "CRL_SHADER_FOLDER=${CRL_SHADER_FOLDER}" #
        PUBLIC "CRL_IMGUI_FONT_FOLDER=${CRL_IMGUI_FONT_FOLDER}" #
        PUBLIC "IMGUI_IMPL_OPENGL_LOADER_GLAD" #
)

# we use filesystem which is "experimental" for some compilers for those we need
# to add fs lib to link libraries
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    # gcc < 7: supports fs as experimentals
    # gcc = 8: supports fs but needs to link stdc++fs
    # gcc > 9: supports fs as std library
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
        list(APPEND CRL_TARGET_LINK_LIBS PUBLIC "stdc++fs")
    endif ()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    # msvc
    # TODO:
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # llvm clang
    # ~~~
    # llvm clang < 9.0 needs to link stdc++fs
    if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9.0)
        list(APPEND CRL_TARGET_LINK_LIBS PUBLIC "stdc++fs")
    endif ()
endif ()

# create target
create_crl_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}" #
)