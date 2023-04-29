# FetchContent command is available with cmake >= 3.11
include(FetchContent)

# -----------------------------------------------------------------------------
# helper funtion to download a 'fetch content declare'
# -----------------------------------------------------------------------------

# download project and add it to build tree
macro(fetch_and_add what)
    FetchContent_GetProperties("${what}")
    if (NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
        add_subdirectory(${${what}_SOURCE_DIR} ${what})
    endif ()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()

# only download project
macro(fetch what)
    FetchContent_GetProperties("${what}")
    if (NOT ${${what}_POPULATED})
        message(STATUS "fetching ${what} ...")
        FetchContent_Populate(${what})
    endif ()
    mark_as_advanced(${${what}_SOURCE_DIR})
endmacro()

# -----------------------------------------------------------------------------
# dependencies for utils
# -----------------------------------------------------------------------------

# Eigen library
#
# download as zip, so it's a bit smaller than downloading the whole repo
if (NOT TARGET eigen)
    FetchContent_Declare(
            eigen #
            URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip #
            URL_HASH MD5=994092410ba29875184f7725e0371596
    )
    fetch(eigen)
    add_library(eigen INTERFACE)
    add_library(Eigen3::Eigen ALIAS eigen)
    target_include_directories(eigen SYSTEM INTERFACE ${eigen_SOURCE_DIR})
endif ()

# -----------------------------------------------------------------------------
# json
FetchContent_Declare(
        json #
        URL https://github.com/nlohmann/json/releases/download/v3.7.3/include.zip #
        URL_HASH MD5=fb96f95cdf609143e998db401ca4f324 #
)
fetch(json)

# this is a bit mess... (alias name on DDE cmake has some typo)
if (TARGET nlohmann_json)
    if (NOT TARGET nlohmann_json::nlohmann_json)
        add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
    endif (NOT TARGET nlohmann_json::nlohmann_json)
else ()
    add_library(nlohmann_json INTERFACE)
    add_library(nlohmann_json::nlohmann_json ALIAS nlohmann_json)
    target_include_directories(
            nlohmann_json SYSTEM INTERFACE $<BUILD_INTERFACE:${json_SOURCE_DIR}/include>
    )
endif ()

# -----------------------------------------------------------------------------
# dependencies for gui
# -----------------------------------------------------------------------------

# glad
if (NOT TARGET glad)
    set(OpenGL_GL_PREFERENCE GLVND)
    add_subdirectory(glad)
endif ()

# -----------------------------------------------------------------------------
# glfw
FetchContent_Declare(
        glfw
        URL https://github.com/glfw/glfw/archive/3.3.2.zip
        URL_HASH MD5=69d0aa90a9278c0dd8be975d1ae4aaf2
        UPDATE_COMMAND "" CONFIGURE_COMMAND "" BUILD_COMMAND "" INSTALL_COMMAND ""
)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "GLFW lib only")
set(GLFW_BUILD_TESTS OFF CACHE BOOL "GLFW lib only")
set(GLFW_BUILD_DOCS OFF CACHE BOOL "GLFW lib only")
set(GLFW_INSTALL OFF CACHE BOOL "GLFW lib only")
fetch_and_add(glfw)

# -----------------------------------------------------------------------------
# glm
FetchContent_Declare(
        glm #
        URL https://github.com/g-truc/glm/releases/download/0.9.9.7/glm-0.9.9.7.zip #
        URL_HASH MD5=da4740b5b41a1e2ccb2955af74e7c298 #
)
set(GLM_TEST_ENABLE OFF CACHE BOOL "GLM lib only")
fetch_and_add(glm)
get_target_property(GLM_INTERFACE_INC glm INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(glm SYSTEM INTERFACE ${GLM_INTERFACE_INC})

# -----------------------------------------------------------------------------
# imgui
#
# imgui for graphical user interface. since imgui is not a cmake project, we
# need to add target by our own. we use imgui implementation for glfw and
# opengl3

if (NOT TARGET imgui)
    FetchContent_Declare(
            imgui #
            URL https://github.com/ocornut/imgui/archive/refs/tags/v1.86.zip #
            URL_HASH MD5=bccb1c173296979e6a61b4f9da572429 #
    )
    fetch(imgui)

    file(
            GLOB
            IMGUI_SRC #
            ${imgui_SOURCE_DIR}/*.cpp #
            ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp #
            ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp #
    )

    add_library(imgui STATIC ${IMGUI_SRC})

    if (NOT WIN32)
        # Add this line, if GLAD should be used (is needed for fresh Ubuntu)
        target_compile_definitions(imgui PUBLIC -DIMGUI_IMPL_OPENGL_LOADER_GLAD)
    endif (NOT WIN32)

    set_target_properties(imgui PROPERTIES LINKER_LANGUAGE CXX)

    target_include_directories(
            imgui PUBLIC ${imgui_SOURCE_DIR} #
            ${imgui_SOURCE_DIR}/examples #
    )

    target_link_libraries(
            imgui PUBLIC glfw #
            glad #
    )

    set(CRL_IMGUI_FONT_FOLDER
            "\"${imgui_SOURCE_DIR}/misc/fonts\""
            CACHE STRING ""
            )
endif ()

# -----------------------------------------------------------------------------
# stb_image
if (NOT TARGET stb_image)
    add_subdirectory(stb_image)
endif ()

# -----------------------------------------------------------------------------
# stl_reader
if (NOT TARGET stl_reader)
    add_subdirectory(stl_reader)
endif ()

# -----------------------------------------------------------------------------
# tinyobjloader
FetchContent_Declare(
        tinyobjloader #
        URL https://github.com/tinyobjloader/tinyobjloader/archive/v2.0-rc1.zip #
        URL_HASH MD5=f660a9f91fc2ea5c4f33d7530f6a69bc #
)
fetch_and_add(tinyobjloader)

# -----------------------------------------------------------------------------
# google test
# Added latest tag that passes their CI stuff.
if (NOT TARGET google-test)
    FetchContent_Declare(
            google-test #
            GIT_REPOSITORY https://github.com/google/googletest.git #
            GIT_TAG release-1.11.0 #
    )
    # gtest_force_shared_crt require for windows compatibility.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    fetch(google-test)
    add_subdirectory(${google-test_SOURCE_DIR} google-test)
endif ()
