project(glad C)

find_package(OpenGL REQUIRED)

add_library(glad STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/glad.c
)

target_include_directories(glad PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(glad 
	${OPENGL_LIBRARIES} 
	${CMAKE_DL_LIBS}
)
