#pragma once

#include "glad/glad.h"

#include <iostream>

/**
    Source: https://www.youtube.com/watch?v=FBbPWSOQ0-w&list=PLlrATfBNZ98foTJPJ_Ev03o2oq3-GGOS2&index=10
*/

namespace crl {
namespace gui {

#ifdef NDEBUG
#define GLCall(x) x;
#else
static void GLClearError() {
    GLenum error;
    while ((error = glGetError()) != GL_NO_ERROR) {
        std::cout << "Cleared OpenGL Error " << error << "..." << std::endl;
    }
}

static void GLLogCall(const char* function, const char* file, int line) {
    while (GLenum error = glGetError())
        std::cout << "[OpenGL Error] (" << error << "): " << function << " " << file << ": " << line << std::endl;
}

#define GLCall(x)   \
    GLClearError(); \
    x;              \
    GLLogCall(#x, __FILE__, __LINE__)
#endif
}  // namespace gui
}  // namespace crl