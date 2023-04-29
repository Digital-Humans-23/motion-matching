#pragma once

#include "glad/glad.h"

namespace crl {
namespace gui {

class ShadowMapFBO {
public:
    ShadowMapFBO();
    ~ShadowMapFBO();

    bool Init(GLuint bufferWidth, GLuint bufferHeight);

    void BindForWriting();

    void BindForReading(GLuint TextureUnit);
    GLuint bufferWidth, bufferHeight;
    GLuint fbo;
    GLuint shadowMap;
};

}  // namespace gui
}  // namespace crl