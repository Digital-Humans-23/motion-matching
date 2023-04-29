#include "crl-basic/gui/shadow_map_fbo.h"

#include <stdio.h>

#include "crl-basic/gui/glUtils.h"

namespace crl {
namespace gui {

ShadowMapFBO::ShadowMapFBO() {
    fbo = 0;
    shadowMap = 0;
}

ShadowMapFBO::~ShadowMapFBO() {
    //Disabled this, since it causes OpenGL to throw errors
    //if (fbo != 0) {
    //    GLCall(glDeleteFramebuffers(1, &fbo));
    //}

    //if (shadowMap != 0) {
    //    GLCall(glDeleteTextures(1, &shadowMap));
    //}
}

bool ShadowMapFBO::Init(GLuint bufferWidth, GLuint bufferHeight) {
    // Create the FBO
    GLCall(glGenFramebuffers(1, &fbo));

    // Create the depth buffer
    GLCall(glGenTextures(1, &shadowMap));
    GLCall(glBindTexture(GL_TEXTURE_2D, shadowMap));
    GLCall(glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT32, bufferWidth, bufferHeight, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL));
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR));
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR));
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE));
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL));
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER));
    GLCall(glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER));
    float borderColor[] = {1.0f, 1.0f, 1.0f, 1.0f};
    GLCall(glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor));

    GLCall(glBindFramebuffer(GL_FRAMEBUFFER, fbo));
    GLCall(glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowMap, 0));

    // Disable writes to the color buffer
    GLCall(glDrawBuffer(GL_NONE));
    GLCall(glReadBuffer(GL_NONE));

    GLCall(GLenum Status = glCheckFramebufferStatus(GL_FRAMEBUFFER));

    if (Status != GL_FRAMEBUFFER_COMPLETE) {
        printf("FB error, status: 0x%x\n", Status);
        return false;
    }
    this->bufferHeight = bufferHeight;
    this->bufferWidth = bufferWidth;
    return true;
}

void ShadowMapFBO::BindForWriting() {
    GLCall(glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo));
}

void ShadowMapFBO::BindForReading(GLuint TextureUnit) {
    GLCall(glActiveTexture(TextureUnit));
    GLCall(glBindTexture(GL_TEXTURE_2D, shadowMap));
}

}  // namespace gui
}  // namespace crl