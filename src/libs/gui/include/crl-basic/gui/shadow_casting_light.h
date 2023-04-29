#pragma once

#include "crl-basic/gui/light.h"

namespace crl {
namespace gui {

class ShadowCastingLight : public Light {
public:
    float lightSize = 1;
    float s = 1;  // shadow bounding box scale factor
    float left = -50.0f, right = 50.0f, bottom = -50.0f, top = 50.0f, zNear = 0.10f, zFar = 1000.0f;
    void setRange(float left = -50.0f, float right = 50.0f, float bottom = -50.0f, float top = 50.0f, float zNear = 0.1f, float zFar = 1000.0f) {
        this->left = left;
        this->right = right;
        this->bottom = bottom;
        this->top = top;
        this->zNear = zNear;
        this->zFar = zFar;
    }

    glm::mat4 getOrthoProjectionMatrix() {
        return glm::ortho<float>(s * left, s * right, s * bottom, s * top, zNear, zFar);
    }

    glm::mat4 getViewMatrix() {
        return glm::lookAt(toGLM(pos), toGLM(target), toGLM(worldUp));
    }
};

}  // namespace gui
}  // namespace crl