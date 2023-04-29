#include "crl-basic/gui/inputstate.h"

namespace crl {
namespace gui {

MouseState::MouseState() {}

MouseState::~MouseState() {}

void MouseState::onMouseClick(double xPos, double yPos, int button, int action, int mods) {
    this->mods = mods;

    lastMouseX = xPos;
    lastMouseY = yPos;

    dragging = (action == GLFW_PRESS);

    if (button == GLFW_MOUSE_BUTTON_LEFT)
        lButtonPressed = (action != GLFW_RELEASE);
    if (button == GLFW_MOUSE_BUTTON_MIDDLE)
        mButtonPressed = (action != GLFW_RELEASE);
    if (button == GLFW_MOUSE_BUTTON_RIGHT)
        rButtonPressed = (action != GLFW_RELEASE);
}

void MouseState::onMouseMove(double xPos, double yPos) {
    mouseMoveX = lastMouseX - xPos;
    mouseMoveY = -lastMouseY + yPos;
    lastMouseX = xPos;
    lastMouseY = yPos;
}

}  // namespace gui
}  // namespace crl