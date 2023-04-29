#pragma once

#include "GLFW/glfw3.h"

#include <map>

namespace crl {
namespace gui {

/**
 * Keep track of the mouse state, positions, buttons pressed, etc...
 */
class MouseState {
public:
    // keep track of the last mouse position
    double lastMouseX = 0, lastMouseY = 0;
    double mouseMoveX = 0, mouseMoveY = 0;

    bool rButtonPressed = false, lButtonPressed = false, mButtonPressed = false;
    bool dragging = false;

    int mods = 0;

public:
    MouseState();
    ~MouseState();
    void onMouseClick(double xPos, double yPos, int button, int action, int mods);
    void onMouseMove(double xPos, double yPos);
};

typedef std::map<int, bool> KeyboardState;

}  // namespace gui
}  // namespace crl