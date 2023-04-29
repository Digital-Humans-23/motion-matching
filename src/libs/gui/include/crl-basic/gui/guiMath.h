#pragma once

#include "crl-basic/utils/mathUtils.h"
#include "glm/glm.hpp"
#include "glm/gtc/type_ptr.hpp"
#include "imgui.h"
#include "imgui_widgets/ImGuizmo.h"

namespace crl {
namespace gui {

inline glm::vec3 toGLM(const Vector3d &v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

inline glm::vec3 toGLM(const P3D &p) {
    return glm::vec3(p.x, p.y, p.z);
}

inline V3D toV3D(const glm::vec3 &v) {
    return V3D(v[0], v[1], v[2]);
}

inline P3D toP3D(const glm::vec3 &v) {
    return P3D(v[0], v[1], v[2]);
}

inline V3D toV3D(const float *v) {
    return V3D(v[0], v[1], v[2]);
}

inline P3D toP3D(const float *v) {
    return P3D(v[0], v[1], v[2]);
}

inline glm::mat4 getGLMTransform(const V3D &scale, const Quaternion &orientation, const P3D &position) {
    // scale, then rotate, then translate...
    glm::mat4 transform = glm::mat4(1.0);
    AngleAxisd rot(orientation);
    transform = glm::translate(transform, toGLM(position));
    transform = transform * glm::rotate(glm::mat4(1.0), (float)(rot.angle()), toGLM(rot.axis()));
    transform = transform * glm::scale(glm::mat4(1.0), toGLM(scale));
    return transform;
}

inline void setTransformFromWidgets(const glm::mat4 &cameraView, const glm::mat4 &cameraProjection, glm::mat4 &matrix) {
    static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);
    static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
    static bool useSnap = false;
    static float snap[3] = {1.f, 1.f, 1.f};
    static float bounds[] = {-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f};
    static float boundsSnap[] = {0.1f, 0.1f, 0.1f};
    static bool boundSizing = false;
    static bool boundSizingSnap = false;

    if (ImGui::IsKeyPressed('W') || ImGui::IsKeyPressed('w'))
        mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
    if (ImGui::IsKeyPressed('E') || ImGui::IsKeyPressed('e'))
        mCurrentGizmoOperation = ImGuizmo::ROTATE;
    if (ImGui::IsKeyPressed('R') || ImGui::IsKeyPressed('r'))  // r Key
        mCurrentGizmoOperation = ImGuizmo::SCALE;
    if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
        mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
        mCurrentGizmoOperation = ImGuizmo::ROTATE;
    ImGui::SameLine();
    if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
        mCurrentGizmoOperation = ImGuizmo::SCALE;

    float matrixTranslation[3], matrixRotation[3], matrixScale[3];
    ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(matrix), matrixTranslation, matrixRotation, matrixScale);
    ImGui::InputFloat3("Tr", matrixTranslation);
    ImGui::InputFloat3("Rt", matrixRotation);
    ImGui::InputFloat3("Sc", matrixScale);
    ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, glm::value_ptr(matrix));

    if (mCurrentGizmoOperation == ImGuizmo::SCALE)
        mCurrentGizmoMode = ImGuizmo::LOCAL;
    else
        mCurrentGizmoMode = ImGuizmo::WORLD;

    if (ImGui::IsKeyPressed(83))
        useSnap = !useSnap;
    ImGui::Checkbox("##UseSnap", &useSnap);
    ImGui::SameLine();

    switch (mCurrentGizmoOperation) {
        case ImGuizmo::TRANSLATE:
            ImGui::InputFloat3("Snap", &snap[0]);
            break;
        case ImGuizmo::ROTATE:
            ImGui::InputFloat("Angle Snap", &snap[0]);
            break;
        case ImGuizmo::SCALE:
            ImGui::InputFloat("Scale Snap", &snap[0]);
            break;
        default:
            break;
    }

    ImGui::Checkbox("Bounds", &boundSizing);
    if (boundSizing) {
        ImGui::PushID(3);
        ImGui::Checkbox("##BoundSizingSnap", &boundSizingSnap);
        ImGui::SameLine();
        ImGui::InputFloat3("Snap", boundsSnap);
        ImGui::PopID();
    }

    ImGuiIO &io = ImGui::GetIO();
    ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
    ImGuizmo::Manipulate(glm::value_ptr(cameraView), glm::value_ptr(cameraProjection), mCurrentGizmoOperation, mCurrentGizmoMode, glm::value_ptr(matrix), NULL,
                         useSnap ? &snap[0] : NULL, boundSizing ? bounds : NULL, boundSizingSnap ? boundsSnap : NULL);
}

inline void setTransformFromWidgets(const glm::mat4 &cameraView, const glm::mat4 &cameraProjection, V3D &scale, Quaternion &orientation, P3D &position) {
    glm::mat4 transform = getGLMTransform(scale, orientation, position);
    setTransformFromWidgets(cameraView, cameraProjection, transform);

    float matrixTranslation[3], matrixRotation[3], matrixScale[3];
    ImGuizmo::DecomposeMatrixToComponents(glm::value_ptr(transform), matrixTranslation, matrixRotation, matrixScale);

    scale = toV3D(matrixScale);
    position = toP3D(matrixTranslation);
    orientation = Quaternion(AngleAxisd(RAD(matrixRotation[2]), V3D(0, 0, 1))) * Quaternion(AngleAxisd(RAD(matrixRotation[1]), V3D(0, 1, 0))) *
                  Quaternion(AngleAxisd(RAD(matrixRotation[0]), V3D(1, 0, 0)));
}

}  // namespace gui
}  // namespace crl