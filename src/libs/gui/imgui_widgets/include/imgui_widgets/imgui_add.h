#pragma once

#pragma warning(disable : 4244)

#include <imgui.h>
#include <imgui_internal.h>

namespace ImGui {

inline IMGUI_API bool SliderDouble(const char* label, double* v, double v_min, double v_max, const char* format = NULL, ImGuiSliderFlags flags = 0) {
    return ImGui::SliderScalar(label, ImGuiDataType_Double, v, &v_min, &v_max, format, flags);
}

inline IMGUI_API bool InputUInt(const char* label, unsigned int* v, unsigned int step = 1, unsigned int step_fast = 100, ImGuiInputTextFlags flags = 0) {
    // Hexadecimal input provided as a convenience but the flag name is awkward. Typically you'd use InputText() to parse your own data, if you want to handle prefixes.
    const char* format = (flags & ImGuiInputTextFlags_CharsHexadecimal) ? "%08X" : "%d";
    return ImGui::InputScalar(label, ImGuiDataType_U32, (void*)v, (void*)(step > 0 ? &step : NULL), (void*)(step_fast > 0 ? &step_fast : NULL), format, flags);
}

inline IMGUI_API bool SliderUInt(const char* label, unsigned int* v, unsigned int v_min, unsigned int v_max, const char* format = "%d",
                                 ImGuiSliderFlags flags = 0) {
    return ImGui::SliderScalar(label, ImGuiDataType_U32, v, &v_min, &v_max, format, flags);
}

inline bool ToggleButton(const char* str_id, bool* v) {
    bool clicked = false;
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float height = ImGui::GetFrameHeight();
    float width = height * 1.55f;
    float radius = height * 0.50f;

    ImGui::InvisibleButton(str_id, ImVec2(width, height));
    if (ImGui::IsItemClicked()) {
        clicked = true;
        *v = !*v;
    }

    float t = *v ? 1.0f : 0.0f;

    ImGuiContext& g = *GImGui;
    float ANIM_SPEED = 0.08f;
    if (g.LastActiveId == g.CurrentWindow->GetID(str_id))  // && g.LastActiveIdTimer < ANIM_SPEED)
    {
        float t_anim = ImSaturate(g.LastActiveIdTimer / ANIM_SPEED);
        t = *v ? (t_anim) : (1.0f - t_anim);
    }

    ImU32 col_bg;
    if (ImGui::IsItemHovered())
        col_bg = ImGui::GetColorU32(ImLerp(ImVec4(0.78f, 0.78f, 0.78f, 1.0f), ImVec4(0.64f, 0.83f, 0.34f, 1.0f), t));
    else
        col_bg = ImGui::GetColorU32(ImLerp(ImVec4(0.85f, 0.85f, 0.85f, 1.0f), ImVec4(0.56f, 0.83f, 0.26f, 1.0f), t));

    draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, height * 0.5f);
    draw_list->AddCircleFilled(ImVec2(p.x + radius + t * (width - radius * 2.0f), p.y + radius), radius - 1.5f, IM_COL32(255, 255, 255, 255));

    return clicked;
}

inline bool PlayPauseButton(const char* str_id, bool* v) {
    bool clicked = false;
    ImVec2 p = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    float height = ImGui::GetFrameHeight();
    float width = height * 1.55f;
    float radius = height * 0.50f;

    ImGui::InvisibleButton(str_id, ImVec2(width, height));
    if (ImGui::IsItemClicked()) {
        clicked = true;
        *v = !*v;
    }

    ImU32 col_bg;

    if (*v) {
        if (ImGui::IsItemHovered())
            col_bg = ImGui::GetColorU32(ImVec4(0.54f, 0.73f, 0.3f, 1.0f));
        else
            col_bg = ImGui::GetColorU32(ImVec4(0.64f, 0.83f, 0.34f, 1.0f));
    } else {
        if (ImGui::IsItemHovered())
            col_bg = ImGui::GetColorU32(ImVec4(0.75f, 0.75f, 0.75f, 1.0f));
        else
            col_bg = ImGui::GetColorU32(ImVec4(0.85f, 0.85f, 0.85f, 1.0f));
    }

    draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), col_bg, height * 0.25f);

    if (!*v)
        draw_list->AddTriangleFilled(ImVec2(p.x + width * 0.35, p.y + height * 0.2), ImVec2(p.x + width * 0.65, p.y + height * 0.5),
                                     ImVec2(p.x + width * 0.35, p.y + height * 0.8), ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 1.0f)));
    else {
        draw_list->AddRectFilled(ImVec2(p.x + width * 0.3, p.y + height * 0.2), ImVec2(p.x + width * 0.45, p.y + height * 0.8),
                                 ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 1.0f)), 0);
        draw_list->AddRectFilled(ImVec2(p.x + width * 0.55, p.y + height * 0.2), ImVec2(p.x + width * 0.7, p.y + height * 0.8),
                                 ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 1.0f)), 0);
    }

    return clicked;
}

};  // namespace ImGui
