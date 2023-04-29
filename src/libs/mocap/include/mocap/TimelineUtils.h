//
// Created by Dongho Kang on 31.12.21.
//

#ifndef CRL_TIMELINEUTILS_H
#define CRL_TIMELINEUTILS_H

#include <imgui.h>
#include <imgui_widgets/ImGuizmo.h>

#include <map>
#include <string>
#include <vector>

namespace crl::mocap {

class Timeline {
public:
    typedef std::map<std::string, std::vector<std::pair<double, double>>> TimelineData;

    // in sec
    double timeWindow = 3;

    // options
    mutable bool drawLabels = true;
    // window size options in pixels
    double timelineWidth = 1300;
    double labelWidth = 100;

private:
    // timeline data
    const TimelineData &data_;

public:
    explicit Timeline(const TimelineData &data) : data_(data) {}

    virtual ~Timeline() = default;

    void draw(double t) const {
        ImGui::SetNextWindowBgAlpha(1.0);
        ImGui::Begin("Contact Schedule Visualizer");
        ImGuizmo::BeginFrame();
        ImGui::Checkbox("draw labels", &drawLabels);

        // this is where the window screen starts...
        ImVec2 p = ImGui::GetCursorScreenPos();
        ImDrawList *draw_list = ImGui::GetWindowDrawList();

        double timeStart = t - timeWindow * 0.25;
        double timeEnd = t + timeWindow;

        // this is for every row which will visualize different data sequence.
        float height = ImGui::GetFrameHeight();
        float radius = height * 0.005f;
        ImU32 col_bg = ImGui::GetColorU32(ImVec4(0.0f, 0.0f, 0.0f, 1.0f));
        draw_list->AddRectFilled(ImVec2(p.x, p.y), ImVec2(p.x + labelWidth + timelineWidth + 20, p.y + (float)data_.size() * height), col_bg);

        ImU32 col_text_gray = ImGui::GetColorU32(ImVec4(0.8f, 0.8f, 1.0f, 1.0f));
        ImU32 col_line = ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
        ImU32 col_cursor = ImGui::GetColorU32(ImVec4(1.0f, 0.7f, 0.7f, 1.0f));
        ImU32 col_horizon = ImGui::GetColorU32(ImVec4(0.5f, 0.5f, 0.5f, 1.0f));
        ImU32 col_text_dark = ImGui::GetColorU32(ImVec4(0.1f, 0.1f, 0.1f, 1.0f));

        // draw the swing phases now...
        float rowOffset = 0;
        for (const auto &row : data_) {
            for (const auto &se : row.second) {
                float start = getWindowCoord((float)se.first, timeStart, timeEnd);
                float end = getWindowCoord((float)se.second, timeStart, timeEnd);
                if (end > labelWidth && start < labelWidth + timelineWidth) {
                    ImU32 col = ImGui::GetColorU32(ImVec4(0.7f, 0.7f, 0.7f, 1.0f));
                    if (start < labelWidth - 5)
                        start = labelWidth - 5;
                    if (end > labelWidth + timelineWidth + 5)
                        end = labelWidth + timelineWidth + 5;
                    draw_list->AddRectFilled(ImVec2(p.x + start, p.y + rowOffset + height * 0.15),  //
                                             ImVec2(p.x + end, p.y + rowOffset + height * 0.85),    //
                                             col, height * radius);
                }
            }
            rowOffset += height;
        }

        // cover up the loose ends of the swing phases...
        draw_list->AddRectFilled(ImVec2(p.x, p.y), ImVec2(p.x + labelWidth, p.y + rowOffset), col_bg);
        draw_list->AddRectFilled(ImVec2(p.x + labelWidth + timelineWidth, p.y), ImVec2(p.x + labelWidth + timelineWidth + 20, p.y + rowOffset), col_bg);

        // draw the grid, the labels of the limbs, and the time cursor...
        rowOffset = 0;
        draw_list->AddLine(ImVec2(p.x, p.y + rowOffset), ImVec2(p.x + labelWidth + timelineWidth, p.y + rowOffset), col_line);
        for (const auto &row : data_) {
            draw_list->AddText(ImVec2(p.x + height * 0.5, p.y + rowOffset + height * 0.1), col_text_gray, row.first.c_str());
            rowOffset += height;
            draw_list->AddLine(ImVec2(p.x, p.y + rowOffset), ImVec2(p.x + labelWidth + timelineWidth, p.y + rowOffset), col_line);
        }
        draw_list->AddLine(ImVec2(p.x, p.y), ImVec2(p.x, p.y + rowOffset), col_line);
        draw_list->AddLine(ImVec2(p.x + labelWidth, p.y), ImVec2(p.x + labelWidth, p.y + rowOffset), col_line);
        draw_list->AddLine(ImVec2(p.x + labelWidth + timelineWidth, p.y), ImVec2(p.x + labelWidth + timelineWidth, p.y + rowOffset), col_line);

        // show where the current moment in time is...
        float cursorVal;
        cursorVal = getWindowCoord(t, timeStart, timeEnd);
        draw_list->AddLine(ImVec2(p.x + cursorVal, p.y), ImVec2(p.x + cursorVal, p.y + rowOffset), col_cursor);
        char timeText[100];
        sprintf(timeText, "t = %2.2lfs", t);
        draw_list->AddText(ImVec2(p.x + cursorVal - height / 2.0, p.y + rowOffset + height * 0.5), col_cursor, timeText);
        draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y - 5), 5, col_cursor);
        draw_list->AddCircleFilled(ImVec2(p.x + cursorVal, p.y + rowOffset + 5), 5, col_cursor);
        ImGui::End();
    }

private:
    float getWindowCoord(double tVal, double tStart, double tEnd) const {
        double p = (tVal - tStart) / (tEnd - tStart);
        return labelWidth + (float)p * (timelineWidth);
    }
};

}  // namespace crl::mocap

#endif  //CRL_TIMELINEUTILS_H
