//
// Created by Dongho Kang on 12.12.21.
//

#ifndef CRL_BASIC_PLOTUTILS_H
#define CRL_BASIC_PLOTUTILS_H

#include <string>
#include <vector>

#include "crl-basic/utils/mathDefs.h"
#include "imgui_widgets/implot.h"

namespace crl::gui {

template <typename T>
struct Line2D {
    std::string label;
    /*
     * getter function for data field. getter function should return float.
     * e.g. [](const P3D &d) { return (float)d.y; }
     */
    std::function<float(T)> getter;
    ImPlotMarker marker = ImPlotMarker_None;
};

/**
 * A class for 2D Real-time plots.
 */
template <typename T>
class RealTimeLinePlot2D {
protected:
    // gui
    bool fitAxisX = true;
    bool fitAxisY = true;
    float scaleAxisX = 1;
    int plotHeight = 300;
    // vector of pair (x, T y)
    std::vector<std::pair<float, T>> data;
    std::string title;
    std::string xlabel;
    std::string ylabel;
    // vector of line spec
    std::vector<Line2D<T>> lineSpecs;

    // internal variable
    int DRAWINDEX_ = 0;
    int maxSize = 1000;
    int offset = 0;

public:
    explicit RealTimeLinePlot2D(const std::string &title, const std::string &xlabel, const std::string &ylabel, int maxSize = 1000, int plotHeight = 300)
        : title(title), xlabel(xlabel), ylabel(ylabel), maxSize(maxSize), plotHeight(plotHeight) {
        if (maxSize > 2000)  // for saving memory...
            maxSize = 2000;
        data.reserve(maxSize);
    }

    virtual ~RealTimeLinePlot2D() = default;

    const std::vector<std::pair<float, T>>& read_data() const{
        return data;
    }

    const std::vector<Line2D<T>>& read_linespecs() const{
        return lineSpecs;
    }

    const std::string & read_title() const{
        return title;
    }

    void clearData() {
        data.clear();
        offset = 0;
    }

    void addData(float x, const T &y) {
        if (data.size() < maxSize) {
            data.push_back({x, y});
        } else {
            data[offset] = {x, y};
            offset = (offset + 1) % maxSize;
        }
    }

    void addLineSpec(const Line2D<T> &lineSpec) {
        lineSpecs.push_back(lineSpec);
    }

    void draw() {
        ImGui::Checkbox(std::string("Always Fit X Axes##" + title).c_str(), &fitAxisX);
        ImGui::SameLine();
        ImGui::Checkbox(std::string("Always Fit Y Axes##" + title).c_str(), &fitAxisY);
        // use slider for fit axis x instead of ImPlot API.
        if (fitAxisY) {
            ImPlot::SetNextAxisToFit(ImAxis_Y1);
        }
        if (fitAxisX) {
            ImGui::SliderFloat(std::string("X Axis Scale##" + title).c_str(), &scaleAxisX, 0.1, 1);
            ImPlot::SetNextAxisLimits(ImAxis_X1, getXBegin() * scaleAxisX + getXEnd() * (1.f - scaleAxisX), getXEnd(), ImGuiCond_Always);
        }

        // draw plot
        if (ImPlot::BeginPlot(title.c_str(), ImVec2(-1, plotHeight))) {
            ImPlot::SetupAxis(ImAxis_X1, xlabel.c_str());
            ImPlot::SetupAxis(ImAxis_Y1, ylabel.c_str());
            for (DRAWINDEX_ = 0; DRAWINDEX_ < lineSpecs.size(); DRAWINDEX_++) {
                ImPlot::SetNextMarkerStyle(lineSpecs[DRAWINDEX_].marker);
                ImPlot::PlotLineG(
                    lineSpecs[DRAWINDEX_].label.c_str(),
                    [](void *data, int idx) {
                        auto *my_data = (RealTimeLinePlot2D<T> *)data;
                        auto &datapoint = my_data->getData(idx);
                        ImPlotPoint p;
                        p.x = (float)datapoint.first;
                        p.y = my_data->lineSpecs[my_data->DRAWINDEX_].getter(datapoint.second);
                        return p;
                    },
                    this, getSize());
            }
            ImPlot::EndPlot();
        }

        ImGui::Separator();
    }

private:
    int getSize() const {
        if (data.size() < maxSize)
            return data.size();
        return maxSize;
    }

    int getOffset() const {
        return offset;
    }

    const std::pair<float, T> &getData(int idx) {
        if (data.size() < maxSize)
            return data[idx];
        return data[(offset + idx) % maxSize];
    }

    float getXBegin() const {
        if (data.size() == 0)
            return -1;
        if (data.size() < maxSize)
            return data[0].first;
        return data[offset].first;
    }

    float getXEnd() const {
        if (data.size() == 0)
            return 1;
        if (data.size() < maxSize)
            return data.back().first;
        return data[offset - 1].first;
    }
};

}  // namespace crl::gui

#endif  //CRL_BASIC_PLOTUTILS_H