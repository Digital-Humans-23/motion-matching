//
// Created by Dongho Kang on 12.12.21.
//

#ifndef CRL_MOCAP_PLOTUTILS_H
#define CRL_MOCAP_PLOTUTILS_H

#include <string>
#include <vector>

#include "crl-basic/utils/mathDefs.h"
#include "imgui_widgets/implot.h"

namespace crl::mocap {

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

template <typename T>
class PlotLine2D {
public:
    /* options */
    // draw vertical line at verticalX for guide (e.g. current time)
    bool drawVerticalGuide = false;

private:
    int maxSize = 1000;
    int offset = 0;
    bool fitAxisX = true;
    bool fitAxisY = true;
    // vector of pair (x, T y)
    std::vector<std::pair<float, T>> data;
    std::string title;
    std::string xlabel;
    std::string ylabel;

    // vector of line spec
    std::vector<Line2D<T>> lineSpecs;
    int DRAWINDEX_ = 0;

public:
    explicit PlotLine2D(const std::string &title, const std::string &xlabel, const std::string &ylabel, int maxSize = 1000)
        : title(title), xlabel(xlabel), ylabel(ylabel), maxSize(maxSize) {
        if (maxSize > 2000)  // for saving memory...
            maxSize = 2000;
        data.reserve(maxSize);
    }

    void setMaxSize(int maxSize) {
        clearData();
        this->maxSize = maxSize;
        data.reserve(maxSize);
    }

    void clearAll() {
        clearData();
        clearLineSpec();
    }

    /**
     * clear data. leave line specs
     */
    void clearData() {
        data.clear();
        offset = 0;
    }

    /**
     * clear line specs.
     */
    void clearLineSpec() {
        lineSpecs.clear();
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

    void draw(double verticalGuide = 0) {
        ImGui::Checkbox(std::string("Always Fit X Axes##" + title).c_str(), &fitAxisX);
        ImGui::SameLine();
        ImGui::Checkbox(std::string("Always Fit Y Axes##" + title).c_str(), &fitAxisY);
        // use slider for fit axis x instead of ImPlot API.
        if (fitAxisY) {
            ImPlot::SetNextAxisToFit(ImAxis_Y1);
        }
        if (fitAxisX) {
            ImPlot::SetNextAxisToFit(ImAxis_X1);
        }

        if (ImPlot::BeginPlot(title.c_str(), ImVec2(-1, 300))) {
            ImPlot::SetupAxis(ImAxis_X1, xlabel.c_str());
            ImPlot::SetupAxis(ImAxis_Y1, ylabel.c_str());
            for (DRAWINDEX_ = 0; DRAWINDEX_ < lineSpecs.size(); DRAWINDEX_++) {
                ImPlot::SetNextMarkerStyle(lineSpecs[DRAWINDEX_].marker);
                ImPlot::PlotLineG(
                    lineSpecs[DRAWINDEX_].label.c_str(),
                    [](void *data, int idx) {
                        auto *my_data = (PlotLine2D<T> *)data;
                        auto &datapoint = my_data->getData(idx);
                        ImPlotPoint p;
                        p.x = (float)datapoint.first;
                        p.y = my_data->lineSpecs[my_data->DRAWINDEX_].getter(datapoint.second);
                        return p;
                    },
                    this, getSize());
            }
            if (drawVerticalGuide)
                ImPlot::DragLineX(0, &verticalGuide, ImVec4(1,1,1,1));
            ImPlot::EndPlot();
        }
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
};

}  // namespace crl::mocap

#endif  //CRL_MOCAP_PLOTUTILS_H
