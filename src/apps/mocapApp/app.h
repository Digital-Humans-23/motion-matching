#pragma once

#include <crl-basic/gui/application.h>
#include <imgui_widgets/imfilebrowser.h>

#include "mocap/MocapClip.h"
#include "mocap/MocapClipUtils.h"
#include "mocap/MocapMarkers.h"
#include "mocap/MocapSkeletonState.h"
#include "mocap/PlotUtils.h"
#include "mocap/TimelineUtils.h"

namespace mocapApp {

class App : public crl::gui::ShadowApplication {
public:
    App()
        : crl::gui::ShadowApplication("Mocap App"),
          fileDialog(ImGuiFileBrowserFlags_SelectDirectory),
          baseHeightPlot("Root Height", "[sec]", "[m]"),
          baseSpeedPlot("Root Speed", "[sec]", "[m/s or rad/s]"),
          feetHeightPlot("Feet Height", "[sec]", "[m]"),
          feetVelocityPlot("Feet Velocity", "[sec]", "[m/s]"),
          feetAccelerationPlot("Feet Acceleration", "[sec]", "[m/s/s]"),
          swingTimeline(footSteps) {
        fileDialog.SetPwd(fs::path(CRL_MOCAP_DATA_FOLDER));
        fileDialog.SetTitle("Mocap Directory");
    }

    ~App() override {}

    void process() override {
        if (selectedBvhClipIdx == -1 && selectedC3dClipIdx == -1)
            return;

        if (selectedBvhClipIdx > -1) {
            auto &clip = bvhClips[selectedBvhClipIdx];
            if (auto *skel = clip->getModel()) {
                auto state = clip->getState(frameIdx);
                skel->setState(&state);
                if (followCharacter) {
                    camera.target.x = (float)clip->getModel()->root->state.pos.x;
                    camera.target.z = (float)clip->getModel()->root->state.pos.z;
                }
                light.target.x() = (float)clip->getModel()->root->state.pos.x;
                light.target.z() = (float)clip->getModel()->root->state.pos.z;
            }
            if (++frameIdx >= clip->getFrameCount())
                frameIdx = 0;
        }

        if (selectedC3dClipIdx > -1) {
            auto &clip = c3dClips[selectedC3dClipIdx];
            if (auto *model = clip->getModel()) {
                auto state = clip->getState(frameIdx);
                model->setState(&state);
                camera.target.x = (float)clip->getModel()->getMarkerByName("S_1")->state.pos.x;
                camera.target.z = (float)clip->getModel()->getMarkerByName("S_1")->state.pos.z;
            }
            if (++frameIdx >= clip->getFrameCount())
                frameIdx = 0;
        }
    }

    void restart() override {
        frameIdx = 0;
    }

    void prepareToDraw() override {
        if (selectedBvhClipIdx > -1) {
            bvhClips[selectedBvhClipIdx]->getModel()->showCoordFrame = showCoordinateFrames;
        }
        if (selectedC3dClipIdx > -1) {
            c3dClips[selectedC3dClipIdx]->getModel()->showCoordFrame = showCoordinateFrames;
        }
    }

    void drawShadowCastingObjects(const crl::gui::Shader &shader) override {
        if (selectedBvhClipIdx > -1) {
            bvhClips[selectedBvhClipIdx]->draw(shader, frameIdx);
        }
        if (selectedC3dClipIdx > -1) {
            c3dClips[selectedC3dClipIdx]->draw(shader, frameIdx);
            auto state = c3dClips[selectedC3dClipIdx]->getState(frameIdx);
            c3dClips[selectedC3dClipIdx]->getModel()->setState(&state);
            for (const auto &linkName : linkNames) {
                const auto *startMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.first.c_str());
                const auto *endMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.second.c_str());
                if (startMarker && endMarker)
                    crl::gui::drawCapsule(startMarker->state.pos, endMarker->state.pos, 0.01, shader, crl::V3D(0.5, 0.5, 0.5));
            }
        }
    }

    void drawObjectsWithoutShadows(const crl::gui::Shader &shader) override {
        if (selectedBvhClipIdx > -1)
            bvhClips[selectedBvhClipIdx]->draw(shader, frameIdx);
        if (selectedC3dClipIdx > -1) {
            c3dClips[selectedC3dClipIdx]->draw(shader, frameIdx);

            // draw skeleton
            if (selectedC3dClipIdx > -1) {
                c3dClips[selectedC3dClipIdx]->draw(shader, frameIdx);
                auto state = c3dClips[selectedC3dClipIdx]->getState(frameIdx);
                c3dClips[selectedC3dClipIdx]->getModel()->setState(&state);
                for (const auto &linkName : linkNames) {
                    const auto *startMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.first.c_str());
                    const auto *endMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.second.c_str());
                    if (startMarker && endMarker)
                        crl::gui::drawCapsule(startMarker->state.pos, endMarker->state.pos, 0.01, shader, crl::V3D(0.5, 0.5, 0.5));
                }

                if (showVirtualLimbs)
                    for (const auto &linkName : virtualLinkNames) {
                        const auto *startMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.first.c_str());
                        const auto *endMarker = c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(linkName.second.c_str());
                        if (startMarker && endMarker)
                            crl::gui::drawCapsule(startMarker->state.pos, endMarker->state.pos, 0.01, shader, crl::V3D(0.5, 0.5, 0.5), 0.2);
                    }
            }

            // draw foot contact
            if (showContactState)
                for (const auto &footMarkerName : footMarkerNames) {
                    bool isSwing = false;
                    for (const auto &swing : footSteps[footMarkerName]) {
                        if (swing.first <= frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() &&
                            frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() <= swing.second)
                            isSwing = true;
                    }
                    if (isSwing)
                        crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
                                             0.05, shader, crl::V3D(1, 0, 1), 0.5);
                    else
                        crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
                                             0.05, shader, crl::V3D(0, 1, 0), 0.5);
                }
            for (const auto &footMarkerName : footMarkerNames) {
                bool isSwing = false;
                for (const auto &swing : footSteps[footMarkerName]) {
                    if (swing.first <= frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() &&
                        frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep() <= swing.second)
                        isSwing = true;
                }
                if (isSwing)
                    crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
                                         0.05, shader, crl::V3D(1, 0, 1), 0.5);
                else
                    crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarkerByName(footMarkerName.c_str())->state.pos,  //
                                         0.05, shader, crl::V3D(0, 1, 0), 0.5);
            }

            // selected marker (for debugging)
            if (selectedMarkerIdx > -1)
                crl::gui::drawSphere(c3dClips[selectedC3dClipIdx]->getModel()->getMarker(selectedMarkerIdx)->state.pos, 0.05, shader, crl::V3D(1, 0, 0), 0.5);
        }
    }

    void drawPlots() {
        if (selectedBvhClipIdx == -1 && selectedC3dClipIdx == -1)
            return;
        double t = 0;
        if (selectedBvhClipIdx > -1)
            t = frameIdx * bvhClips[selectedBvhClipIdx]->getFrameTimeStep();
        if (selectedC3dClipIdx > -1)
            t = frameIdx * c3dClips[selectedC3dClipIdx]->getFrameTimeStep();

        ImGui::SetNextWindowPos(ImVec2(this->width - 720, 20), ImGuiCond_Once);
        ImGui::SetNextWindowSize(ImVec2(720, this->height - 20 - this->consoleHeight), ImGuiCond_Once);
        ImGui::Begin("Plot");
        ImGui::Text("Frame time = %lf", t);
        baseHeightPlot.draw(t);
        baseSpeedPlot.draw(t);
        feetHeightPlot.draw(t);
        feetVelocityPlot.draw(t);
        feetAccelerationPlot.draw(t);
        ImGui::End();
        swingTimeline.draw(t);
    }

    void drawImGui() override {
        crl::gui::ShadowApplication::drawImGui();

        ImGui::Begin("Main Menu");
        ImGui::Checkbox("Follow Character", &followCharacter);
        if (ImGui::CollapsingHeader("Mocap Data", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Import")) {
                crl::Logger::consolePrint("Loading mocap data...\n");
                fileDialog.Open();
            }
            fileDialog.Display();
            if (fileDialog.HasSelected()) {
                loadMocapFile(fileDialog.GetSelected());
                fileDialog.ClearSelected();
            }
            ImGui::SameLine();
            if (ImGui::Button("Clear")) {
                crl::Logger::consolePrint("Clear loaded mocap data...\n");
                bvhClips.clear();
                c3dClips.clear();
                selectedBvhClipIdx = -1;
                selectedC3dClipIdx = -1;
            }
            if (ImGui::ListBoxHeader("BVH\nMocap Clips##MocapClips", 10)) {
                for (int i = 0; i < bvhClips.size(); i++) {
                    bool isSelected = (selectedBvhClipIdx == (int)i);
                    if (ImGui::Selectable(bvhClips[i]->getName().c_str(), isSelected)) {
                        selectedBvhClipIdx = i;
                        selectedC3dClipIdx = -1;
                        selectedMarkerIdx = -1;
                        frameIdx = 0;
                        processBVHClip();
                    }
                }
                ImGui::ListBoxFooter();
            }
            if (ImGui::ListBoxHeader("C3D\nMocap Clips##MocapClips", 10)) {
                for (int i = 0; i < c3dClips.size(); i++) {
                    bool isSelected = (selectedC3dClipIdx == (int)i);
                    if (ImGui::Selectable(c3dClips[i]->getName().c_str(), isSelected)) {
                        selectedBvhClipIdx = -1;
                        selectedC3dClipIdx = i;
                        selectedMarkerIdx = -1;
                        frameIdx = 0;
                        processC3DClip();
                    }
                }
                ImGui::ListBoxFooter();
            }
            if (selectedC3dClipIdx > -1) {
                if (ImGui::ListBoxHeader("Mocap\nMarkers##MocapClips", 10)) {
                    for (int i = 0; i < c3dClips[selectedC3dClipIdx]->getModel()->getMarkerCount(); i++) {
                        bool isSelected = (selectedMarkerIdx == (int)i);
                        if (ImGui::Selectable(c3dClips[selectedC3dClipIdx]->getModel()->getMarker(i)->name.c_str(), isSelected)) {
                            selectedMarkerIdx = i;
                        }
                    }
                    ImGui::ListBoxFooter();
                }
            }
        }

        if (ImGui::CollapsingHeader("Post Processing", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::SliderDouble("Foot Height Threshold", &footHeightThreshold, 0.0, 0.2, "%.2f")) {
                processBVHClip();
                processC3DClip();
            }
            if (ImGui::SliderDouble("Foot Velocity Threshold", &footVelocityThreshold, 0.0, 2.0, "%.2f")) {
                processBVHClip();
                processC3DClip();
            }
        }

        if (ImGui::CollapsingHeader("Draw")) {
            ImGui::Checkbox("Show Coordinate Frames", &showCoordinateFrames);
            ImGui::Checkbox("Show Virtual Limb", &showVirtualLimbs);
            ImGui::Checkbox("Show Contact State", &showContactState);
        }

        ImGui::End();
        drawPlots();
    }

    bool mouseButtonPressed(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            crl::P3D rayOrigin;
            crl::V3D rayDirection;
            crl::Ray mouseRay(rayOrigin, rayDirection);
            camera.getRayFromScreenCoordinates(mouseState.lastMouseX, mouseState.lastMouseY, rayOrigin, rayDirection);
            if (selectedC3dClipIdx > -1) {
                crl::P3D selectedPoint;
                crl::mocap::MocapMarker *marker = c3dClips[selectedC3dClipIdx]->getModel()->getFirstJointHitByRay(mouseRay, selectedPoint);
                if (marker)
                    crl::Logger::consolePrint("Selected marker: %s\n", marker->name.c_str());
            }
        }
        return true;
    }

    bool drop(int count, const char **fileNames) override {
        for (uint i = 0; i < count; i++)
            loadMocapFile(fs::path(fileNames[i]));
        return true;
    }

    void loadMocapFile(const fs::path &path) {
        int cnt = 0;
        if (is_directory(path)) {
            for (const auto &entry : fs::directory_iterator(path)) {
                if (loadSingleMocapFile(entry.path()))
                    cnt++;
            }
        } else {
            if (loadSingleMocapFile(path))
                cnt++;
        }

        // sort by name
        std::sort(bvhClips.begin(),
                  bvhClips.end(),  //
                  [](const auto &a, const auto &b) { return a->getName() < b->getName(); });
        std::sort(c3dClips.begin(),
                  c3dClips.end(),  //
                  [](const auto &a, const auto &b) { return a->getName() < b->getName(); });

        // reset indices
        selectedBvhClipIdx = -1;
        selectedC3dClipIdx = -1;
        frameIdx = 0;

        crl::Logger::consolePrint("Imported %d clips.\n", cnt);
    }

private:
    bool loadSingleMocapFile(const fs::path &path) {
        try {
            if (path.extension() == ".bvh")
                bvhClips.push_back(std::make_unique<crl::mocap::BVHClip>(path));
            else if (path.extension() == ".c3d")
                c3dClips.push_back(std::make_unique<crl::mocap::C3DClip>(path));
        } catch (...) {
            crl::Logger::consolePrint("Failed to load %s file.\n", path.c_str());
            return false;
        }
        return true;
    }

    void processBVHClip() {
        if (selectedBvhClipIdx == -1)
            return;

        footSteps.clear();
        // footMarkerNames = {"LeftHand", "LeftFoot", "RightHand", "RightFoot"};
        footMarkerNames = {"LeftHand", "LeftToe", "RightHand", "RightToe"};
        linkNames = {};
        virtualLinkNames = {};

        // initialize plots
        baseHeightPlot.clearAll();
        baseSpeedPlot.clearAll();
        feetHeightPlot.clearAll();
        feetVelocityPlot.clearAll();
        feetAccelerationPlot.clearAll();
        baseHeightPlot.addLineSpec({"h", [](const auto &d) { return (float)d.y; }});
        baseSpeedPlot.addLineSpec({"forward", [](const auto &d) { return (float)d[0]; }});
        baseSpeedPlot.addLineSpec({"sideways", [](const auto &d) { return (float)d[1]; }});
        baseSpeedPlot.addLineSpec({"turning", [](const auto &d) { return (float)d[2]; }});

        baseHeightPlot.drawVerticalGuide = true;
        baseHeightPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        baseSpeedPlot.drawVerticalGuide = true;
        baseSpeedPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        feetHeightPlot.drawVerticalGuide = true;
        feetHeightPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        feetVelocityPlot.drawVerticalGuide = true;
        feetVelocityPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        feetAccelerationPlot.drawVerticalGuide = true;
        feetAccelerationPlot.setMaxSize(bvhClips[selectedBvhClipIdx]->getFrameCount());
        for (uint i = 0; i < footMarkerNames.size(); i++) {
            feetHeightPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
            feetVelocityPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
            feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " x", [i](const auto &d) { return (float)d[3 * i]; }});
            feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " y", [i](const auto &d) { return (float)d[3 * i + 1]; }});
            feetAccelerationPlot.addLineSpec({footMarkerNames[i] + " z", [i](const auto &d) { return (float)d[3 * i + 2]; }});
        }

        // extracting foot velocity and position
        auto *sk = bvhClips[selectedBvhClipIdx]->getModel();
        double t = 0;
        const auto &speed = crl::mocap::extractRootSpeedProfileFromBVHClip(bvhClips[selectedBvhClipIdx].get());

        std::vector<crl::V3D> footVelocityBackup;
        for (int i = 0; i < footMarkerNames.size(); i++)
            footVelocityBackup.push_back(crl::V3D());

        for (int i = 0; i < bvhClips[selectedBvhClipIdx]->getFrameCount(); i++) {
            sk->setState(&bvhClips[selectedBvhClipIdx]->getState(i));

            baseHeightPlot.addData((float)t, sk->root->state.pos);
            baseSpeedPlot.addData((float)t, speed.evaluate_linear(t));
            crl::dVector footHeights(footMarkerNames.size());
            crl::dVector footVelocities(footMarkerNames.size());
            crl::dVector footAcceleration(footMarkerNames.size() * 3);
            for (int j = 0; j < footMarkerNames.size(); j++) {
                const auto &name = footMarkerNames[j];
                if (const auto joint = sk->getMarkerByName(name.c_str())) {
                    crl::P3D eepos = joint->state.getWorldCoordinates(joint->endSites[0].endSiteOffset);
                    crl::V3D eevel = joint->state.getVelocityForPoint_local(joint->endSites[0].endSiteOffset);
                    footHeights[j] = eepos.y;
                    footVelocities[j] = eevel.norm();

                    if (i == 0) {
                        footAcceleration[3 * j] = 0;
                        footAcceleration[3 * j + 1] = 0;
                        footAcceleration[3 * j + 2] = 0;
                    } else {
                        crl::V3D acc = (eevel - footVelocityBackup[j]) / bvhClips[selectedBvhClipIdx]->getFrameTimeStep();
                        footAcceleration[3 * j] = acc.x();
                        footAcceleration[3 * j + 1] = acc.y();
                        footAcceleration[3 * j + 2] = acc.z();
                    }

                    footVelocityBackup[j] = eevel;
                }
            }

            feetHeightPlot.addData((float)t, footHeights);
            feetVelocityPlot.addData((float)t, footVelocities);
            feetAccelerationPlot.addData((float)t, footAcceleration);
            t += bvhClips[selectedBvhClipIdx]->getFrameTimeStep();
        }

        // extracting swing sequences
        for (const auto &name : footMarkerNames) {
            auto contactInfos = crl::mocap::extractFootContactStateFromBVHClip(  //
                bvhClips[selectedBvhClipIdx].get(), name, footHeightThreshold, footVelocityThreshold);
            footSteps[name] = crl::mocap::convertFootSwingSequenceFromFootContactStates(contactInfos);
        }
    }

    void processC3DClip() {
        if (selectedC3dClipIdx == -1)
            return;

        footSteps.clear();
        std::unique_ptr<crl::mocap::C3DClip> &clip = c3dClips[selectedC3dClipIdx];
        const std::vector<double> gaussianKernel = {0.031091376664711, 0.041904779550059, 0.053944239501404, 0.066326105267268, 0.077890021426103,
                                                    0.087364914695958, 0.093594471765148, 0.095768182258699, 0.093594471765148, 0.087364914695958,
                                                    0.077890021426103, 0.066326105267268, 0.053944239501404, 0.041904779550059, 0.031091376664711};

        const std::string rootName = "S_1";
        footMarkerNames = {"lf_fetlock", "lh_fetlock", "rf_fetlock", "rh_fetlock"};
        linkNames = {{"S_1", "Th_6"},
                     {"S_1", "S_6"},
                     {"S_1", "r_sacrum"},
                     {"S_1", "l_sacrum"},
                     {"l_sacrum", "l_hip"},
                     {"l_hip", "l_knee"},
                     {"l_knee", "l_tarsus"},
                     {"l_tarsus", "lh_fetlock"},
                     {"Th_6", "l_spina_ud"},
                     {"l_spina_ud", "l_spina_d"},
                     {"l_spina_d", "l_shoulder"},
                     {"l_shoulder", "l_elbow"},
                     {"l_elbow", "l_carpus"},
                     {"l_carpus", "lf_fetlock"}};
        virtualLinkNames = {{"Th_6", "rf_fetlock"}, {"r_sacrum", "rh_fetlock"}};

        double treadmill = 0;
        if (clip->getName().rfind("wa") != std::string::npos)
            treadmill = 1.1;
        if (clip->getName().rfind("tr") != std::string::npos)
            treadmill = 2.1;

        // initialize plots
        baseHeightPlot.clearAll();
        baseSpeedPlot.clearAll();
        feetHeightPlot.clearAll();
        feetVelocityPlot.clearAll();
        baseHeightPlot.addLineSpec({"h", [](const auto &d) { return (float)d.y; }});
        baseSpeedPlot.addLineSpec({"forward", [](const auto &d) { return (float)d[0]; }});
        baseSpeedPlot.addLineSpec({"sideways", [](const auto &d) { return (float)d[1]; }});
        baseSpeedPlot.addLineSpec({"turning", [](const auto &d) { return (float)d[2]; }});
        baseSpeedPlot.addLineSpec({"forward post", [](const auto &d) { return (float)d[3]; }});
        baseSpeedPlot.addLineSpec({"sideways post", [](const auto &d) { return (float)d[4]; }});
        baseSpeedPlot.addLineSpec({"turning post", [](const auto &d) { return (float)d[5]; }});

        baseHeightPlot.drawVerticalGuide = true;
        baseHeightPlot.setMaxSize(clip->getFrameCount());
        baseSpeedPlot.drawVerticalGuide = true;
        baseSpeedPlot.setMaxSize(clip->getFrameCount());
        feetHeightPlot.drawVerticalGuide = true;
        feetHeightPlot.setMaxSize(clip->getFrameCount());
        feetVelocityPlot.drawVerticalGuide = true;
        feetVelocityPlot.setMaxSize(clip->getFrameCount());
        for (uint i = 0; i < footMarkerNames.size(); i++) {
            feetHeightPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i]; }});
            feetVelocityPlot.addLineSpec({footMarkerNames[i], [i](const auto &d) { return (float)d[i * 2]; }});
            feetVelocityPlot.addLineSpec({footMarkerNames[i] + " post", [i](const auto &d) { return (float)d[i * 2 + 1]; }});
        }

        auto *model = clip->getModel();
        double t = 0;

        // populate data
        std::map<std::string, std::vector<crl::V3D>> footVelocityMap;      // from data
        std::map<std::string, std::vector<crl::V3D>> footVelocityPostMap;  // after post-processing
        for (int i = 0; i < clip->getFrameCount(); i++) {
            model->setState(&clip->getState(i));

            // TODO: consider heading...
            crl::V3D rootVelocity = model->getMarkerByName(rootName.c_str())->state.velocity;
            crl::dVector velocityData(6);
            velocityData << rootVelocity.x(), rootVelocity.z(), 0, rootVelocity.x() + treadmill, rootVelocity.z(), 0;
            baseHeightPlot.addData((float)t, model->getMarkerByName(rootName.c_str())->state.pos);
            baseSpeedPlot.addData((float)t, velocityData);

            crl::dVector footHeights(footMarkerNames.size());
            for (int j = 0; j < footMarkerNames.size(); j++) {
                const auto &name = footMarkerNames[j];
                if (const auto joint = model->getMarkerByName(name.c_str())) {
                    crl::P3D eepos = joint->state.pos;
                    crl::V3D eevel = joint->state.velocity;
                    footHeights[j] = eepos.y;
                    footVelocityMap[name].push_back(eevel);
                }
            }

            feetHeightPlot.addData((float)t, footHeights);
            t += clip->getFrameTimeStep();
        }

        // filtering for denoise
        for (int i = 0; i < clip->getFrameCount(); i++) {
            for (const auto &footMarkerName : footMarkerNames) {
                crl::V3D ret(0, 0, 0);
                for (int offset = -7; offset <= 7; ++offset) {
                    int idx = i + offset;
                    if (idx >= 0 && idx < (int)clip->getFrameCount()) {
                        // Add the forward speed values multiplied by the corresponding kernel coefficient otherwise "add a zero" (i.e. assume zero padding)
                        ret += gaussianKernel[offset + 7] * footVelocityMap[footMarkerName][idx];
                    }
                }
                footVelocityPostMap[footMarkerName].push_back(ret + crl::V3D(treadmill, 0, 0));
            }
        }

        // populate foot velocity plot data (after filtering)
        t = 0;
        for (int i = 0; i < clip->getFrameCount(); i++) {
            crl::dVector footHeights(footMarkerNames.size());
            crl::dVector footVelocities(footMarkerNames.size() * 2);
            for (int j = 0; j < footMarkerNames.size(); j++) {
                const auto &name = footMarkerNames[j];
                crl::V3D eevel = footVelocityMap[name][i];
                crl::V3D eevelPost = footVelocityPostMap[name][i];
                footVelocities[j * 2] = eevel.norm();
                footVelocities[j * 2 + 1] = eevelPost.norm();
            }

            feetVelocityPlot.addData((float)t, footVelocities);
            t += clip->getFrameTimeStep();
        }

        // populate footstep
        for (const auto &footMarkerName : footMarkerNames) {
            std::vector<std::pair<double, bool>> contactYN;
            contactYN.reserve(clip->getFrameCount());
            auto *sk = clip->getModel();

            for (int i = 0; i < clip->getFrameCount(); i++) {
                sk->setState(&clip->getState(i));
                if (const auto marker = sk->getMarkerByName(footMarkerName.c_str())) {
                    crl::P3D eepos = marker->state.pos;
                    crl::V3D eevel = footVelocityPostMap[footMarkerName][i];
                    eevel.y() = 0;

                    if (eepos.y < footHeightThreshold && eevel.norm() < footVelocityThreshold)
                        contactYN.emplace_back(clip->getFrameTimeStep() * i, true);  // contact
                    else
                        contactYN.emplace_back(clip->getFrameTimeStep() * i, false);  // swing
                }
            }

            footSteps[footMarkerName] = crl::mocap::convertFootSwingSequenceFromFootContactStates(contactYN);
        }
    }

private:
    ImGui::FileBrowser fileDialog;
    std::vector<std::unique_ptr<crl::mocap::BVHClip>> bvhClips;
    std::vector<std::unique_ptr<crl::mocap::C3DClip>> c3dClips;
    int selectedBvhClipIdx = -1;
    int selectedC3dClipIdx = -1;
    int selectedMarkerIdx = -1;
    int frameIdx = 0;

    // post processing
    std::vector<std::string> footMarkerNames;
    std::vector<std::pair<std::string, std::string>> linkNames;
    std::vector<std::pair<std::string, std::string>> virtualLinkNames;
    crl::mocap::Timeline::TimelineData footSteps;
    double footVelocityThreshold = 0.8;
    double footHeightThreshold = 0.055;

    // plot and visualization
    bool followCharacter = true;
    bool showCoordinateFrames = false;
    bool showVirtualLimbs = true;
    bool showContactState = false;
    crl::mocap::PlotLine2D<crl::P3D> baseHeightPlot;
    crl::mocap::PlotLine2D<crl::dVector> baseSpeedPlot;
    crl::mocap::PlotLine2D<crl::dVector> feetHeightPlot;
    crl::mocap::PlotLine2D<crl::dVector> feetVelocityPlot;
    crl::mocap::PlotLine2D<crl::dVector> feetAccelerationPlot;
    crl::mocap::Timeline swingTimeline;
};

}  // namespace mocapApp