#include "crl-basic/gui/application.h"

#include <iomanip>

#include "crl-basic/gui/glUtils.h"
#include "crl-basic/utils/json_helpers.h"
#include "crl-basic/utils/logger.h"
#include "crl-basic/utils/timer.h"
#include "crl-basic/utils/utils.h"

// defined in model.cpp
//#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION

#include <stb_image_write.h>

#include "crl-basic/gui/renderer.h"

namespace crl {
namespace gui {

Application::Application(const char *title, int width, int height, std::string iconPath) : width(width), height(height) {
    if (!glfwInit()) {
        // An error occured
        std::cout << "GLFW initialization failed\n";
        exit(0);
    }

    init(title, width, height, iconPath);
}

Application::Application(const char *title, std::string iconPath) {
    if (!glfwInit()) {
        // An error occured
        std::cout << "GLFW initialization failed\n";
        exit(0);
    }

    const GLFWvidmode *mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

#ifdef __APPLE__
    int borderLeft = 0;
    int borderTop = 42;
    int borderRight = 0;
    int borderBottom = 60;
#else
    int borderLeft = 2;
    int borderTop = 70;
    int borderRight = 2;
    int borderBottom = 105;
#endif

    init(title, (mode->width - borderLeft - borderRight), (mode->height - borderTop - borderBottom), iconPath);
    glfwSetWindowPos(window, borderLeft, borderTop);
    glfwMaximizeWindow(window);
}

Application::~Application() {
    if (useSeparateProcessThread)
        processThread.std::thread::~thread();
}

void Application::init(const char *title, int width, int height, std::string iconPath) {
    // glfw: initialize and configure

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 8);

#ifdef SINGLE_BUFFER
    glfwWindowHint(GLFW_DOUBLEBUFFER, GL_FALSE);  // turn off framerate limit
#endif

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // fix compilation on OS X
#endif

    // units of width and height are coordinate unit not pixel.
    // for retina, this is x0.5 to pixel size.
    this->width = width;
    this->height = height;

    // glfw window creation
    window = glfwCreateWindow(this->width, this->height, title, nullptr, nullptr);
    if (window == nullptr) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    glfwMakeContextCurrent(window);

    // app icon
    if (iconPath != "") {
        GLFWimage image;
        image.pixels = stbi_load(iconPath.c_str(), &image.width, &image.height, nullptr, 4);
        glfwSetWindowIcon(window, 1, &image);
        stbi_image_free(image.pixels);
    }

    // glad: load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    GLCall(glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
    GLCall(glEnable(GL_BLEND));
    GLCall(glEnable(GL_MULTISAMPLE));

    // Setup Dear ImGui binding
    const char *glsl_version = "#version 150";
    IMGUI_CHECKVERSION();
    rendering::CreateContext();
    ImGui::CreateContext();
    ImPlot::CreateContext();

    // init
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    setCallbacks();

    // set UI scale
    rescaleUI();
}

void Application::setCallbacks() {
    glfwSetWindowUserPointer(window, this);

    glfwSetErrorCallback([](int error, const char *description) { std::cout << "Error " << error << ": " << description << std::endl; });

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow *window, int width, int height) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->resizeBuffer(width, height);

        // Framebuffer resize callback is called separately from the window size change callback.
        // We can ensure correct scaling by always rescaling the ui.
        app->rescaleUI();
    });

    glfwSetWindowContentScaleCallback(window, [](GLFWwindow *window, float xscale, float yscale) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->rescaleUI();
    });

    glfwSetWindowSizeCallback(window, [](GLFWwindow *window, int width, int height) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->resizeWindow(width, height);

        // Framebuffer resize callback is called separately from the window size change callback.
        // We can ensure correct scaling by always rescaling the ui.
        app->rescaleUI();
    });

    glfwSetKeyCallback(window, [](GLFWwindow *window, int key, int scancode, int action, int mods) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->keyboardState[key] = (action != GLFW_RELEASE);

        if (ImGui::GetIO().WantCaptureKeyboard || ImGui::GetIO().WantTextInput) {
            ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
            return;
        }

        if (key == GLFW_KEY_ESCAPE) {
            if (app->useSeparateProcessThread && app->processIsRunning) {
                app->processIsRunning = false;
                app->processThread.std::thread::~thread();
            }
            glfwSetWindowShouldClose(window, GL_TRUE);
            return;
        }

        if (key == GLFW_KEY_GRAVE_ACCENT && action == GLFW_PRESS) {
            app->showConsole = !app->showConsole;
            return;
        }

        if (action == GLFW_PRESS)
            app->keyPressed(key, mods);

        if (action == GLFW_RELEASE)
            app->keyReleased(key, mods);
    });

    glfwSetMouseButtonCallback(window, [](GLFWwindow *window, int button, int action, int mods) {
        double xPos, yPos;
        glfwGetCursorPos(window, &xPos, &yPos);

        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->mouseState.onMouseClick(xPos, yPos, button, action, mods);

        if (ImGui::GetIO().WantCaptureMouse) {
            ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
            return;
        }

        if (action == GLFW_PRESS)
            app->mouseButtonPressed(button, mods);

        if (action == GLFW_RELEASE)
            app->mouseButtonReleased(button, mods);
    });

    glfwSetCursorPosCallback(window, [](GLFWwindow *window, double xpos, double ypos) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->mouseState.onMouseMove(xpos, ypos);

        if (ImGui::GetIO().WantCaptureMouse)
            return;

        app->mouseMove(xpos, ypos);
    });

    glfwSetScrollCallback(window, [](GLFWwindow *window, double xoffset, double yoffset) {
        if (ImGui::GetIO().WantCaptureMouse) {
            ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
            return;
        }

        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->scrollWheel(xoffset, yoffset);
    });

    glfwSetDropCallback(window, [](GLFWwindow *window, int count, const char **filenames) {
        auto app = static_cast<Application *>(glfwGetWindowUserPointer(window));
        app->drop(count, filenames);
    });
}

void Application::run() {
    float tmpEntireLoopTimeRunningAverage = 0.0f;
    float tmpProcessTimeRunningAverage = 0.0f;
    int runningAverageStepCount = 0;

    Timer FPSDisplayTimer, processTimer, FPSTimer;
    glfwSwapInterval(0);  //Disable waiting for framerate of glfw window

    while (!glfwWindowShouldClose(window)) {
        if (FPSDisplayTimer.timeEllapsed() > 0.33) {
            FPSDisplayTimer.restart();
            if (runningAverageStepCount > 0) {
                averageFPS = 1.0 / (tmpEntireLoopTimeRunningAverage / runningAverageStepCount);
                averagePercentTimeSpentProcessing = tmpProcessTimeRunningAverage / tmpEntireLoopTimeRunningAverage;
            } else {
                averageFPS = -1;
                averagePercentTimeSpentProcessing = -1;
            }
            tmpEntireLoopTimeRunningAverage = 0;
            tmpProcessTimeRunningAverage = 0;
            runningAverageStepCount = 0;
        }
        runningAverageStepCount++;

        tmpEntireLoopTimeRunningAverage += FPSTimer.timeEllapsed();
        FPSTimer.restart();

        processTimer.restart();
        if (!useSeparateProcessThread && processIsRunning)
            process();
        tmpProcessTimeRunningAverage += processTimer.timeEllapsed();

        draw();

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse
        // moved etc.)
#ifdef SINGLE_BUFFER
        glFlush();
#else
        glfwSwapBuffers(window);
#endif
        glfwPollEvents();

        if (limitFramerate)
            while (FPSTimer.timeEllapsed() < (1.0 / (double)targetFramerate)) {
#ifndef WIN32
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(1ms);  //Sleep for a bit (too inaccurate to be used on windows)
#endif                                             // WIN32
            }

        if (screenIsRecording) {
            char filename[1000];
            sprintf(filename, "%s_%04d.png", screenshotPath.c_str(), screenShotCounter);
            screenshot(filename);
            screenShotCounter++;
        }
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    rendering::DestroyContext();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
}

void Application::baseProcess() {
    while (processIsRunning)
        process();
}

void Application::processCallback() {
    if (useSeparateProcessThread) {
        if (processIsRunning) {
            processThread = std::thread(&Application::baseProcess, this);
            processThread.detach();
            Logger::print(Logger::DEFAULT, "Process thread started...\n");
        } else {
            processThread.std::thread::~thread();
            Logger::print(Logger::DEFAULT, "Process thread terminated...\n");
        }
    }
}

void Application::draw() {
    GLCall(glClearColor(clearColor[0], clearColor[1], clearColor[2], 1.f));
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT));

    //ImGui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    drawFPS();
    drawConsole();
    drawImGui();
    if (showPlots)
        drawImPlot();

    ImGui::EndFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void Application::drawImGui() {
    ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
    ImGui::Begin("Main Menu");

    ImGui::Text("Play:");
    ImGui::SameLine();
    if (ImGui::ToggleButton("Play", &processIsRunning))
        processCallback();

    if (!processIsRunning) {
        ImGui::SameLine();
        if (ImGui::ArrowButton("tmp", ImGuiDir_Right))
            process();
    }

    ImGui::SameLine();
    if (ImGui::Button("Restart"))
        restart();

    if (ImGui::TreeNode("Console")) {
        ImGui::Checkbox("Draw Console", &showConsole);
        ImGui::Checkbox("Automanage Console", &automanageConsole);
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Plot")) {
        ImGui::Checkbox("Draw Plots", &showPlots);
        ImGui::TreePop();
    }
    ImGui::End();
}

void Application::drawImPlot() {
    ImGui::SetNextWindowPos(ImVec2(this->width, 20), ImGuiCond_Once, ImVec2(1.0, 0));
    ImGui::SetNextWindowSize(ImVec2(700, this->height - 20 - this->consoleHeight), ImGuiCond_Once);
    ImGui::Begin("Plots");
    ImGui::End();
}

void Application::drawFPS() {
    ImGui::SetNextWindowPos(ImVec2(width, 0), ImGuiCond_Always, ImVec2(1, 0));
    ImGui::SetNextWindowSize(ImVec2(pixelRatio * 320, pixelRatio * 80), ImGuiCond_Always);
    ImGui::SetNextWindowCollapsed(true, ImGuiCond_Once);
    char title[100];
    sprintf(title, "FPS: %.2f###FPS", averageFPS);
    ImGui::Begin(title);
    ImGui::Text("Time spent processing: %.2f%%", 100.0 * averagePercentTimeSpentProcessing);
    ImGui::Checkbox("Limit FPS", &limitFramerate);
    ImGui::SameLine(pixelRatio * 100);
    if (limitFramerate)
        ImGui::InputInt("###targetFrameRateIn", &targetFramerate);
    ImGui::End();
}

void Application::drawConsole() {
    if (showConsole == false)
        return;

    if (automanageConsole == false) {
        ImGui::SetNextWindowPos(ImVec2(0, this->height), ImGuiCond_Once, ImVec2(0, 1.0));
        ImGui::SetNextWindowSize(ImVec2(this->width, pixelRatio * 335), ImGuiCond_Once);
    }
    ImGui::Begin("Console");
    if (automanageConsole == true) {
        if (ImGui::IsWindowCollapsed()) {
            ImGui::SetWindowPos(ImVec2(this->width - pixelRatio * 300, this->height - pixelRatio * 20), ImGuiCond_Always);
            ImGui::SetWindowSize(ImVec2(pixelRatio * 300, pixelRatio * 80), ImGuiCond_Always);
        } else {
            ImGui::SetWindowPos(ImVec2(0, this->height - pixelRatio * consoleHeight), ImGuiCond_Always);
            ImGui::SetWindowSize(ImVec2(this->width, pixelRatio * consoleHeight), ImGuiCond_Always);
        }
    }

    for (int i = 0; i < (int)Logger::consoleOutput.size(); i++) {
        for (int j = 0; j < (int)Logger::consoleOutput[i].size(); j++) {
            const Logger::ConsoleText &cText = Logger::consoleOutput[i][j];
            ImVec4 color(cText.color.x(), cText.color.y(), cText.color.z(), 1.0);
            ImGui::TextColored(color, "%s", cText.text.c_str());
            ImGui::SameLine();
        }
        ImGui::NewLine();
    }
    ImGui::End();
}

void Application::resizeWindow(int width, int height) {
    this->width = width;
    this->height = height;
}

void Application::resizeBuffer(int width, int height) {
    GLCall(glViewport(0, 0, width, height));
}

void Application::rescaleUI() {
    // get window content scale factor
    float xscale = 1.0f;
    float yscale = 1.0f;
    glfwGetWindowContentScale(window, &xscale, &yscale);

    // get framebuffer size
    int bufferW = 1, bufferH = 1;
    glfwGetFramebufferSize(window, &bufferW, &bufferH);  // this is workaround for retina support

    // compute pixel scale based on window / framebuffer ratio as well as the content scale
    this->pixelRatio = xscale * this->width / float(bufferW);

    // pixelRatio <= 0.0 triggers an assertion in ImGui. Can happen if the window hasn't been initialized yet.
    if (this->pixelRatio <= 0.0f) {
        return;
    }

    // update UI
    ImGuiIO &io = ImGui::GetIO();
    ImFontConfig cfg;
    cfg.SizePixels = 40 * this->pixelRatio;
    io.Fonts->Clear();
    ImFont *imFont = io.Fonts->AddFontFromFileTTF(CRL_IMGUI_FONT_FOLDER "/Roboto-Medium.ttf", 15.0f * this->pixelRatio, &cfg);
    io.Fonts->Build();

    ImGui_ImplOpenGL3_DestroyFontsTexture();
    ImGui_ImplOpenGL3_CreateFontsTexture();

    ImGuiStyle &style = ImGui::GetStyle();
    // Repeated scaling will result in losses since the scaling value is rounded, therefore we create a new, fresh style here.
    style = ImGuiStyle();
    style.ScaleAllSizes(this->pixelRatio);
}

bool Application::keyPressed(int key, int mods) {
    if (key == GLFW_KEY_SPACE) {
        processIsRunning = !processIsRunning;
        processCallback();
    } else if (key == GLFW_KEY_RIGHT) {
        if (!processIsRunning)
            process();
    }
    return false;
}

bool Application::keyReleased(int key, int mods) {
    return false;
}

bool Application::mouseButtonPressed(int button, int mods) {
    return false;
}

bool Application::mouseButtonReleased(int button, int mods) {
    return false;
}

bool Application::mouseMove(double xpos, double ypos) {
    return false;
}

bool Application::scrollWheel(double xoffset, double yoffset) {
    return false;
}

bool Application::drop(int count, const char **filenames) {
    return false;
}

bool Application::screenshot(const char *filename) const {
    std::vector<unsigned char> pixels(width * height * 3);
    GLCall(glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, &pixels[0]));
    stbi_flip_vertically_on_write(1);
    return (bool)stbi_write_png(filename, width, height, 3, &pixels[0], 0);
}

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

ShadowApplication::ShadowApplication(const char *title, int width, int height, std::string iconPath) : Application(title, width, height, iconPath) {
    camera.distanceToTarget = 5.f;
    camera.aspectRatio = float(width) / height;

    light.s = 0.1f;

    limitFramerate = true;

    if (!shadowMapFBO.Init(this->width, this->height)) {
        std::cout << "Shadow map initialization failed\n";
        exit(0);
    }

    GLCall(glEnable(GL_DEPTH_TEST));
}

ShadowApplication::ShadowApplication(const char *title, std::string iconPath) : Application(title, iconPath) {
    camera.distanceToTarget = 5.f;
    camera.aspectRatio = float(width) / height;

    light.s = 0.1f;

    limitFramerate = true;

    if (!shadowMapFBO.Init(this->width, this->height)) {
        std::cout << "Shadow map initialization failed\n";
        exit(0);
    }

    GLCall(glEnable(GL_DEPTH_TEST));
}

void ShadowApplication::draw() {
    //Clear
    GLCall(glClearColor(clearColor[0], clearColor[1], clearColor[2], 1.f));
    GLCall(glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT));

    //Drawing
    prepareToDraw();
    shadowPass();
    renderPass();

    //ImGui
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    drawFPS();
    drawConsole();
    drawImGui();
    if (showPlots)
        drawImPlot();

    ImGui::EndFrame();
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ShadowApplication::prepareToDraw() {}

void ShadowApplication::shadowPass() {
    shadowMapFBO.BindForWriting();
    GLCall(glClear(GL_DEPTH_BUFFER_BIT));
    shadowMapRenderer.use();
    shadowMapRenderer.setMat4("projection", light.getOrthoProjectionMatrix());
    shadowMapRenderer.setMat4("view", light.getViewMatrix());
    GLCall(glViewport(0, 0, shadowMapFBO.bufferWidth, shadowMapFBO.bufferHeight));

    drawShadowCastingObjects(shadowMapRenderer);

    int bufferWidth, bufferHeight;
    glfwGetFramebufferSize(window, &bufferWidth, &bufferHeight);
    GLCall(glBindFramebuffer(GL_FRAMEBUFFER, 0));
    GLCall(glUseProgram(0));
    GLCall(glViewport(0, 0, bufferWidth, bufferHeight));
}

void ShadowApplication::renderPass() {
    shadowMapFBO.BindForReading(GL_TEXTURE0);

#define SETUP_SHADER(shader)                                      \
    (shader).use();                                               \
    (shader).setMat4("projection", camera.getProjectionMatrix()); \
    (shader).setMat4("view", camera.getViewMatrix());             \
    (shader).setVec3("camPos", camera.position());                \
    (shader).setVec3("lightPos", light.position());               \
    (shader).setVec3("lightColor", light.color());

    // set up shaders
    SETUP_SHADER(shadowShader);
    shadowShader.setMat4("lightProjection", light.getOrthoProjectionMatrix());
    shadowShader.setMat4("lightView", light.getViewMatrix());
    shadowShader.setInt("shadowMap", 0);
    shadowShader.setFloat("bias", shadowbias);

    SETUP_SHADER(basicShader);
    // better lighting approximation here so that regions of the model do
    // not remain forever shaded dark...
    //basicShader.setVec3("lightPos", camera.position());

    drawObjectsWithShadows(shadowShader);
    drawObjectsWithoutShadows(basicShader);
}

void ShadowApplication::drawObjectsWithShadows(const Shader &shader) {
    if (showGround)
        ground.draw(shader, groundIntensity, crl::gui::toV3D(groundColor));

    if(show_world_frame){
        crl::gui::drawArrow3d(P3D(0,0,0), world_frame_length * V3D(1,0,0), world_frame_radius, shader, V3D(0.75, 0.25, 0.25), 1.0);
        crl::gui::drawArrow3d(P3D(0,0,0), world_frame_length * V3D(0,1,0), world_frame_radius, shader, V3D(0.25, 0.75, 0.25), 1.0);
        crl::gui::drawArrow3d(P3D(0,0,0), world_frame_length * V3D(0,0,1), world_frame_radius, shader, V3D(0.25, 0.25, 0.75), 1.0);
    }
}

void ShadowApplication::drawImGui() {
    Application::drawImGui();

    ImGui::Begin("Main Menu");
    if (ImGui::TreeNode("Ground")) {
        ImGui::Checkbox("Show Ground", &showGround);
        static int size = ground.getSize();
        static double thickness = ground.gridThickness;
        if (ImGui::SliderInt("Ground Size", &size, 1.0, 100.0)) {
            ground.setSize(size);
            ground.gridThickness = thickness;
        }
        if (ImGui::SliderDouble("Grid Thickness", &thickness, 0.001, 0.1))
            ground.gridThickness = thickness;
        ImGui::Checkbox("Show Grid", &ground.showGrid);
        ImGui::SliderDouble("Ground Intensity", &groundIntensity, 0.0, 10.0);
        ImGui::ColorPicker3("Ground Color", &groundColor[0]);

        ImGui::TreePop();
    }
    if (ImGui::TreeNode("World frame")){
        ImGui::Checkbox("Show world frame", &show_world_frame);
        ImGui::SliderDouble("World frame length", &world_frame_length, 0.1, 1.0);
        ImGui::SliderDouble("World frame radius", &world_frame_radius, 0.01, 0.05);
        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Light")) {
        ImGui::SliderFloat("Shadow Bias", &shadowbias, 0.0f, 0.01f, "%.5f");
        ImGui::SliderFloat("Light Proj Scale", &light.s, 0.0f, 5.0f);
        ImGui::InputScalarN("Light Location", ImGuiDataType_Double, &light.pos, 3);

        static glm::vec3 lightDir = toGLM(light.pos);
        lightDir = toGLM(light.pos);
        if (ImGui::gizmo3D("##Dir1", lightDir))
            light.pos = toV3D(lightDir);
        //ImGui::Image((void *)shadowMapFBO.shadowMap, ImVec2(200, 200));

        ImGui::TreePop();
    }

    if (ImGui::TreeNode("Camera")) {
        if (ImGui::Button("Front")) {
            camera.rotAboutUpAxis = 0;
            camera.rotAboutRightAxis = 0.1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Left")) {
            camera.rotAboutUpAxis = PI / 2;
            camera.rotAboutRightAxis = 0.1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Right")) {
            camera.rotAboutUpAxis = -PI / 2;
            camera.rotAboutRightAxis = 0.1;
        }
        ImGui::SameLine();
        if (ImGui::Button("Up")) {
            camera.rotAboutUpAxis = 0;
            camera.rotAboutRightAxis = PI / 2;
        }

        ImGui::SliderFloat("Up Axis", &camera.rotAboutUpAxis, -PI, PI);
        ImGui::SliderFloat("Right Axis", &camera.rotAboutRightAxis, -PI, PI);

        ImGui::TreePop();
    }

    if (ImGui::TreeNode("App Settings")) {
        if (ImGui::SmallButton("Print To Terminal"))
            printCurrentAppSettings();
        if (ImGui::SmallButton("Save To File"))
            saveCurrentAppSettings();
        if (ImGui::SmallButton("Load From File"))
            loadCurrentAppSettings();
        ImGui::TreePop();
    }

    ImGui::End();
}

void ShadowApplication::resizeWindow(int width, int height) {
    camera.aspectRatio = float(width) / height;
    return Application::resizeWindow(width, height);
}

void ShadowApplication::resizeBuffer(int width, int height) {
    if (width != 0 && height != 0) {
        if (!shadowMapFBO.Init(width, height)) {
            std::cout << "Shadow map initialization failed\n";
            exit(0);
        }
    }
    return Application::resizeBuffer(width, height);
}

bool ShadowApplication::mouseMove(double xpos, double ypos) {
    camera.processMouseMove(mouseState, keyboardState);
    return true;
}

bool ShadowApplication::scrollWheel(double xoffset, double yoffset) {
    camera.processMouseScroll(xoffset, yoffset);
    return true;
}

void ShadowApplication::printCurrentAppSettings() const {
    std::cout << "--- CAMERA --- " << std::endl;
    std::cout << "target: " << camera.target.x << " " << camera.target.y << " " << camera.target.z << std::endl;
    std::cout << "rotAboutUpAxis: " << camera.rotAboutUpAxis << std::endl;
    std::cout << "rotAboutRightAxis: " << camera.rotAboutRightAxis << std::endl;
    std::cout << "distanceToTarget: " << camera.distanceToTarget << std::endl << std::endl;

    std::cout << "--- LIGHT --- " << std::endl;
    std::cout << "pos: " << light.pos.x() << " " << light.pos.y() << " " << light.pos.z() << std::endl;
    std::cout << "s: " << light.s << std::endl << std::endl;

    std::cout << "--- SHADOW --- " << std::endl;
    std::cout << "shadowbias:" << shadowbias << std::endl;
    std::cout << "-------------------------------------------------------------------------------" << std::endl;
}

bool ShadowApplication::saveCurrentAppSettings(const char *filePath) const {
    std::string file = filePath ? std::string(filePath) : CRL_DATA_FOLDER "/out/AppSettings-" + getCurrentDateAndTime() + ".json";

    if (!checkFileExtension(file, "json")) {
        Logger::print(Logger::RED, "ShadowApplication::saveCurrentAppSettings -> should be a `json` file... Abort!\n");
        return false;
    }

    std::ofstream f(file);
    if (!f.is_open()) {
        Logger::print(Logger::RED, "ShadowApplication::saveCurrentAppSettings -> -> file path `%s` could not be opened\n", file.c_str());
        return false;
    }

    nlohmann::json j;
    j["camera.target.x"] = camera.target.x;
    j["camera.target.y"] = camera.target.y;
    j["camera.target.z"] = camera.target.z;
    j["camera.rotAboutUpAxis"] = camera.rotAboutUpAxis;
    j["camera.rotAboutRightAxis"] = camera.rotAboutRightAxis;
    j["camera.distanceToTarget"] = camera.distanceToTarget;

    j["light.pos.x"] = light.pos.x();
    j["light.pos.y"] = light.pos.y();
    j["light.pos.z"] = light.pos.z();
    j["light.s"] = light.s;

    j["shadowbias"] = shadowbias;

    f << std::setw(2) << j << std::endl;
    f.close();

    Logger::print(Logger::GREEN, "ShadowApplication::saveCurrentAppSettings -> successfully saved into file `%s`\n", file.c_str());
    return true;
}

bool ShadowApplication::loadCurrentAppSettings(const char *filePath) {
    std::string file = filePath ? std::string(filePath) : browseFile();

    if (!checkFileExtension(file, "json")) {
        Logger::print(Logger::RED, "ShadowApplication::loadCurrentAppSettings -> should be a `json` file... Abort!\n");
        return false;
    }

    std::ifstream f(file);
    if (!f.is_open()) {
        Logger::print(Logger::RED, "ShadowApplication::loadCurrentAppSettings -> -> file path `%s` could not be opened\n", file.c_str());
        return false;
    }

    nlohmann::json j;
    f >> j;

    camera.target.x = j.value("camera.target.x", float());
    camera.target.y = j.value("camera.target.y", float());
    camera.target.z = j.value("camera.target.z", float());
    camera.rotAboutUpAxis = j.value("camera.rotAboutUpAxis", float());
    camera.rotAboutRightAxis = j.value("camera.rotAboutRightAxis", float());
    camera.distanceToTarget = j.value("camera.distanceToTarget", float());

    light.pos.x() = j.value("light.pos.x", double());
    light.pos.y() = j.value("light.pos.y", double());
    light.pos.z() = j.value("light.pos.z", double());
    light.s = j.value("light.s", float());

    shadowbias = j.value("shadowbias", float());

    f.close();

    Logger::print(Logger::GREEN, "ShadowApplication::loadCurrentAppSettings -> successfully loaded from file `%s`\n", file.c_str());
    return true;
}

}  // namespace gui
}  // namespace crl