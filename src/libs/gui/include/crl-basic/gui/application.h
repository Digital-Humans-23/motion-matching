#pragma once
#pragma warning(disable : 4312)

#include <glad/glad.h>

// do not put glfw before glad!
#include <GLFW/glfw3.h>
#include <crl-basic/gui/camera.h>
#include <crl-basic/gui/inputstate.h>
#include <crl-basic/gui/renderer.h>
#include <crl-basic/gui/shader.h>
#include <crl-basic/gui/shadow_casting_light.h>
#include <crl-basic/gui/shadow_map_fbo.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>
#include <imgui_widgets/imGuIZMOquat.h>
#include <imgui_widgets/imgui_add.h>
#include <imgui_widgets/implot.h>

#include <thread>

namespace crl {
namespace gui {

class Application {
public:
public:
    Application(const char *title, int width, int height, std::string iconPath = CRL_DATA_FOLDER "/crl.png");
    Application(const char *title, std::string iconPath = CRL_DATA_FOLDER "/crl.png");
    virtual ~Application();

    //--- Helpers
    virtual void init(const char *title, int width, int height, std::string iconPath = CRL_DATA_FOLDER "/crl.png");
    virtual void setCallbacks();
    virtual void run();

    //--- Process
    virtual void restart() {}
    virtual void process() {}
    virtual void baseProcess();
    virtual void processCallback();

    //--- Drawing
    virtual void draw();

    //--- ImGui
    virtual void drawImGui();
    virtual void drawImPlot();
    virtual void drawFPS();
    virtual void drawConsole();

    //--- Interaction
    virtual bool keyPressed(int key, int mods);
    virtual bool keyReleased(int key, int mods);
    virtual bool mouseButtonPressed(int button, int mods);
    virtual bool mouseButtonReleased(int button, int mods);
    virtual bool mouseMove(double xpos, double ypos);
    virtual bool scrollWheel(double xoffset, double yoffset);
    virtual bool drop(int count, const char **filenames);
    virtual void resizeWindow(int width, int height);
    virtual void resizeBuffer(int width, int height);

    /**
     * Adjust UI scale based on the framebuffer / window size ratio and window scale factor.
     */
    virtual void rescaleUI();

    //--- Screenshot
    virtual bool screenshot(const char *filename) const;

public:
    //--- Window
    GLFWwindow *window;
    int width, height;
    float pixelRatio = 1.0;
    float clearColor[3] = {1.f, 1.f, 1.f};

    //--- Interaction
    MouseState mouseState;
    KeyboardState keyboardState;

    //--- Framerate
    bool limitFramerate = true;
    int targetFramerate = 60;
    float averageFPS = 0.f;
    float averagePercentTimeSpentProcessing = 0.f;

    //--- Process
    bool useSeparateProcessThread = false;
    std::thread processThread;
    bool processIsRunning = false;

    //--- Console
    bool automanageConsole = false;
    bool showConsole = false;
    bool showPlots = false;
    int consoleHeight = 250;  // in pixels

    //--- Screenshot
    bool screenIsRecording = false;
    int screenShotCounter = 0;
    std::string screenshotPath = CRL_DATA_FOLDER "/out/screenshots";
};

//-----------------------------------------------------------------------------------------------------------------------------------------------------------------

class ShadowApplication : public Application {
public:
    ShadowApplication(const char *title, int width, int height, std::string iconPath = CRL_DATA_FOLDER "/crl.png");
    ShadowApplication(const char *title, std::string iconPath = CRL_DATA_FOLDER "/crl.png");
    ~ShadowApplication() override = default;

    //--- Drawing
    virtual void prepareToDraw();
    virtual void drawShadowCastingObjects(const Shader &shader) {}   // objects that will cast a shadow
    virtual void drawObjectsWithShadows(const Shader &shader);       // objects that will have a shadow cast on them
    virtual void drawObjectsWithoutShadows(const Shader &shader) {}  // objectst that will NOT have shadows cast on them

    virtual void draw() override;
    virtual void shadowPass();
    virtual void renderPass();

    //--- ImGui
    virtual void drawImGui() override;

    //--- Interaction
    virtual void resizeWindow(int width, int height) override;
    virtual void resizeBuffer(int width, int height) override;
    virtual bool mouseMove(double xpos, double ypos) override;
    virtual bool scrollWheel(double xoffset, double yoffset) override;

    //--- App settings
    virtual void printCurrentAppSettings() const;
    virtual bool saveCurrentAppSettings(const char *filePath = nullptr) const;
    virtual bool loadCurrentAppSettings(const char *filePath = nullptr);

public:
    //--- Camera
    TrackingCamera camera;

    //--- Light & Shadows
    ShadowCastingLight light;
    ShadowMapFBO shadowMapFBO;
    float shadowbias = 0.0001f;

    //--- Ground
    SizableGroundModel ground = SizableGroundModel(10);
    bool showGround = true;
    double groundIntensity = 1.5;
    float groundColor[3] = {1.0, 1.0, 1.0};

    //--- World
    double world_frame_length = 1.0, world_frame_radius = 0.01;
    bool show_world_frame = true;

    //--- Shaders
    Shader shadowShader = Shader(CRL_SHADER_FOLDER "/basic_lighting.vert", CRL_SHADER_FOLDER "/basic_shadow_lighting.frag");
    Shader shadowMapRenderer = Shader(CRL_SHADER_FOLDER "/basic_lighting.vert", CRL_SHADER_FOLDER "/render_shadow.frag");
    Shader basicShader = Shader(CRL_SHADER_FOLDER "/basic_lighting.vert", CRL_SHADER_FOLDER "/basic_lighting.frag");
};

}  // namespace gui
}  // namespace crl