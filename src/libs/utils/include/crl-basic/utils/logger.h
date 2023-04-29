#pragma once

#pragma warning(disable : 4996)

#include <Eigen/Eigen>
#include <string>
#include <vector>

namespace crl {

class Logger {
public:
    struct ConsoleText {
        std::string text;
        Eigen::Vector3d color;
    };
    static std::vector<std::vector<Logger::ConsoleText>> consoleOutput;
    static int maxConsoleLineCount;

    enum PRINT_COLOR { RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, DEFAULT };

    static void print(const char *fmt, ...);
    static void print(PRINT_COLOR color, const char *fmt, ...);
    static void logPrint(const char *fmt, ...);
    static void consolePrint(const char *fmt, ...);
    static void consolePrint(const Eigen::Vector3d &color, const char *fmt, ...);

private:
    static std::string ms_strLogPath;
    static std::string ms_strPrintFileName;
    static std::string ms_strLogFileName;
    static std::string ms_strConsoleFileName;

    static void consolePrint(const Eigen::Vector3d &color, char *pBuffer);
    static void print(PRINT_COLOR color, char *pBuffer);
};

}  // namespace crl