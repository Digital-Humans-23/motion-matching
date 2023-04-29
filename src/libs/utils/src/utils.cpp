#include "crl-basic/utils/utils.h"

#include <algorithm>
#if WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif
#include <ctime>

namespace crl {

bool checkFileExtension(const std::string& filePath, const std::string& extension) {
    std::string fpExt = filePath.substr(filePath.find_last_of('.') + 1);
    return fpExt == extension;
}

std::string getCurrentDateAndTime() {
    time_t now = time(nullptr);
    char name[80];
    struct tm tstruct = *localtime(&now);
    strftime(name, sizeof(name), "%Y-%m-%d_%X", &tstruct);
    std::string name_str = std::string(name);
    std::replace(name_str.begin(), name_str.end(), ':', '-');
    return name_str;
}

std::string browseFile() {
#if WIN32
    const int bufferSize = MAX_PATH;
    char currentDir[bufferSize];
    if (!GetCurrentDirectory(bufferSize, currentDir))
        std::cerr << "WARNING in app::utils::browseFile -> could not GET current directory" << std::endl;

    OPENFILENAME ofn;
    char szFile[100];

    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = nullptr;
    ofn.lpstrFile = szFile;
    ofn.lpstrFile[0] = '\0';
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrFilter = "All\0*.*\0Text\0*.TXT\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrFileTitle = nullptr;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = nullptr;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
    GetOpenFileName(&ofn);

    if (!SetCurrentDirectory(currentDir))
        std::cerr << "Error in Utils::browseFile -> could not SET current directory" << std::endl;

    return std::string(szFile);
#else
    std::string openString = "zenity --file-selection --filename=";
    FILE* f = popen(openString.c_str(), "r");
    char file[1024];
    char* fg = fgets(file, 1024, f);
    std::string filePath = std::string(file);
    filePath.erase(std::remove(filePath.begin(), filePath.end(), '\n'), filePath.end());
    return filePath;
#endif
}

bool createPath(const std::string& _strPath) {
#if WIN32
    std::string stemp = std::string(_strPath.begin(), _strPath.end());
    LPCSTR sw = stemp.c_str();

    bool bSuccess = false;
    if (CreateDirectory(sw, nullptr)) {
        // Directory created
        bSuccess |= true;
    } else if (ERROR_ALREADY_EXISTS == GetLastError()) {
        // Directory already exists
        bSuccess |= true;
    }

    return bSuccess;
#else
    const int dir_err = mkdir(_strPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (-1 == dir_err)
        return false;
    return true;
#endif
}

}  // namespace crl