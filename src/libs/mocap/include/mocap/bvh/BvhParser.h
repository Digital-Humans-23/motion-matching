#pragma once

// Lib includes
#include "mocap/bvh/Bvh.h"
#include "mocap/bvh/BvhJoint.h"

// STL includes
#include <algorithm>
#include <functional>
#include <locale>
#include <memory>

// Filesystem include - check compiler version
// TODO: this needs to be verified for different platforms...
#if defined(__clang__)

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#elif defined(__GNUC__)

// becareful... clang is also __GNUC__
#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#endif

#elif defined(_WIN32)

#if _MSC_VER < 1920  // MSVC 2017
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else  // MSVC 2019
#include <filesystem>
namespace fs = std::filesystem;
#endif

#endif
namespace crl::mocap {

/** Bvh Parser class that is responsible for parsing .bvh file */
class BvhParser {
public:
    /** Parses single bvh file and stored data into bvh structure
     *  \param  path  The path to file to be parsed
     *  \param  bvh   The pointer to bvh object where parsed data will be stored
     *  \return  0 if success, -1 otherwise
     */
    int parse(const fs::path &path, Bvh *bvh);

private:
    /** Parses single hierarchy in bvh file
     *  \param  file  The input stream that is needed for reading file content
     *  \return  0 if success, -1 otherwise
     */
    int parseHierarchy(std::ifstream &file);

    /** Parses joint and its children in bvh file
     *  \param  file    The input stream that is needed for reading file content
     *  \param  parent  The pointer to parent joint
     *  \param  parsed  The output parameter, here will be stored parsed joint
     *  \return  0 if success, -1 otherwise
     */
    int parseJoint(std::ifstream &file, std::shared_ptr<BvhJoint> parent, std::shared_ptr<BvhJoint> &parsed);

    /** Parses order of channel for single joint
     *  \param  file    The input stream that is needed for reading file content
     *  \param  joint   The pointer to joint that channels order will be parsed
     *  \return  0 if success, -1 otherwise
     */
    int parseChannelOrder(std::ifstream &file, std::shared_ptr<BvhJoint> joint);

    /** Parses motion part data
     *  \param  file    The input stream that is needed for reading file content
     *  \return  0 if success, -1 otherwise
     */
    int parseMotion(std::ifstream &file);

    /** Trims the string, removes leading and trailing whitespace from it
     *  \param  s   The string, which leading and trailing whitespace will be
     *              trimmed
     */
    inline void trim(std::string &s) {
        s.erase(std::remove_if(s.begin(), s.end(), std::bind(std::isspace<char>, std::placeholders::_1, std::locale::classic())), s.end());
    }

    /** Converts the vector of float to string, ex. "el1, el2, el3"
     *  \param  vector  The data that will be converted to string
     *  \return  The string that will be created from input data
     */
    std::string vtos(const std::vector<float> &vector);

    /** The bvh object to store parsed data */
    Bvh *bvh_;

    /** Next joint index */
    unsigned nextJointIndex_{0};
};

}  // namespace crl::mocap
