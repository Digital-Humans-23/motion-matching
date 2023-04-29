#pragma once

#pragma warning(disable : 4996)

#include <assert.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace crl {

typedef struct key_word {
    char keyWord[50];
    int retVal;
} KeyWord;

inline bool isWhiteSpace(char ch) {
    return (ch == ' ' || ch == '\t' || ch == '\n' || ch == '\r' || ch == '\0');
}

/**
 * given a list of keywords that map strings to integer values denoting types
 * of keywords, this method will determine the type of command that is passed in
 * 'line' will be modified after this function call to point to the first
 * character after the keyword. If no keywords are matched, the method returns
 * -1
 */
inline int getLineType(char *&line, KeyWord *keywords, int nKeywords) {
    for (int i = 0; i < nKeywords; i++) {
        if (strncmp(line, keywords[i].keyWord, strlen(keywords[i].keyWord)) == 0 && isWhiteSpace(line[strlen(keywords[i].keyWord)])) {
            line += strlen(keywords[i].keyWord);
            return keywords[i].retVal;
        }
    }
    return -1;
}

/**
 * given a list of keywords that map strings to integer values denoting types of
 * keywords, this method will determine the string corresponding to the token
 * passed in
 */
inline char *getKeyword(int lineType, KeyWord *keywords, int nKeywords) {
    for (int i = 0; i < nKeywords; i++) {
        if (lineType == keywords[i].retVal)
            return keywords[i].keyWord;
    }

    return nullptr;
}

/**
 * This method returns a pointer to the first non-white space character location
 * in the provided buffer
 */
inline char *lTrim(char *buffer) {
    while (*buffer == ' ' || *buffer == '\t' || *buffer == '\n' || *buffer == '\r')
        buffer++;
    return buffer;
}

inline char *rTrim(char *buffer) {
    int index = (int)strlen(buffer) - 1;
    while (index >= 0) {
        if (buffer[index] == ' ' || buffer[index] == '\t' || buffer[index] == '\n' || buffer[index] == '\r') {
            buffer[index] = '\0';
            index--;
        } else
            break;
    }
    return buffer;
}

inline char *trim(char *buffer) {
    return rTrim(lTrim(buffer));
}

/**
 * This method reads a line from a file. It does not return empty lines or ones
 * that start with a pound key - those are assumed to be comments. This method
 * returns true if a line is read, false otherwise (for instance the end of file
 * is met).
 */
inline bool readValidLine(char *line, int nChars, FILE *fp) {
    line[0] = '\0';
    while (!feof(fp)) {
        if (fgets(line, nChars, fp)) {
            if ((int)strlen(line) >= nChars)
                std::cout << "The input file contains a line that is longer "
                             "than the buffer - not allowed"
                          << std::endl;
        } else
            break;  // well this should never happen but...
        char *tmp = trim(line);
        if (tmp[0] != '#' && tmp[0] != '\0')
            return true;
    }

    return false;
}

/**
    Checks the file extension of a file (path)
*/
bool checkFileExtension(const std::string& filePath, const std::string& extension);
/**
    Returns a string of the current date and time
*/
std::string getCurrentDateAndTime();

/**
    Opens file browser of corresponding operating system and returns the file path to the selected file
*/
std::string browseFile();

/**
    Creates the corresponding file directory
*/
bool createPath(const std::string &_strPath);

/**
 * This method returns a std::vector of char pointers that correspond to the
 * addressed of the tokens that are separated by white space in the string that
 * is passed in as a pointer.
 */
inline std::vector<char *> getTokens(char *input) {
    std::vector<char *> result;
    input = lTrim(input);
    // read in the strings one by one - assume that each tokens are less than
    // 100 chars in length
    while (input[0] != '\0') {
        result.push_back(input);
        char tempStr[100] = {
            '\0',
        };
        sscanf(input, "%s", tempStr);
        input = lTrim(input + strlen(tempStr));
    }
    return result;
}

#define GET_STRING_FROM_ARGUMENT_LIST(fmt, pBuffer)              \
    {                                                            \
        va_list args;                                            \
        pBuffer = nullptr;                                       \
        int length = 1024;                                       \
        int result = -1;                                         \
        while (result == -1) {                                   \
            delete[] pBuffer;                                    \
            pBuffer = new char[length + 1];                      \
            memset(pBuffer, 0, length + 1);                      \
            va_start(args, fmt);                                 \
            result = std::vsnprintf(pBuffer, length, fmt, args); \
            va_end(args);                                        \
            if (result >= length)                                \
                result = -1;                                     \
            length *= 2;                                         \
        }                                                        \
    }

#define RELEASE_STRING_FROM_ARGUMENT_LIST(pBuffer) \
    {                                              \
        delete[] pBuffer;                          \
        pBuffer = nullptr;                         \
    }

/**
 * This method throws an error with a specified text and arguments
 */
#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_DEFAULT "\x1b[0m"
inline void throwError(const char *fmt, ...) {
    char *pBuffer = nullptr;
    GET_STRING_FROM_ARGUMENT_LIST(fmt, pBuffer);
    std::cout << ANSI_COLOR_RED << "Error Thrown: " << pBuffer << ANSI_COLOR_DEFAULT << std::endl;
    throw pBuffer;
    RELEASE_STRING_FROM_ARGUMENT_LIST(pBuffer);
}

}  // namespace crl