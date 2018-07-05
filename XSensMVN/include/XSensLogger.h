/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef XSENS_LOGGER_H
#define XSENS_LOGGER_H

/* Hack required to print verbose log messages when not in Release
 * It is not possible to directly use the NDEBUG variable since XSens defines regardless from the
 * compile option
 * This brings as consequence that the Logger should be included in the .h files ALWAYS before XSens
 * SDK files
 */
#ifndef NDEBUG
#define XS_VERBOSE_LOG
#endif

#include <iostream>
#include <sstream>
#include <string>

// Define macros to print YARP-like info, warning, and error messages with exactly the same usage
// of std::cout (operator <<)
#ifndef xsError
#define xsError            \
    std::cerr << std::endl \
              << printLogMessage("[ERROR]", __FILE__, std::to_string(__LINE__), __FUNCTION__)
#endif

#ifndef xsWarning
#define xsWarning          \
    std::cout << std::endl \
              << printLogMessage("[WARNING]", __FILE__, std::to_string(__LINE__), __FUNCTION__)
#endif

#ifndef xsInfo
#define xsInfo             \
    std::cout << std::endl \
              << printLogMessage("[INFO]", __FILE__, std::to_string(__LINE__), __FUNCTION__)
#endif

// Utility function to print "advanced" log messages
inline std::string printLogMessage(const std::string& msgType,
                                   const std::string& file = "",
                                   const std::string& line = "",
                                   const std::string& function = "")
{
    std::string filename = file.substr(file.find_last_of("/\\") + 1);
    std::string ss;
    ss = msgType + " ";

    // If debug print addictional metadata
#ifdef XS_VERBOSE_LOG
    ss = ss + filename + "@" + function + ":" + line + " - ";
#endif
    return ss;
};

#endif // XSENS_LOGGER_H
