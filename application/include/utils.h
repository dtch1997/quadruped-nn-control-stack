#ifndef APPLICATION__UTILS
#define APPLICATION__UTILS

#pragma once

#include <string>

char* getCmdOption(char ** begin, char ** end, const std::string& option);
bool cmdOptionExists(char** begin, char** end, const std::string& option);

#endif // APPLICATION__UTILS