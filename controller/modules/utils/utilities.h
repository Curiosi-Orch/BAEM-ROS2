#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <chrono>

/**
 * @brief split string with given seperators
 *
 * @param s input string
 * @param seperator delimator such as "," and " "
 * @return std::vector<std::string>
 */
std::vector<std::string> split(std::string s, std::string seperator);

/**
 * @brief append string to given file
 * @param file_path
 * @param str
 */
bool AppendString2File(std::string file_path, std::string str);

/**
 * @brief remove commented and empty line.
 * @param line
 * @param comment_str default "#"
 */
void ProcessLine(std::string* line, const std::string comment_str = "#");

bool LoadStringFromFile(std::string file_path, std::vector<std::string>* str,
                        const std::string comment_str = "#");

void MergeConsecutiveSpaces(std::string* str);

/**
 * @brief Get the Values From String Split object
 * @param str input string
 * @param separator delimator such as "," and " "
 * @return std::vector<T>
 */
template <class T>
std::vector<T> GetValuesFromStringSplit(std::string str,
                                        std::string separator) {
  std::vector<T> values;
  std::vector<std::string> str_list = split(str, separator);
  for (int i = 0; i < str_list.size(); ++i) {
    values.push_back(std::atof(str_list[i].c_str()));
  }
  return values;
}

// get system time stamp in milliseconds
double GetTimeStamp();


#endif  // UTILITIES_H_