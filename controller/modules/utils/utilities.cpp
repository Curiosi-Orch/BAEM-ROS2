#include "utils/utilities.h"

void ProcessLine(std::string* line, const std::string comment_str) {
  for (char& c : *line) {
    //"\t" "\r" "\n" are translated to be " "
    if (c == '\t' || c == '\r' || c == '\n') c = ' ';
  }
  line->erase(0, line->find_first_not_of(" "));  // erase space at the beginning
  line->erase(line->find_last_not_of(" ") + 1);  // erase space in the end
  MergeConsecutiveSpaces(line);

  std::size_t n_comment_start = line->find_first_of(comment_str);
  if (n_comment_start != std::string::npos)
    line->erase(n_comment_start);  // erase comment
}

std::vector<std::string> split(std::string s, std::string seperator) {
  std::vector<std::string> output;
  std::string::size_type prev_pos = 0, pos = 0;
  ProcessLine(&s);
  while ((pos = s.find(seperator, pos)) != std::string::npos) {
    std::string substring(s.substr(prev_pos, pos - prev_pos));
    output.push_back(substring);
    prev_pos = ++pos;
  }
  output.push_back(s.substr(prev_pos, pos - prev_pos));  // Last word
  return output;
}

bool AppendString2File(std::string file_path, std::string str) {
  std::ofstream File(file_path, std::ios::app);
  if (File.is_open()) {
    File << str << "\n";
  } else {
    std::cout << "ERROR SAVING FILE" << std::endl;
    return false;
  }
  return true;
}

bool LoadStringFromFile(std::string file_path, std::vector<std::string>* str,
                        const std::string comment_str) {
  std::ifstream inputs_file(file_path);
  if (inputs_file.is_open()) {
    std::string line;
    while (std::getline(inputs_file, line)) {
      ProcessLine(&line, comment_str);
      if (line.empty()) {
        continue;
      }
      str->push_back(line);
    }
    inputs_file.close();
  } else {
    return false;
  }
  return true;
}

void MergeConsecutiveSpaces(std::string* str) {
  std::string src = *str;
  std::string result = "";
  for (int i = 0; src[i] != '\0'; i++) {
    if (src[i] != ' ')
      result.append(1, src[i]);
    else if (src[i + 1] != ' ')
      result.append(1, src[i]);
  }
  *str = result;
}

double GetTimeStamp() {
  auto time_now = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  double timestamp = time_now.count()/1000.0;
  return timestamp;
}
