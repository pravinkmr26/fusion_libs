#pragma once

#include <fstream>

namespace fusion::utils::file {}

namespace fusion::utils::csv {
class CSVReader {
  std::ofstream file;

public:
  CSVReader(const std::string& file_name) : file(file_name, std::ios::in){
    
  }

  ~CSVReader(){
    file.close();
  }
};
} // namespace fusion::utils::csv