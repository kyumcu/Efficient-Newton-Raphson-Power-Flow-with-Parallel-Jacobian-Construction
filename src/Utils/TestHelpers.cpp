#include <vector>

#include "Utils/TestHelpers.h"
#include <fstream>

std::vector<BusData>
readCsv(const std::string& filename)
{
  std::vector<BusData> data;
  std::ifstream file(filename);
  std::string line, cell;

  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return data;
  }

  // Skip the header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::istringstream lineStream(line);
    std::vector<std::string> row;

    while (std::getline(lineStream, cell, ',')) {
      row.push_back(cell);
    }

    if (row.size() == 3) {
      data.emplace_back(std::stoi(row[0]), std::stod(row[1]), std::stod(row[2]));
    }
  }

  return data;
}
