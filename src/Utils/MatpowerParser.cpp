#include <fstream> // For file reading
#include <sstream> // For std::istringstream

#include <iomanip> // For std::setw and std::setprecision
#include <iostream>
#include <string>
#include <vector>

#include "Utils/MatpowerParser.h"

namespace MatpowerParser {

// Function to replace all occurrences of a substring with another substring
void
replaceAll(std::string& source, const std::string& from, const std::string& to)
{
  std::string::size_type pos = 0;
  while ((pos = source.find(from, pos)) != std::string::npos) {
    source.replace(pos, from.length(), to);
    pos += to.length();
  }
}

std::string
extractCaseName(const std::string& fullPath)
{

  size_t lastSlashPos = fullPath.find_last_of("/");
  size_t lastDotPos = fullPath.find_last_of(".");
  std::string extracted;

  if (lastSlashPos != std::string::npos && lastDotPos != std::string::npos) {
    extracted = fullPath.substr(lastSlashPos + 1, lastDotPos - lastSlashPos - 1);
  } else {
    std::cout << "Format not recognized." << std::endl;
    extracted = "Bad Format";
  }

  return extracted;
}

void
parsePowerFlowData(const std::string& filename, PowerFlowData& pfData)
{
  pfData.caseName = extractCaseName(filename);

  std::ifstream file(filename);
  std::string line;
  bool parseBusData = false, parseGenData = false, parseBranchData = false;

  if (!file.is_open()) {
    std::cerr << "Failed to open file: " << filename << std::endl;
    return;
  }

  // get baseMVA
  while (getline(file, line)) {
    // Simple parsing example: Look for the baseMVA line
    if (line.find("mpc.baseMVA") != std::string::npos) {
      std::istringstream iss(line);
      std::string temp;
      double baseMVA;
      iss >> temp >> temp >> baseMVA; // Skipping "mpc.baseMVA ="
      pfData.baseMVA = baseMVA;
      break; // Example stops after finding baseMVA
    }
  }
  while (getline(file, line)) {

    // Trim leading and trailing whitespace from the line (optional,
    // based on data format)
    line.erase(0, line.find_first_not_of(" \t\n\r\f\v")); // Left trim
    line.erase(line.find_last_not_of(" \t\n\r\f\v") + 1); // Right trim

    // Skip empty lines or lines starting with '%'
    if (line.empty() || line[0] == '%') {
      continue; // Skip this iteration and proceed to the next line
    }

    replaceAll(line, "Inf", "0");
    replaceAll(line, "-Inf", "0");

    std::istringstream iss(line);

    if (line.find("mpc.bus = [") != std::string::npos) {
      parseBusData = true;
      continue;
    } else if (line.find("mpc.gen = [") != std::string::npos) {
      parseBusData = false; // Stop parsing bus data
      parseGenData = true;
      continue;
    } else if (line.find("mpc.branch = [") != std::string::npos) {
      parseGenData = false; // Stop parsing generator data
      parseBranchData = true;
      continue;
    }

    if (parseBusData && line.find("];") != std::string::npos) {
      parseBusData = false; // Stop parsing bus data
      continue;
    } else if (parseGenData && line.find("];") != std::string::npos) {
      parseGenData = false; // Stop parsing generator data
      continue;
    } else if (parseBranchData && line.find("];") != std::string::npos) {
      parseBranchData = false; // Stop parsing branch data
      break;                   // Assuming branch is the last data block you're
                               // interested in
    }

    if (parseBusData) {
      Bus bus;
      if (iss >> bus.bus_i >> bus.type >> bus.Pd >> bus.Qd >> bus.Gs >> bus.Bs >> bus.area >>
          bus.Vm >> bus.Va >> bus.baseKV >> bus.zone >> bus.Vmax >> bus.Vmin) {
        pfData.buses.push_back(bus);
      } else {
        std::cerr << "Error parsing bus line: " << line << std::endl;
      }
    } else if (parseGenData) {
      Generator gen;
      if (iss >> gen.bus >> gen.Pg >> gen.Qg >> gen.Qmax >> gen.Qmin >> gen.Vg >> gen.mBase >>
          gen.status >> gen.Pmax >> gen.Pmin >> gen.Pc1 >> gen.Pc2 >> gen.Qc1min >> gen.Qc1max >>
          gen.Qc2min >> gen.Qc2max >> gen.ramp_agc >> gen.ramp_10 >> gen.ramp_30 >> gen.ramp_q >>
          gen.apf) {
        pfData.generators.push_back(gen);
      } else {
        std::cerr << "Error parsing gen line: " << line << std::endl;
      }
    } else if (parseBranchData) {
      Branch branch;
      if (iss >> branch.fbus >> branch.tbus >> branch.r >> branch.x >> branch.b >> branch.rateA >>
          branch.rateB >> branch.rateC >> branch.ratio >> branch.angle >> branch.status >>
          branch.angmin >> branch.angmax) {
        pfData.branches.push_back(branch);
      } else {
        std::cerr << "Error parsing branch line: " << line << std::endl;
      }
    } else
      std::cerr << "Error parsing UNKWN line: " << line << std::endl;
  }

  // Extend this function to parse other parts of the data.
}

void
printPowerFlowData(const PowerFlowData& pfData)
{
  std::cout << "MATPOWER Case Format - Version: " << pfData.version << "\n";
  std::cout << "Base MVA: " << pfData.baseMVA << "\n\n";

  std::cout << "Bus Data:\n";
  std::cout << "Bus\tType\tPd\tQd\tGs\tBs\tArea\tVm\tVa\tBaseKV\tZone"
               "\tVmax\tVmin\n";
  for (const auto& bus : pfData.buses) {
    std::cout << std::setw(4) << bus.bus_i << "\t" << bus.type << "\t" << std::setw(6)
              << std::setprecision(2) << std::fixed << bus.Pd << "\t" << bus.Qd << "\t" << bus.Gs
              << "\t" << bus.Bs << "\t" << bus.area << "\t" << bus.Vm << "\t" << bus.Va << "\t"
              << bus.baseKV << "\t" << bus.zone << "\t" << bus.Vmax << "\t" << bus.Vmin << "\n";
  }

  std::cout << "\nGenerator Data:\n";
  std::cout << "Bus\tPg\tQg\tQmax\tQmin\tVg\tmBase\tStatus\tPmax\tPmin\n";
  for (const auto& gen : pfData.generators) {
    std::cout << gen.bus << "\t" << gen.Pg << "\t" << gen.Qg << "\t" << gen.Qmax << "\t" << gen.Qmin
              << "\t" << gen.Vg << "\t" << gen.mBase << "\t" << gen.status << "\t" << gen.Pmax
              << "\t" << gen.Pmin << "\n";
  }

  std::cout << "\nBranch Data:\n";
  std::cout << "Fbus\tTbus\tR\tX\tB\tRateA\tRateB\tRateC\tRatio\tAngl"
               "e\tStatus"
               "\tAngmin\tAngmax\n";
  for (const auto& branch : pfData.branches) {
    std::cout << branch.fbus << "\t" << branch.tbus << "\t" << branch.r << "\t" << branch.x << "\t"
              << branch.b << "\t" << branch.rateA << "\t" << branch.rateB << "\t" << branch.rateC
              << "\t" << branch.ratio << "\t" << branch.angle << "\t" << branch.status << "\t"
              << branch.angmin << "\t" << branch.angmax << "\n";
  }
}

} // end namespace MatpowerParser