#pragma once

#include <string>
#include <vector>

namespace MatpowerParser {

struct Bus
{
  int bus_i, type;
  double Pd, Qd, Gs, Bs;
  int area;
  double Vm, Va, baseKV;
  int zone;
  double Vmax, Vmin;
};

struct Generator
{
  int bus;
  double Pg, Qg, Qmax, Qmin, Vg, mBase, Pmax, Pmin;
  int status;
  double Pc1, Pc2;
  double Qc1min, Qc1max, Qc2min, Qc2max;
  double ramp_agc, ramp_10, ramp_30, ramp_q, apf;
};

struct Branch
{
  int fbus, tbus;
  double r, x, b;
  double rateA, rateB, rateC;
  double ratio, angle;
  int status, angmin, angmax;
};

struct PowerFlowData
{
  std::string version;
  std::string caseName;
  double baseMVA;
  std::vector<Bus> buses;
  std::vector<Generator> generators;
  std::vector<Branch> branches;
};

std::string
extractCaseName(const std::string& fullPath);

void
replaceAll(std::string& source, const std::string& from, const std::string& to);

void
parsePowerFlowData(const std::string& filename, PowerFlowData& pfData);

void
printPowerFlowData(const PowerFlowData& pfData);

}