#include <gtest/gtest.h>

#include <iostream>

#include "Network_models/Network_Model.h"

#include "Utils/DigestParsedData.h"
#include "Utils/MatpowerParser.h"
#include "Utils/MatrixCSVOps.h"
#include "Utils/TestHelpers.h"

#include "Utils/DebugTools.h"

namespace MMP = MatpowerParser;

void
testMacro(std::string case_str, int g_size, bool print = true)
{

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/" + case_str + ".m", pfData);

  auto Network = GridNetwork<double, double>();
  digest<double, double>(pfData, Network);

  EXPECT_EQ(Network._NetworkNodes.size(), g_size);

  Network.prepeare_PF();

  if (print) {
#ifdef DEBUG
    Network.PrintNetworkStats();
    Network.writeReverse_mapToCSV();
#else
    std::cout << "enable -DDEBUG for extras." << std::endl;
#endif
  }
  compareYbusOf(Network,
                ROOTDIR "test/"
                        "MatpowerData/" +
                  case_str + "_Ybus.csv");

  size_t iter_counter = 0;
  do {
    Network.do_PF();
    iter_counter++;
    if (iter_counter == 1)
      compareJacOf(Network,
                   ROOTDIR "test/"
                           "MatpowerData/" +
                     case_str + "_Jac.csv");

  } while (Network.getDeltaX().maxCoeff() > 1e-6);

  compareResOf(Network,
               ROOTDIR "test/"
                       "MatpowerData/" +
                 case_str + ".csv");
}

TEST(MatpowerTests, case4gs)
{
  testMacro("case4gs", 4, 0);
}

TEST(MatpowerTests, case9)
{
  testMacro("case9", 9, 0);
}

TEST(MatpowerTests, case30)
{
  testMacro("case30", 30, 0);
}

TEST(MatpowerTests, case57)
{
  testMacro("case57", 57, 0);
}

TEST(MatpowerTests, case118)
{
  testMacro("case118", 118, 0);
}

TEST(MatpowerTests, case89pegase)
{
  testMacro("case89pegase", 89, 0);
}

TEST(MatpowerTests, case300)
{
  testMacro("case300", 300, 0);
}

TEST(MatpowerTests, case1354pegase)
{
  testMacro("case1354pegase", 1354, 0);
}

TEST(MatpowerTests, case_ACTIVSg2000)
{
  testMacro("case_ACTIVSg2000", 2000, 0);
}

TEST(MatpowerTests, case_ACTIVSg10k)
{
  testMacro("case_ACTIVSg10k", 10000, 0);
}

TEST(MatpowerTests, case_ACTIVSg25k)
{
  testMacro("case_ACTIVSg25k", 25000, 0);
}

TEST(MatpowerTests, case_SyntheticUSA)
{

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/case_SyntheticUSA.m", pfData);

  auto Network = GridNetwork<double, double>();
  digest<double, double>(pfData, Network);

  EXPECT_EQ(Network._NetworkNodes.size(), 82000);

  Network.prepeare_PF();
  Network.PrintNetworkStats();

  compareYbusOf(Network,
                ROOTDIR "test/"
                        "MatpowerData/case_SyntheticUSA_Ybus.csv");

  size_t iter_counter = 0;
  do {
    Network.do_PF();
    iter_counter++;
    if (iter_counter == 1)
      compareJacOf(Network,
                   ROOTDIR "test/"
                           "MatpowerData/case_SyntheticUSA_Jac.csv");

  } while (Network.getDeltaX().maxCoeff() > 1e-6);

  compareResOf(Network,
               ROOTDIR "test/"
                       "MatpowerData/case_SyntheticUSA.csv");
}

TEST(MatpowerTests, case2868rte)
{

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/case2868rte.m", pfData);

  auto Network = GridNetwork<double, double>();
  digest<double, double>(pfData, Network);

  EXPECT_EQ(Network._NetworkNodes.size(), 2868);

  Network.prepeare_PF();
  Network.PrintNetworkStats();
  Network.writeReverse_mapToCSV();

  compareYbusOf(Network,
                ROOTDIR "test/"
                        "MatpowerData/case2868rte_Ybus.csv");

  size_t iter_counter = 0;
  do {
    Network.do_PF();
    iter_counter++;
    if (iter_counter == 1)
      compareJacOf(Network,
                   ROOTDIR "test/"
                           "MatpowerData/case2868rte_Jac.csv");
    break;

  } while (Network.getDeltaX().maxCoeff() > 1e-6);

  compareResOf(Network,
               ROOTDIR "test/"
                       "MatpowerData/case2868rte.csv");

  Network.PrintNetworkStats();
}

TEST(MatpowerTests, case6515rte)
{

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/case6515rte.m", pfData);

  auto Network = GridNetwork<double, double>();
  digest<double, double>(pfData, Network);

  EXPECT_EQ(Network._NetworkNodes.size(), 6515);

  Network.prepeare_PF();
  Network.PrintNetworkStats();
  Network.writeReverse_mapToCSV();

  compareYbusOf(Network,
                ROOTDIR "test/"
                        "MatpowerData/case6515rte_Ybus.csv");

  size_t iter_counter = 0;
  do {
    Network.do_PF();
    iter_counter++;
    if (iter_counter == 1)
      compareJacOf(Network,
                   ROOTDIR "test/"
                           "MatpowerData/case6515rte_Jac.csv");

  } while (Network.getDeltaX().maxCoeff() > 1e-6);

  compareResOf(Network,
               ROOTDIR "test/"
                       "MatpowerData/case6515rte.csv");
}
