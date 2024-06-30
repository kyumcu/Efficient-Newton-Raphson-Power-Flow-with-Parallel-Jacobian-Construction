#include <gtest/gtest.h>

#include <iostream>

#include "Network_models/Network_Model.h"

#include "Utils/DigestParsedData.h"
#include "Utils/MatpowerParser.h"
#include "Utils/MatrixCSVOps.h"
#include "Utils/TestHelpers.h"

namespace MMP = MatpowerParser;

TEST(DigestParsedDataTest, FunctionCall)
{

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/case4gs.m",
                          pfData); // book: power_system_analysis_john_grainger_1st.pdf
                                   // pg: 337

  auto Network = GridNetwork<double, double>();
  digest<double, double>(pfData, Network);
}

TEST(DigestParsedDataTest, ParseAndCompute)
{

  auto ref_YBus = loadMatrixFromCSV<std::complex<double>>(ROOTDIR "test/"
                                                                  "MatpowerData/"
                                                                  "Mapped/case4gs_Ybus.csv");

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/case4gs.m",
                          pfData); // book: power_system_analysis_john_grainger_1st.pdf
                                   // pg: 337

  auto Network = GridNetwork<double, double>();
  digest<double, double>(pfData, Network);

  Network.prepeare_PF();
  Eigen::Matrix<std::complex<double>, 4, 4> mat = Network.getYBus();
  expectMatrixNear(ref_YBus, mat);

  size_t iter_counter = 0;
  do {
    Network.do_PF();
    iter_counter++;

  } while (Network.getDeltaX().maxCoeff() > 1e-3);

  // printVector(Network.getDataVector(StateType::Voltage_rms));

  // printVector(Network.getDataVector(StateType::Phase_angle));
}
