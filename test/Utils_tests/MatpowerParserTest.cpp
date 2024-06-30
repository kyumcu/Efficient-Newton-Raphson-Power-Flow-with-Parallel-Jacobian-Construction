#include <gtest/gtest.h>

#include <iostream>

#include "Utils/MatpowerParser.h"

namespace MMP = MatpowerParser;

TEST(MatpowerParserTest, FunctionCall)
{

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/case4gs.m",
                          pfData); // book: power_system_analysis_john_grainger_1st.pdf
                                   // pg: 337

  // MMP::printPowerFlowData(pfData);
}
