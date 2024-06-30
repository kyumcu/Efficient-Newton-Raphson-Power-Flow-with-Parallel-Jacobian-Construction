#include <gtest/gtest.h>

#include "Device_models/Device_onePort.h"

class OnePortDeviceTest : public ::testing::Test
{
protected:
  // Objects declared here can be used by all tests in the test suite.
  OnePortDevice<double, double> device;
};

TEST_F(OnePortDeviceTest, ConstructorTest)
{

  // The 'device' can be used directly here
  EXPECT_DOUBLE_EQ(device.getTerminalVoltage(), 0.0);
}

TEST_F(OnePortDeviceTest, GetSetTerminalVoltageTest)
{

  double testVoltage = 5.0; // Example voltage value
  device.setTerminalVoltage(testVoltage);
  EXPECT_DOUBLE_EQ(device.getTerminalVoltage(), testVoltage);
}
