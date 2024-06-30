#include <gtest/gtest.h>

#include <iostream>

#include "Utils/MatrixCSVOps.h"
#include "Utils/TestHelpers.h"

TEST(MatrixCSVOpsTest, case9)
{

  auto mat = loadMatrixFromCSV<std::complex<double>>(ROOTDIR "test/"
                                                             "MatpowerData/"
                                                             "Mapped/case9_Ybus.csv");

  Eigen::Matrix<std::complex<double>, 9, 9> ref_YBus(9, 9);

  ref_YBus.setZero(); // Initialize with zeros

  // Assign the given complex numbers to their positions
  ref_YBus(0, 0) = std::complex<double>(0, -17.3611);
  ref_YBus(0, 3) = std::complex<double>(0, 17.3611);
  ref_YBus(1, 1) = std::complex<double>(0, -16);
  ref_YBus(1, 7) = std::complex<double>(0, 16);
  ref_YBus(2, 2) = std::complex<double>(0, -17.0648);
  ref_YBus(2, 5) = std::complex<double>(0, 17.0648);
  ref_YBus(3, 0) = std::complex<double>(0, 17.3611);
  ref_YBus(3, 3) = std::complex<double>(3.30738, -39.3089);
  ref_YBus(3, 4) = std::complex<double>(-1.94219, 10.5107);
  ref_YBus(3, 8) = std::complex<double>(-1.36519, 11.6041);
  ref_YBus(4, 3) = std::complex<double>(-1.94219, 10.5107);
  ref_YBus(4, 4) = std::complex<double>(3.2242, -15.8409);
  ref_YBus(4, 5) = std::complex<double>(-1.28201, 5.58824);
  ref_YBus(5, 2) = std::complex<double>(0, 17.0648);
  ref_YBus(5, 4) = std::complex<double>(-1.28201, 5.58824);
  ref_YBus(5, 5) = std::complex<double>(2.4371, -32.1539);
  ref_YBus(5, 6) = std::complex<double>(-1.15509, 9.78427);
  ref_YBus(6, 5) = std::complex<double>(-1.15509, 9.78427);
  ref_YBus(6, 6) = std::complex<double>(2.77221, -23.3032);
  ref_YBus(6, 7) = std::complex<double>(-1.61712, 13.698);
  ref_YBus(7, 1) = std::complex<double>(0, 16);
  ref_YBus(7, 6) = std::complex<double>(-1.61712, 13.698);
  ref_YBus(7, 7) = std::complex<double>(2.80473, -35.4456);
  ref_YBus(7, 8) = std::complex<double>(-1.1876, 5.97513);
  ref_YBus(8, 3) = std::complex<double>(-1.36519, 11.6041);
  ref_YBus(8, 7) = std::complex<double>(-1.1876, 5.97513);
  ref_YBus(8, 8) = std::complex<double>(2.55279, -17.3382);

  expectMatrixNear(ref_YBus, mat);
}