#pragma once

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <algorithm> // For std::replace
#include <cmath>
#include <complex>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "Network_models/Network_Model.h"
#include "Utils/Naming.h"

template<typename T>
Eigen::SparseMatrix<T>
loadMatrixFromCSV(const std::string& filename)
{
  std::ifstream file(filename);
  if (!file) {
    throw std::runtime_error("Could not open file: " + filename);
  }

  std::string line;
  std::vector<Eigen::Triplet<T>> tripletList;
  int maxRow = 0, maxCol = 0;

  getline(file, line);

  while (getline(file, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream lineStream(line);
    int row, col;
    T value;

    lineStream >> row >> col;
    if constexpr (std::is_same<T, std::complex<double>>::value) {
      double real, imag;
      lineStream >> real >> imag;
      value = T(real, imag);
    } else {
      lineStream >> value;
    }

    tripletList.emplace_back(row, col, value);
    maxRow = std::max(maxRow, row + 1);
    maxCol = std::max(maxCol, col + 1);
  }

  Eigen::SparseMatrix<T> matrix(maxRow, maxCol);
  matrix.setFromTriplets(tripletList.begin(), tripletList.end());

  return matrix;
}

// Helper trait to determine if a matrix type is sparse
template<typename T>
struct is_sparse_matrix : std::false_type
{
};

template<typename T, int Options, typename Index>
struct is_sparse_matrix<Eigen::SparseMatrix<T, Options, Index>> : std::true_type
{
};

// Function to compare matrices with a tolerance for floating-point
// values
template<typename MatrixType_1, typename MatrixType_2>
void
expectMatrixNear(const MatrixType_1& mat1, const MatrixType_2& mat2, double tolerance = 0.000001)
{
  ASSERT_EQ(mat1.rows(), mat2.rows());
  ASSERT_EQ(mat1.cols(), mat2.cols());

  std::ofstream logFile;
  logFile.open(OUTDIR "expectMatrixNear.log", std::ios::app); // Open in append mode

  if constexpr (is_sparse_matrix<MatrixType_1>::value || is_sparse_matrix<MatrixType_2>::value) {
    // Special handling for sparse matrices
    auto diff = (mat1 - mat2).eval(); // Ensure we have the right type for iteration
    for (int k = 0; k < diff.outerSize(); ++k) {
      for (typename decltype(diff)::InnerIterator it(diff, k); it; ++it) {
        if (std::abs(it.value()) > tolerance) {

          logFile << it.row() << "," << it.col() << "," << it.value() << "\n";
          logFile.flush();
        }
        EXPECT_LE(std::abs(it.value()), tolerance);
      }
    }
  } else {
    // Handling for dense matrices
    for (int i = 0; i < mat1.rows(); ++i) {
      for (int j = 0; j < mat1.cols(); ++j) {
        auto diff = std::abs(mat1(i, j) - mat2(i, j));
        EXPECT_LE(diff, tolerance) << "Matrices differ at (" << i << "," << j << ")";
      }
    }
  }
}

template<typename T>
void
expectVectorNear(const EigenRDVType<T>& vec1,
                 const EigenRDVType<T>& vec2,
                 T tolerance = static_cast<T>(0.000001))
{
  ASSERT_EQ(vec1.size(), vec2.size()) << "Vectors have different sizes.";

  for (uint i = 0; i < vec1.size(); ++i) {
    T diff = std::abs(vec1(i) - vec2(i));
    EXPECT_LE(diff, tolerance) << "Vectors differ at index " << i;
  }
}

template<typename T>
void
expectVectorNear(const EigenRDVType<T>& vec1,
                 const std::vector<T>& vec2,
                 T tolerance = static_cast<T>(0.000001))
{
  Eigen::Map<const EigenRDVType<T>> eigenVecMapped(vec2.data(), vec2.size());
  expectVectorNear(vec1, eigenVecMapped, tolerance);
}
template<typename T>
void
expectVectorNear(const std::vector<T>& vec2,
                 const EigenRDVType<T>& vec1,
                 T tolerance = static_cast<T>(0.000001))
{
  Eigen::Map<const EigenRDVType<T>> eigenVecMapped(vec2.data(), vec2.size());
  expectVectorNear(vec1, vec2, tolerance);
}
template<typename T>
void
expectVectorNear(const std::vector<T>& vec1,
                 const std::vector<T>& vec2,
                 T tolerance = static_cast<T>(0.000001))
{
  Eigen::Map<const EigenRDVType<T>> eigenVecMapped_1(vec1.data(), vec1.size());
  Eigen::Map<const EigenRDVType<T>> eigenVecMapped_2(vec2.data(), vec2.size());
  expectVectorNear(eigenVecMapped_1, eigenVecMapped_2, tolerance);
}

template<typename T, typename U>
void
compareYbusOf(GridNetwork<T, U>& Network, const std::string& filename)
{
  Network.writeY_busToCSV();
  auto mat1 =
    loadMatrixFromCSV<std::complex<T>>(std::string(OUTDIR) + Network._caseName + "_Ybus.csv");
  auto mat2 = loadMatrixFromCSV<std::complex<T>>(filename);
  expectMatrixNear(mat1, mat2);
}

template<typename T, typename U>
void
compareJacOf(GridNetwork<T, U>& Network, const std::string& filename)
{
  Network.writeJacToCSV();
  auto mat1 =
    loadMatrixFromCSV<std::complex<T>>(std::string(OUTDIR) + Network._caseName + "_Jac.csv");
  auto mat2 = loadMatrixFromCSV<std::complex<T>>(filename);
  expectMatrixNear(mat1, mat2);
}

struct BusData
{
  int bus_i;
  double Vm, Va;

  // Constructor for easy initialization
  BusData(int bus_i, double Vm, double Va)
    : bus_i(bus_i)
    , Vm(Vm)
    , Va(Va)
  {
  }
};

std::vector<BusData>
readCsv(const std::string& filename);

template<typename T, typename U>
void
compareResOf(GridNetwork<T, U>& Network, const std::string& filename)
{
  Network.writeVoltagePhToCSV();
  std::vector<BusData> data1 = readCsv(std::string(OUTDIR) + Network._caseName + ".csv");
  std::vector<BusData> data2 = readCsv(filename);

  if (data1.size() != data2.size()) {
    ASSERT_EQ(data1.size(), data2.size()) << "Result Vectors have different sizes.";
    return;
  }

  double diff = 0.0, tol = 0.0001;
  for (size_t i = 0; i < data1.size(); ++i) {
    ASSERT_EQ(data1[i].bus_i, data2[i].bus_i) << "Result index do not match.";

    diff = std::abs(data1[i].Vm - data2[i].Vm);
    EXPECT_LE(diff, tol) << "Result Vm differ at index " << i;

    diff = std::abs(data1[i].Va - data2[i].Va);
    EXPECT_LE(diff, tol) << "Result Va differ at index " << i;

    return;
  }
}