#pragma once

#include "Utils/Naming.h"

#include <cmath>
#include <complex>
#include <iostream>
#include <string>

template<typename T>
inline std::complex<T>
calculateAdmittance(const std::complex<T> impedance)
{
  return 1.0 / impedance;
}

template<typename T>
void
printVector(const EigenRDVType<T>& vec, const std::string& name = "")
{
  std::cout << name << ": ";
  for (int i = 0; i < vec.size(); ++i) {
    std::cout << vec(i) << " ";
  }
  std::cout << std::endl;
}

template<typename T>
void
printVector(const std::vector<Eigen::Triplet<T>>& triplets, const std::string& name = "")
{
  std::cout << name << " (row, col, value):" << std::endl;
  for (const auto& triplet : triplets) {
    std::cout << "(" << triplet.row() << ", " << triplet.col() << ", " << triplet.value() << ")"
              << std::endl;
  }
}

template<typename T>
void
printVector(const std::vector<T>& vec, std::string name = "")
{
  Eigen::Map<const EigenRDVType<T>> eigenVecMapped(vec.data(), vec.size());
  printVector<T>(eigenVecMapped, name);
}

template<typename T>
void
printSparseMatrix(const Eigen::SparseMatrix<T, Eigen::RowMajor>& matrix,
                  const std::string& name = "")
{
  std::cout << name << ":\n";

  // Iterate over each row (outer loop)
  for (int k = 0; k < matrix.outerSize(); ++k) {
    // Iterate over each non-zero element in the row (inner loop)
    for (typename Eigen::SparseMatrix<T, Eigen::RowMajor>::InnerIterator it(matrix, k); it; ++it) {
      std::cout << "(" << it.row() << "," << it.col() << "): " << it.value() << "\n";
    }
  }

  std::cout << std::endl;
}

template<typename T>
T
radiansToDeg(const T rad)
{
  return rad * 180.0 / PI;
}

template<typename T>
T
degToRadians(const T deg)
{
  return deg * PI / 180.0;
}

template<typename T>
std::complex<T>
polarDegToCart(const T mag, const T deg)
{
  T rad = degToRadians(deg);

  return std::complex<T>(mag * cos(rad), mag * sin(rad));
}