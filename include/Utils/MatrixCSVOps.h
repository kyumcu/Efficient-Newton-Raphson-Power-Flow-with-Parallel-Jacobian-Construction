#pragma once

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <complex>
#include <fstream>
#include <sstream>
#include <stdexcept> // For std::runtime_error
#include <string>
#include <type_traits>
#include <vector>

template<typename T>
void
writeDenseMatrixToCSV(const std::string& filename,
                      const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>& matrix)
{
  std::ofstream file(filename);

  // Header
  if constexpr (std::is_same<T, std::complex<double>>::value) {
    file << "Row,Column,Real,Imaginary\n";
  } else {
    file << "Row,Column,Real\n";
  }

  for (int i = 0; i < matrix.rows(); ++i) {
    for (int j = 0; j < matrix.cols(); ++j) {
      file << i << "," << j;
      if constexpr (std::is_same<T, std::complex<double>>::value) {
        file << "," << matrix(i, j).real() << "," << matrix(i, j).imag();
      } else {
        file << "," << matrix(i, j);
      }
      file << "\n";
    }
  }
}

template<typename T>
void
writeSparseMatrixToCSV(const std::string& filename, const Eigen::SparseMatrix<T>& matrix)
{
  std::ofstream file(filename);

  // Header
  if constexpr (std::is_same<T, std::complex<double>>::value) {
    file << "Row,Column,Real,Imaginary\n";
  } else {
    file << "Row,Column,Real\n";
  }

  for (int k = 0; k < matrix.outerSize(); ++k) {
    for (typename Eigen::SparseMatrix<T>::InnerIterator it(matrix, k); it; ++it) {
      file << it.row() << "," << it.col();
      if constexpr (std::is_same<T, std::complex<double>>::value) {
        file << "," << it.value().real() << "," << it.value().imag();
      } else {
        file << "," << it.value();
      }
      file << "\n";
    }
  }
}
