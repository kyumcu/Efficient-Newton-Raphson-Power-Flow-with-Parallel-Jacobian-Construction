#pragma once

#include "Utils/Naming.h"

//#include <Eigen/IterativeLinearSolvers>
//#include <Eigen/OrderingMethods>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include <iostream>

template<typename T, int Options>
bool
isRowMajor(const Eigen::SparseMatrix<T, Options>& matrix)
{
  return Options & Eigen::RowMajor;
}

class EigenSparseLUSolver
{
private:
  bool is_set = false;
  Eigen::SparseLU<Eigen::SparseMatrix<double>> _solver; // Eigen SparseLU solver
public:
  // Constructor that initializes the matrix and vectors
  EigenSparseLUSolver() {}

  // Analyze the pattern of the Jacobian matrix
  void init(Eigen::SparseMatrix<double>& A)
  {
    if (!is_set)
     
    is_set = true;
  }

  // Perform the numerical factorization
  void setup(Eigen::SparseMatrix<double>& A)
  {
    _solver.analyzePattern(A);
    _solver.factorize(A);
    if (_solver.info() != Eigen::Success) {
      std::cerr << "Factorization failed." << std::endl;
    }
  }

  // Solve the equation using the computed decomposition
  void solve(EigenRDVType<double>& b, EigenRDVType<double>& x) { x = _solver.solve(b); }
};

class EigenBiCGSTABSolver
{
private:
  bool is_set = false;
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>, Eigen::IncompleteLUT<double>>
    _solver;

public:
  // Constructor that initializes the matrix and vectors
  EigenBiCGSTABSolver() {}

  // Analyze the pattern of the Jacobian matrix
  void init(Eigen::SparseMatrix<double, Eigen::RowMajor>& A)
  {
    if (!is_set) {
      _solver.preconditioner().setDroptol(0.000001); // Set the drop tolerance
      _solver.preconditioner().setFillfactor(40);

      _solver.compute(A);
    }
    is_set = true;
  }

  // Perform the numerical factorization
  void setup(Eigen::SparseMatrix<double, Eigen::RowMajor>& A)
  {
    if (_solver.info() != Eigen::Success) {
      std::cerr << "Factorization failed." << std::endl;
    }
  }

  // Solve the equation using the computed decomposition
  void solve(EigenRDVType<double>& b, EigenRDVType<double>& x) { x = _solver.solve(b); }
};

class EigenBiCGSTABSolver_diag
{
private:
  bool is_set = false;
  Eigen::BiCGSTAB<Eigen::SparseMatrix<double, Eigen::RowMajor>,
                  Eigen::DiagonalPreconditioner<double>>
    _solver;

public:
  // Constructor that initializes the matrix and vectors
  EigenBiCGSTABSolver_diag() {}

  // Analyze the pattern of the Jacobian matrix
  void init(Eigen::SparseMatrix<double, Eigen::RowMajor>& A)
  {
    if (!is_set) {
    }
    is_set = true;
  }

  // Perform the numerical factorization
  void setup(Eigen::SparseMatrix<double, Eigen::RowMajor>& A)
  {
    _solver.compute(A);
    if (_solver.info() != Eigen::Success) {
      std::cerr << "Factorization failed." << std::endl;
    }
  }

  // Solve the equation using the computed decomposition
  void solve(EigenRDVType<double>& b, EigenRDVType<double>& x) { x = _solver.solve(b); }
};