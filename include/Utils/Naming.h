#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Sparse>

const double PI = 3.14159265358979323846;

template<typename TheClass>
using v_sp_ = std::vector<std::shared_ptr<TheClass>>;

template<typename T>                            // double or else float
using EigenRDVType = typename std::conditional< // Real, Dynamic, Vector
  std::is_same<T, double>::value,
  Eigen::VectorXd, // Use Eigen::VectorXd for double
  Eigen::VectorXf  // Use Eigen::VectorXf for float
  >::type;

template<typename T>
using VT_ = std::vector<Eigen::Triplet<T>>;

// Define an enum for Bus Type
enum class BusType
{
  slack,
  PV,
  PQ
};

std::ostream&
operator<<(std::ostream& os, const BusType& busType);

BusType
intToBusType(int input);
