#pragma once
#include <iostream> // delete this
#include <omp.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <iostream>
#include <vector>

#include "Device_models/Conductor.h"
#include "Device_models/Transformer.h"
#include "Naming.h"

template<typename T, typename U>
using v_sp_connector = v_sp_<Conductor<T, U>>;
template<typename T, typename U>
using v_sp_transformer = v_sp_<Transformer<T, U>>;

template<typename T, typename U>
Eigen::SparseMatrix<std::complex<T>, Eigen::RowMajor>
Y_BusGenerate(v_sp_connector<T, U>& connection_list,
              v_sp_transformer<T, U>& _transformer_list,
              v_sp_connector<T, U>& ground_connection_list,
              size_t num_nodes)
{

  size_t num_links = connection_list.size();
  size_t num_ground_links = ground_connection_list.size();
  size_t num_transformer_links = _transformer_list.size();

  Eigen::SparseMatrix<std::complex<T>, Eigen::RowMajor> Y_bus(num_nodes, num_nodes);

  std::vector<Eigen::Triplet<std::complex<T>>> combinedTriplets; // Pre allocate with 0

#pragma omp parallel shared(combinedTriplets)
  {
    // private accumulation vector for row,col,value
    std::vector<Eigen::Triplet<std::complex<T>>> v_data; // ToDo: Pre allocate aprox size
#pragma omp for
    for (size_t i = 0; i < num_links; i++) {
      auto connection = connection_list[i];
      if (!connection->isActive())
        continue;
      auto terminalIndexes = connection->getTerminalIndexes();
      v_data.emplace_back(terminalIndexes[0],
                          terminalIndexes[0],
                          std::complex<T>(connection->getTerminalConductance(),
                                          connection->getTerminalSusceptance()));
      v_data.emplace_back(terminalIndexes[1],
                          terminalIndexes[1],
                          std::complex<T>(connection->getTerminalConductance(),
                                          connection->getTerminalSusceptance()));
      v_data.emplace_back(terminalIndexes[0],
                          terminalIndexes[1],
                          std::complex<T>(-connection->getTerminalConductance(),
                                          -connection->getTerminalSusceptance()));
      v_data.emplace_back(terminalIndexes[1],
                          terminalIndexes[0],
                          std::complex<T>(-connection->getTerminalConductance(),
                                          -connection->getTerminalSusceptance()));
    }
#pragma omp critical
    combinedTriplets.insert(combinedTriplets.end(), v_data.begin(), v_data.end());

    // private accumulation vector for row,col,value
    std::vector<Eigen::Triplet<std::complex<T>>> v_ground_data; // Pre allocate aprox size
#pragma omp for
    for (size_t i = 0; i < num_ground_links; i++) {
      auto connection = ground_connection_list[i];
      if (!connection->isActive())
        continue;
      auto terminalIndexes = connection->getTerminalIndexes();
      v_ground_data.emplace_back(terminalIndexes[0],
                                 terminalIndexes[0],
                                 std::complex<T>(connection->getTerminalConductance(),
                                                 connection->getTerminalSusceptance()));
    }
#pragma omp critical
    combinedTriplets.insert(combinedTriplets.end(), v_ground_data.begin(), v_ground_data.end());

    std::vector<Eigen::Triplet<std::complex<T>>> v_transformer_data; // Pre allocate aprox size
#pragma omp for
    for (size_t i = 0; i < num_transformer_links; i++) {
      auto transformer = _transformer_list[i];
      if (!transformer->isActive())
        continue;
      auto terminalIndexes = transformer->getTerminalIndexes();
      auto ratio = transformer->getRatio();
      v_transformer_data.emplace_back(
        terminalIndexes[0],
        terminalIndexes[0],
        std::complex<T>(transformer->getTerminalConductance(),
                        transformer->getTerminalSusceptance()) / // j,i
          std::norm(ratio));
      v_transformer_data.emplace_back(terminalIndexes[1],
                                      terminalIndexes[1],
                                      std::complex<T>(transformer->getTerminalConductance(),
                                                      transformer->getTerminalSusceptance()));
      v_transformer_data.emplace_back(terminalIndexes[0],
                                      terminalIndexes[1],
                                      std::complex<T>(transformer->getTerminalConductance(),
                                                      transformer->getTerminalSusceptance()) /
                                        -std::conj(ratio) // use the conjugate for complex ratios
      );
      v_transformer_data.emplace_back(terminalIndexes[1],
                                      terminalIndexes[0],
                                      std::complex<T>(transformer->getTerminalConductance(),
                                                      transformer->getTerminalSusceptance()) /
                                        -ratio);
    }
#pragma omp critical
    combinedTriplets.insert(
      combinedTriplets.end(), v_transformer_data.begin(), v_transformer_data.end());

  } // end pragma omp parallel shared(combinedTriplets)

  // Build the matrix from triplets
  Y_bus.setFromTriplets(combinedTriplets.begin(), combinedTriplets.end());

  return Y_bus;
}