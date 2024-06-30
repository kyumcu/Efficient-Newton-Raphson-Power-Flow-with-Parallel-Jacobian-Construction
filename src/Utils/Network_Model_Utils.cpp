#include <omp.h>

#include "Network_models/Network_Model.h"
#include "Utils/BasicMethods.h"
#include "Utils/DebugTools.h"
#include "Utils/Naming.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

template<typename T, typename U>
bool
GridNetwork<T, U>::writeReverse_mapToCSV()
{
  // Open a file in write mode
  std::string path = std::string(OUTDIR) + _caseName + "_reverse_map.csv";
  std::cout << "reverse_map:::" << path << std::endl;
  std::ofstream file(path);

  if (!file.is_open()) {
    std::cerr << "Failed to open file for writing.\n";
    return 1;
  }

  // Write the CSV header
  file << "Index,Tag,_Tag,Type,Connections,Legacy_index\n";

  // Iterate over the non-zero elements of the sparse matrix
  for (size_t i = 0; i < _reverse_map.size(); ++i) {
    file << i << ","                                   //
         << _reverse_map[i] << ","                     //
         << _NetworkNodes[i]->getNodeTag() << ","      //
         << _NetworkNodes[i]->getType() << ","         //
         << _NetworkNodes[i]->getConnectivity() << "," //
         << _tag2legacy[_reverse_map[i]] << "\n";
  }

  file.close(); // Close the file

  return 0;
}

template<typename T, typename U>
bool
GridNetwork<T, U>::writeY_busToCSV()
{

  // Open a file in write mode
  std::string path = std::string(OUTDIR) + _caseName + "_Ybus.csv";
  std::cout << "Y_BUS PATH:::" << path << std::endl;
  std::ofstream file(path);

  int slack_ofset = _NetworkNodes_S.size();

  if (!file.is_open()) {
    std::cerr << "Failed to open file for writing.\n";
    return 1;
  }

  // Write the CSV header
  file << "Row,Column,Real,Imaginary\n";

  // Iterate over the non-zero elements of the sparse matrix
  for (int k = 0; k < _Y_Bus.outerSize(); ++k) {
    for (typename Eigen::SparseMatrix<std::complex<T>, Eigen::RowMajor>::InnerIterator it(_Y_Bus,
                                                                                          k);
         it;
         ++it) {
      file << _tag2legacy[_reverse_map[it.row()]] << "," // un_ordered indexes
           << _tag2legacy[_reverse_map[it.col()]] << "," //
           << it.value().real()                          //
           << "," << it.value().imag() << "\n";
    }
  }

  file.close(); // Close the file

  return 0;
}

template<typename T, typename U>
bool
GridNetwork<T, U>::writeJacToCSV()
{
  // Open a file in write mode
  std::string path = std::string(OUTDIR) + _caseName + "_Jac.csv";
  std::cout << "Jac PATH:::" << path << std::endl;
  std::ofstream file(path);

  int slack_ofset = _NetworkNodes_S.size();

  if (!file.is_open()) {
    std::cerr << "Failed to open file for writing.\n";
    return 1;
  }

  // Write the CSV header
  file << "Row,Column,Real\n";

  // Iterate over the non-zero elements of the sparse matrix
  for (int k = 0; k < _Jacobian.outerSize(); ++k) {
    for (typename Eigen::SparseMatrix<T, Eigen::RowMajor>::InnerIterator it(_Jacobian, k); it;
         ++it) {
      file << it.row() << "," << it.col() << "," << it.value() << "\n";
    }
  }

  file.close(); // Close the file

  return 0;
}

template<typename T, typename U>
bool
GridNetwork<T, U>::writeVoltagePhToCSV()
{
  // Open a file in write mode
  std::string path = std::string(OUTDIR) + _caseName + ".csv";
  std::cout << "RES PATH:::" << path << std::endl;
  std::ofstream file(path);

  if (!file.is_open()) {
    std::cerr << "Failed to open file for writing.\n";
    return 1;
  }

  // Write the CSV header
  file << "bus_i,Vm,Va\n";

  auto Vm = getDataVector(StateType::Voltage_rms);
  auto Va = getDataVector(StateType::Phase_angle);

  for (size_t i = 0; i < _NetworkNodes.size();
       i++) {                                     // reverse_map[index_map[tag2index[tag]]] = tag
    file << _reverse_map[_legacy2index[i]] << "," //
         << Vm[_legacy2index[i]] << ","           //
         << radiansToDeg(Va[_legacy2index[i]]) << "\n";
  }

  file.close(); // Close the file

  return 0;
}

template<typename T, typename U>
void
GridNetwork<T, U>::PrintNetworkStats()
{

  std::string line(30, '-');

  std::cout << line << std::endl;

  std::cout << std::left << std::setw(5) << "CASE: " << std::right << std::setw(5) << _caseName
            << std::endl;

  std::cout << line << std::endl;

  std::cout << std::left << std::setw(15) << "Num nodes: " << std::right << std::setw(5)
            << _NetworkNodes.size() << std::endl;

  std::cout << std::left << std::setw(15) << "Num Slack: " << std::right << std::setw(5)
            << _NetworkNodes_S.size() << std::endl;

  std::cout << std::left << std::setw(15) << "Num Sources: " << std::right << std::setw(5)
            << _NetworkNodes_PV.size() << std::endl;

  std::cout << std::left << std::setw(15) << "Num Loads: " << std::right << std::setw(5)
            << _NetworkNodes_PQ.size() << std::endl;

  std::cout << line << std::endl;

  std::cout << std::left << std::setw(15) << "Branches: " << std::right << std::setw(5)
            << _device_list.size() << std::endl;

  std::cout << std::left << std::setw(15) << "Transformers: " << std::right << std::setw(5)
            << _transformer_list.size() << std::endl;

  std::cout << std::left << std::setw(15) << "Shunts: " << std::right << std::setw(5)
            << _device_to_ground_list.size() << std::endl;

  std::cout << line << std::endl;
  generateNodeNeighborsList();
  std::cout << std::left << std::setw(5) << "Connectivity Stats: " << std::endl;

  double max_g = 0, min_g = 99, mean_g = 0, //
    max_v = 0, min_v = 99, mean_v = 0,      //
    max_q = 0, min_q = 99, mean_q = 0;

  double num_connections = 0, sum_connections = 0;

  for (size_t i = 0; i < _NetworkNodes_S.size(); i++) {
    num_connections = _NetworkNodes_S[i]->getConnectivity();
    if (num_connections > max_g)
      max_g = num_connections;
    if (num_connections < min_g)
      min_g = num_connections;
    sum_connections += num_connections;
  }
  mean_g += sum_connections;

  sum_connections = 0;
  for (size_t i = 0; i < _NetworkNodes_PQ.size(); i++) {
    num_connections = _NetworkNodes_PQ[i]->getConnectivity();
    if (num_connections > max_q)
      max_q = num_connections;
    if (num_connections < min_q)
      min_q = num_connections;
    sum_connections += num_connections;
  }
  mean_q = sum_connections / _NetworkNodes_PQ.size();
  mean_g += sum_connections;

  sum_connections = 0;
  for (size_t i = 0; i < _NetworkNodes_PV.size(); i++) {
    num_connections = _NetworkNodes_PV[i]->getConnectivity();
    if (num_connections > max_v)
      max_v = num_connections;
    if (num_connections < min_v)
      min_v = num_connections;
    sum_connections += num_connections;
  }
  mean_v = sum_connections / _NetworkNodes_PV.size();
  mean_g += sum_connections;

  if (max_q > max_g)
    max_g = max_q;
  if (max_v > max_g)
    max_g = max_v;

  if (min_q < min_g)
    min_g = min_q;
  if (min_v < min_g)
    min_g = min_v;

  mean_g /= _NetworkNodes.size();

  std::cout << std::fixed << std::setprecision(3);
  std::cout << std::left << std::setw(15) << "All Nodes"                         //
            << std::setw(5) << "| mean:" << std::right << std::setw(5) << mean_g //
            << std::setw(5) << "| max:" << std::right << std::setw(5) << max_g   //
            << std::setw(5) << "| min:" << std::right << std::setw(5) << min_g   //
            << std::setw(5) << "| " << std::endl;

  std::cout << std::left << std::setw(15) << "PV Nodes"                          //
            << std::setw(5) << "| mean:" << std::right << std::setw(5) << mean_v //
            << std::setw(5) << "| max:" << std::right << std::setw(5) << max_v   //
            << std::setw(5) << "| min:" << std::right << std::setw(5) << min_v   //
            << std::setw(5) << "| " << std::endl;

  std::cout << std::left << std::setw(15) << "PQ Nodes"                          //
            << std::setw(5) << "| mean:" << std::right << std::setw(5) << mean_q //
            << std::setw(5) << "| max:" << std::right << std::setw(5) << max_q   //
            << std::setw(5) << "| min:" << std::right << std::setw(5) << min_q   //
            << std::setw(5) << "| " << std::endl;

  std::cout << line << std::endl;
}

// Template Instantiation
template class GridNetwork<double, double>;