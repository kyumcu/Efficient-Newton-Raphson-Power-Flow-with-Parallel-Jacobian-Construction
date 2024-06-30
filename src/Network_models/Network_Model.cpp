#include <omp.h>

#include <iostream>

#include "Network_models/Network_Model.h"
#include "Utils/BasicMethods.h"
#include "Utils/DebugTools.h"
#include "Utils/Naming.h"

template<typename T, typename U>
void
GridNetwork<T, U>::CreateEmptyJacobian()
{
  size_t sys_dim = _NetworkNodes_PV.size() + 2 * _NetworkNodes_PQ.size();
  Eigen::SparseMatrix<T, Eigen::RowMajor> Jacobian(sys_dim, sys_dim);

  VT_<T> combinedTriplets;

#pragma omp parallel
  {
    VT_<T> temp;
#pragma omp for nowait
    for (size_t i = 0; i < _NetworkNodes_PV.size(); i++) {
      _NetworkNodes_PV[i]->CreateJacLineSignature_PV(*this, temp);
    }
#pragma omp critical
    combinedTriplets.insert(combinedTriplets.end(), temp.begin(), temp.end());

    VT_<T> temp_2;
#pragma omp for nowait
    for (size_t i = 0; i < _NetworkNodes_PQ.size(); i++) {
      _NetworkNodes_PQ[i]->CreateJacLineSignature_PQ(*this, temp_2);
    }
#pragma omp critical
    combinedTriplets.insert(combinedTriplets.end(), temp_2.begin(), temp_2.end());
  }

  Jacobian.setFromTriplets(combinedTriplets.begin(), combinedTriplets.end());
  _Jacobian = std::move(Jacobian);
}

template<typename T, typename U>
void
GridNetwork<T, U>::UpdateJacobian()
{
#pragma omp parallel
  {

#pragma omp for nowait
    for (size_t i = 0; i < _NetworkNodes_PV.size(); i++) {
      _NetworkNodes_PV[i]->computeJ_PV(*this);
      _NetworkNodes_PV[i]->updateJ_PV(*this);
    }

#pragma omp for nowait
    for (size_t i = 0; i < _NetworkNodes_PQ.size(); i++) {
      _NetworkNodes_PQ[i]->computeJ_PQ(*this);
      _NetworkNodes_PQ[i]->updateJ_PQ(*this);
    }
  }
  // Build the matrix from triplets
}
template<typename T, typename U>
void
GridNetwork<T, U>::correctPhaseandVoltage()
{

  size_t num_PQ = _NetworkNodes_PQ.size();
  size_t num_nodes = _NetworkNodes_PV.size() + num_PQ;

  Eigen::Map<Eigen::VectorXd> M_phase_angle(_phase_angle.data(), _phase_angle.size());
  Eigen::Map<Eigen::VectorXd> M_voltage_rms(_voltage_rms.data(), _voltage_rms.size());

  // using tail excludes slack buses as they are the first nodes
  M_phase_angle.tail(num_nodes) += _deltaX.head(num_nodes);
  M_voltage_rms.tail(num_PQ) += _deltaX.tail(num_PQ);
}

template<typename T, typename U>
void
GridNetwork<T, U>::orderANDClasifyNodes()
{
  size_t num_nodes = _NetworkNodes.size();
  v_sp_<TU_Node> ordered_list;
  ordered_list.reserve(num_nodes); // filled with pushback

  std::vector<size_t> legacy2index;
  legacy2index.resize(num_nodes);

  size_t index = 0;

  // Actual ordering of nodes, as they are added
  for (size_t i = 0; i < num_nodes; ++i) {
    if ((_NetworkNodes[i]->getType() == BusType::slack) && (_NetworkNodes[i]->isGenConnected())) {
      ordered_list.push_back(_NetworkNodes[i]);
      _NetworkNodes_S.push_back(_NetworkNodes[i]);
      legacy2index[i] = index;
      ++index;
    }
  }

  for (size_t i = 0; i < num_nodes; ++i) {
    if ((_NetworkNodes[i]->getType() == BusType::PV) && (_NetworkNodes[i]->isGenConnected())) {
      ordered_list.push_back(_NetworkNodes[i]);
      _NetworkNodes_PV.push_back(_NetworkNodes[i]);
      legacy2index[i] = index;
      ++index;
    }
  }

  for (size_t i = 0; i < num_nodes; ++i) {
    if ((_NetworkNodes[i]->getType() == BusType::PQ) || !(_NetworkNodes[i]->isGenConnected())) {
      ordered_list.push_back(_NetworkNodes[i]);
      _NetworkNodes_PQ.push_back(_NetworkNodes[i]);
      legacy2index[i] = index;
      ++index;
    }
  }

  for (size_t i = 0; i < num_nodes; ++i) {
    ordered_list[i]->setNodeIndex(i);
  }

  std::vector<int> reverse_map;
  reverse_map.resize(num_nodes);

  for (const auto& pair : _tag2legacy) {
    reverse_map[legacy2index[pair.second]] = pair.first;
    // std::cout << legacy2index[pair.second] << " to " << pair.first << std::endl;
  }

  for (size_t i = 0; i < _device_list.size(); ++i)
    _device_list[i]->updateTerminalIndexes();

  for (size_t i = 0; i < _transformer_list.size(); ++i)
    _transformer_list[i]->updateTerminalIndexes();

  for (size_t i = 0; i < _device_to_ground_list.size(); ++i)
    _device_to_ground_list[i]->updateTerminalIndexes();

  setReverse_map(std::move(reverse_map));
  setLegacy2Index(std::move(legacy2index));
  setNetworkNodes(std::move(ordered_list));
}

template<typename T, typename U>
void
GridNetwork<T, U>::prepeare_PF()
{
  orderANDClasifyNodes();
  resize();             // redo if there is "node number"
  localizeDeviceList(); // redo if there is "connection changes"
  computeY_Bus();

  std::vector<double> new_voltage(_NetworkNodes.size(), 1.0);
  std::vector<double> new_phase(_NetworkNodes.size(), 0.0);
  std::vector<double> new_acP(_NetworkNodes.size(), 0.0);
  std::vector<double> new_reP(_NetworkNodes.size(), 0.0);

  // Default values
  pullExtern(new_voltage, StateType::Voltage_rms);
  pullExtern(new_phase, StateType::Phase_angle);
  pullExtern(new_acP, StateType::Power_flow_active);
  pullExtern(new_reP, StateType::Power_flow_reactive);

  pullDefinedFromNodes(); // from initial node values

  generateNodeNeighborsList(); // Localize global data connections to
                               // node level
  CreateEmptyJacobian();
}

template<typename T, typename U>
void
GridNetwork<T, U>::do_PF()
{
#ifdef TIME
  {
    auto start = std::chrono::high_resolution_clock::now();
#endif

    generateNodeNeighborsList(); // Localize global data connections to
                                 // node level

#ifdef TIME
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "'generateNodeNeighborsList': " << elapsed.count() << ",\n";
  }
#endif

#ifdef TIME
  {
    auto start = std::chrono::high_resolution_clock::now();
#endif

    computeLocalPower();
    pullFromNodes_P(); // aggregate node data
    pullFromNodes_Q();

#ifdef TIME
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "'PnQ_compute': " << elapsed.count() << ",\n";
  }
#endif
  // for (auto i = 0; i < _NetworkNodes.size(); ++i) {
  //   std::cout << "node" << i << " P:" << _NetworkNodes[i]->getP()
  //             << " Q:" << _NetworkNodes[i]->getQ() << std::endl;

  // std::cout << "node" << i << " P:" << getData(i, StateType::Power_flow_active)
  //           << " Q:" << getData(i, StateType::Power_flow_reactive) << std::endl;
  //}
  //  for (auto i = 0; i < _PnQ.size(); ++i) {
  //  std::cout << i << " : " << _PnQ[i] << std::endl;
  //}

#ifdef TIME
  {
    auto start = std::chrono::high_resolution_clock::now();
#endif

    constructDiscrepancyVec(); // error in powerflow

#ifdef TIME
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "'constructDiscrepancyVec': " << elapsed.count() << ",\n";
  }
#endif

#ifdef TIME
  {
    auto start = std::chrono::high_resolution_clock::now();
#endif

    UpdateJacobian();

#ifdef TIME
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "'constructJacobian': " << elapsed.count() << ",\n";
  }
#endif

#ifdef TIME
  {
    auto start = std::chrono::high_resolution_clock::now();
#endif

    solvePF();

#ifdef TIME
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    std::cout << "'solvePF': " << elapsed.count() << ",\n";
  }
#endif

  correctPhaseandVoltage();
}

// Template Instantiation
template class GridNetwork<double, double>;