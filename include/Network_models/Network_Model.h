#pragma once

#include "Utils/Solvers.h"

#include <omp.h>

#include <type_traits>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/OrderingMethods>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "Device_models/Conductor.h"
#include "Device_models/Transformer.h"
#include "Network_Node.h"
#include "Utils/BasicMethods.h"
#include "Utils/Naming.h"
#include "Utils/Y_BusGenerator.h"

#include <chrono>

#ifdef MKL
#include <Eigen/PardisoSupport>
#endif

// Define an enum for Bus Type
enum class StateType
{
  Voltage_rms,
  Phase_angle,
  Power_flow_active,
  Power_flow_reactive,
  COUNT
};

template<typename T, typename U>
class GridNetwork
{
  friend class NetworkNode<T, U>;

  using TU_Node = NetworkNode<T, U>;
  using TU_Conductor = Conductor<T, U>;
  using TU_Transformer = Transformer<T, U>;

public:
  std::unique_ptr<EigenSparseLUSolver> _solver;

  std::vector<std::shared_ptr<TU_Node>> _NetworkNodes; // Vector holding Nodes
  std::vector<std::shared_ptr<TU_Node>> _NetworkNodes_PQ;
  std::vector<std::shared_ptr<TU_Node>> _NetworkNodes_PV;
  std::vector<std::shared_ptr<TU_Node>> _NetworkNodes_S;

  Eigen::SparseMatrix<std::complex<T>, Eigen::RowMajor> _Y_Bus;
  Eigen::SparseMatrix<T, Eigen::RowMajor> _Jacobian;

  EigenRDVType<T> _PnQ;
  EigenRDVType<T> _discrepancy;
  EigenRDVType<T> _deltaX;

  // Vector holding Node data in vector form
  std::vector<T> _voltage_rms;
  std::vector<T> _phase_angle;
  std::vector<T> _power_flow_active;
  std::vector<T> _power_flow_reactive;
  std::vector<T>* _statePointers[static_cast<size_t>(StateType::COUNT)];

  std::vector<std::shared_ptr<TU_Conductor>> _device_list;
  std::vector<std::shared_ptr<TU_Transformer>> _transformer_list;
  std::vector<std::shared_ptr<TU_Conductor>> _device_to_ground_list;

  std::shared_ptr<TU_Node> _ground_node;

  std::vector<size_t> _legacy2index;
  std::unordered_map<int, size_t> _tag2legacy;
  std::vector<int> _reverse_map;

public:
  std::string _caseName;

public:
  GridNetwork()
  {
    _solver = std::make_unique<EigenSparseLUSolver>();
    // Initialize the array of pointers to member vectors
    _statePointers[0] = &_voltage_rms;
    _statePointers[1] = &_phase_angle;
    _statePointers[2] = &_power_flow_active;
    _statePointers[3] = &_power_flow_reactive;

    _ground_node =
      std::make_shared<TU_Node>(-1, 0, 0.0, 0.0, 0.0, BusType::PV); // move this to Network
  }

  void addNode(const std::shared_ptr<TU_Node>& node)
  {
    _NetworkNodes.push_back(node);
  } // check if node index and Nodes index are same
  void addConnection(const std::shared_ptr<TU_Conductor>& conductor)
  {
    _device_list.push_back(conductor);
  } // Generalize for generic device addition
  void addTransformer(const std::shared_ptr<TU_Transformer>& transformer)
  {
    _transformer_list.push_back(transformer);
  } // Generalize for generic device addition
  void addGroundConnection(const std::shared_ptr<TU_Conductor>& conductor)
  {
    _device_to_ground_list.push_back(conductor);
  } // Generalize for generic device addition

  void resize() // make this reseerve
  {
    size_t new_size = _NetworkNodes.size();
    _voltage_rms.resize(new_size);
    _phase_angle.resize(new_size);
    _power_flow_active.resize(new_size);
    _power_flow_reactive.resize(new_size);

    _PnQ.resize(
      _NetworkNodes_PV.size() +
      2 * _NetworkNodes_PQ.size()); // Make sure that   orderANDClasifyNodes() is called before
  }

  void pullExtern(const std::vector<T>& extern_data, StateType state_type)
  {
    *(_statePointers[static_cast<size_t>(state_type)]) = extern_data;
  } // Directly access the appropriate vector using the pointer

  std::shared_ptr<TU_Node>& getNode(const std::size_t index) { return _NetworkNodes[index]; }

  std::shared_ptr<TU_Node>& getGNode() { return _ground_node; }

  void setNetworkNodes(std::vector<std::shared_ptr<TU_Node>>&& NetworkNodes)
  {
    _NetworkNodes = std::move(NetworkNodes);
  }

  T getData(const std::size_t index, StateType state_type)
  {
    return (*(_statePointers[static_cast<size_t>(state_type)]))[index];
  }

  std::complex<T> getY_BusAt(size_t i, size_t j) { return _Y_Bus.coeff(i, j); }

  const std::vector<T>& getDataVector(StateType state_type) const
  {
    return *(_statePointers[static_cast<size_t>(state_type)]);
  }

  const EigenRDVType<T> getPnQ() const { return _PnQ; }
  const EigenRDVType<T> getDiscrepancy() const { return _discrepancy; }
  const EigenRDVType<T> getDeltaX() const { return _deltaX; }

  void setLegacy2Index(std::vector<size_t>&& new_index_map)
  {
    _legacy2index = std::move(new_index_map);
  }
  void setTag2legacy(std::unordered_map<int, size_t>&& new_tag2index)
  {
    _tag2legacy = std::move(new_tag2index);
  }
  void setReverse_map(std::vector<int>&& new_reverse_map)
  {
    _reverse_map = std::move(new_reverse_map);
  }

  const std::vector<size_t>& getIndex2legacy() const { return _legacy2index; }
  const std::unordered_map<int, size_t>& getTag2legacy() const { return _tag2legacy; }
  const std::vector<int>& getReverse_map() const { return _reverse_map; }

  // for debug only
  const Eigen::SparseMatrix<T, Eigen::RowMajor> getJac() const { return _Jacobian; }
  const Eigen::SparseMatrix<std::complex<T>, Eigen::RowMajor> getYBus() const { return _Y_Bus; }

  void pushToNodes_V()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _NetworkNodes[i]->setVoltageRms(_voltage_rms[i]);
    }
  }
  void pushToNodes_Ph()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _NetworkNodes[i]->setPhaseAngle(_phase_angle[i]);
    }
  }
  void pushToNodes_AP()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _NetworkNodes[i]->setPowerFlowActive(_power_flow_active[i]);
    }
  }
  void pushToNodes_RP()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _NetworkNodes[i]->setPowerFlowReactive(_power_flow_reactive[i]);
    }
  }

  void pullFromNodes_V()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _voltage_rms[i] = _NetworkNodes[i]->getVoltageRms();
    }
  }
  void pullFromNodes_Ph()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _phase_angle[i] = _NetworkNodes[i]->getPhaseAngle();
    }
  }

  void pullFromNodes_AP()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _power_flow_active[i] = _NetworkNodes[i]->getPowerFlowActive();
    }
  }

  void pullFromNodes_RP()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _power_flow_reactive[i] = _NetworkNodes[i]->getPowerFlowReactive();
    }
  }

  void pullFromNodes_P()
  { // be carefull for cache line false sharing
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes_PV.size(); i++) {
      size_t index_global = _NetworkNodes_PV[i]->getNodeIndex();
      _PnQ[i] = _NetworkNodes[index_global]->getP();
    }
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes_PQ.size(); i++) {
      size_t index_global = _NetworkNodes_PQ[i]->getNodeIndex();
      _PnQ[i + _NetworkNodes_PV.size()] = _NetworkNodes[index_global]->getP();
    }
  }

  void pullFromNodes_Q()
  {
    //#pragma omp parallel for
    for (size_t i = 0; i < _NetworkNodes_PQ.size(); i++) {
      size_t index_global = _NetworkNodes_PQ[i]->getNodeIndex();
      _PnQ[i + _NetworkNodes_PV.size() + _NetworkNodes_PQ.size()] =
        _NetworkNodes[index_global]->getQ();
    }
  }

  void pullDefinedFromNodes()
  { // disregrds ground node
    //#pragma omp parallel for // there is no data race but bucket the taskss to
    // prevent cash line false sharing
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _voltage_rms[_NetworkNodes[i]->getNodeIndex()] = _NetworkNodes[i]->getVoltageRms();
      _phase_angle[_NetworkNodes[i]->getNodeIndex()] = _NetworkNodes[i]->getPhaseAngle();
      _power_flow_active[_NetworkNodes[i]->getNodeIndex()] = _NetworkNodes[i]->getPowerFlowActive();
      _power_flow_reactive[_NetworkNodes[i]->getNodeIndex()] =
        _NetworkNodes[i]->getPowerFlowReactive();
    }
  }

  void localizeDeviceList()
  {
#pragma omp parallel
    {
#pragma omp for
      for (size_t i = 0; i < _device_list.size(); i++) {
        auto device_terminals = _device_list[i]->getTerminalIndexes();
        _NetworkNodes[device_terminals[0]]->addDevice(_device_list[i]);
        _NetworkNodes[device_terminals[1]]->addDevice(_device_list[i]);
      }
#pragma omp for
      for (size_t i = 0; i < _transformer_list.size(); i++) {
        auto device_terminals = _transformer_list[i]->getTerminalIndexes();
        _NetworkNodes[device_terminals[0]]->addDevice(_transformer_list[i]);
        _NetworkNodes[device_terminals[1]]->addDevice(_transformer_list[i]);
      }
    }
  }

  void generateNodeNeighborsList()
  {
#pragma omp parallel
#pragma omp single
#pragma omp taskloop num_tasks(BATCHSIZE)
    for (size_t i = 0; i < _NetworkNodes.size(); i++) {
      _NetworkNodes[i]->fillNodeData(*this);
    }
  }

  void computeLocalPower()
  {
#pragma omp parallel
#pragma omp single
#pragma omp taskloop num_tasks(BATCHSIZE)
    for (size_t i = 0; i < _NetworkNodes_PV.size(); i++) {
      _NetworkNodes_PV[i]->computePnQ(*this);
    }

#pragma omp parallel
#pragma omp single
#pragma omp taskloop num_tasks(BATCHSIZE)
    for (size_t i = 0; i < _NetworkNodes_PQ.size(); i++) {
      _NetworkNodes_PQ[i]->computePnQ(*this);
    }
  }

  void computeY_Bus()
  {
    _Y_Bus = Y_BusGenerate<double, double>(
      _device_list, _transformer_list, _device_to_ground_list, _NetworkNodes.size());
  }

  void constructDiscrepancyVec()
  {

    size_t num_PQ = _NetworkNodes_PQ.size();
    size_t num_PV = _NetworkNodes_PV.size();

    EigenRDVType<T> power_flow(num_PV + 2 * num_PQ);

    size_t num_non_slack = num_PV + num_PQ;

    Eigen::Map<Eigen::VectorXd> M_active(_power_flow_active.data(), _power_flow_active.size());
    Eigen::Map<Eigen::VectorXd> M_reactive(_power_flow_reactive.data(),
                                           _power_flow_reactive.size());

    power_flow.head(num_non_slack) = M_active.tail(num_non_slack);
    power_flow.tail(num_PQ) = M_reactive.tail(num_PQ);

    _discrepancy = power_flow - _PnQ;
  }

  void CreateEmptyJacobian();
  void UpdateJacobian();

  void solvePF()
  {
    // Eigen::ConjugateGradient<Eigen::SparseMatrix<double>,
    //                          Eigen::Lower | Eigen::Upper>
    //   solver;
    //

    /// solver.compute(_Jacobian);


  Eigen::SparseMatrix<double, Eigen::ColMajor> colMajorJacobian = _Jacobian;

#ifdef MKL
    Eigen::PardisoLU<Eigen::SparseMatrix<T, Eigen::RowMajor>>
      solver; // note the use of SparseMatrixR
    solver.compute(_Jacobian);
#else

#ifdef TIME2
    {
      auto start = std::chrono::high_resolution_clock::now();
#endif
      _solver->init(colMajorJacobian);

#ifdef TIME2
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed = end - start;
      std::cout << "'init': " << elapsed.count() << ",\n";
    }
#endif
#ifdef TIME2
    {
      auto start = std::chrono::high_resolution_clock::now();
#endif
      _solver->setup(colMajorJacobian);
#ifdef TIME2
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed = end - start;
      std::cout << "'setup': " << elapsed.count() << ",\n";
    }
#endif

#endif
#ifdef TIME2
    {
      auto start = std::chrono::high_resolution_clock::now();
#endif
      _solver->solve(_discrepancy, _deltaX);
#ifdef TIME2
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed = end - start;
      std::cout << "'solve': " << elapsed.count() << ",\n";
    }
#endif
  }

  void correctPhaseandVoltage();

  void orderANDClasifyNodes();
  void prepeare_PF();
  void do_PF();

  // Utils
  bool writeReverse_mapToCSV();
  bool writeY_busToCSV();
  bool writeJacToCSV();
  bool writeVoltagePhToCSV();
  void PrintNetworkStats();
};
