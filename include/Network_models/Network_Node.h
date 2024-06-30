#pragma once

#include "Device_models/Device_base.h"
#include "Utils/Naming.h"

#include <cmath> // Include for sin, cos, and M_PI
#include <memory>
#include <vector>

// Forward decleration
template<typename T, typename U>
class GridNetwork;

template<typename T>
struct NodeData
{
  NodeData(T index, T node_voltage, T node_delta_phase)
    : _index(index)
    , _voltage(node_voltage)
  {
    _sin_phase = std::sin(node_delta_phase);
    _cos_phase = std::cos(node_delta_phase);
  }
  T _index;
  T _voltage;
  T _sin_phase;
  T _cos_phase;
};

template<typename T, typename U>
class NetworkNode
{
  using TU_Device = Device<T, U>;

public:
  int tag = -1;
  int node_index = -1;
  T voltage_rms = 0.0;
  T phase_angle = 0.0;
  T power_flow_active = 0.0;
  T power_flow_reactive = 0.0;
  T P = 0.0;
  T Q = 0.0;
  BusType type = BusType::PQ;
  bool _GenConnected = false;

  std::vector<std::shared_ptr<TU_Device>> node_devices; // Vector holding Device objects

  std::vector<NodeData<T>> _node_data;

  VT_<T> _Jac_triplets_ph;
  VT_<T> _Jac_triplets_V;

  // Constructor (optional if only default initialization is used)
  NetworkNode() = default;

  // Constructor with index value
  explicit NetworkNode(int node_index)
    : node_index(node_index)
  {
  }

  // Constructor with full values
  NetworkNode(int node_index,
              T voltage_rms,
              T phase_angle,
              T power_flow_active,
              T power_flow_reactive,
              BusType type)
    : node_index(node_index)
    , voltage_rms(voltage_rms)
    , phase_angle(phase_angle)
    , power_flow_active(power_flow_active)
    , power_flow_reactive(power_flow_reactive)
    , type(type)
  {
  }

  // Constructor with full values
  NetworkNode(int tag,
              int node_index,
              T voltage_rms,
              T phase_angle,
              T power_flow_active,
              T power_flow_reactive,
              BusType type)
    : NetworkNode(node_index,
                  voltage_rms,
                  phase_angle,
                  power_flow_active,
                  power_flow_reactive,
                  type)
  {
    this->tag = tag;
  }

  int getNodeTag() const { return tag; }
  // Node Index
  void setNodeIndex(int index) { node_index = index; }
  int getNodeIndex() const { return node_index; }

  // Voltage RMS
  void setVoltageRms(T voltage) { voltage_rms = voltage; }
  T getVoltageRms() const { return voltage_rms; }

  // Phase Angle
  void setPhaseAngle(T angle) { phase_angle = angle; }
  T getPhaseAngle() const { return phase_angle; }

  // Power Flow Active
  void setPowerFlowActive(T powerActive) { power_flow_active = powerActive; }
  T getPowerFlowActive() const { return power_flow_active; }

  // Power Flow Reactive
  void setPowerFlowReactive(T powerReactive) { power_flow_reactive = powerReactive; }
  T getPowerFlowReactive() const { return power_flow_reactive; } // Power Flow Active

  T getP() const { return P; }

  // Power Flow Reactive
  T getQ() const { return Q; }

  // Type
  void setType(BusType busType) { type = busType; }
  BusType getType() const { return type; }

  void setGenConnected(bool connected) { _GenConnected = connected; }
  bool isGenConnected() const { return _GenConnected; }

  // Connectivity data
  size_t getConnectivity() const { return _node_data.size() - 1; }

  // Method to add a device to the circuit
  void addDevice(const std::shared_ptr<TU_Device>& device)
  {
#pragma omp critical
    node_devices.push_back(device);
  }

  void fillNodeData(GridNetwork<T, U>& Network);
  void CreateJacLineSignature_PQ(GridNetwork<T, U>& Network, VT_<T>& J);
  void CreateJacLineSignature_PV(GridNetwork<T, U>& Network, VT_<T>& J);

  void computePnQ(GridNetwork<T, U>& Network);

  void computeJ_PQ(GridNetwork<T, U>& Network);
  void computeJ_PV(GridNetwork<T, U>& Network);

  void updateJ_PQ(GridNetwork<T, U>& Network);
  void updateJ_PV(GridNetwork<T, U>& Network);
};