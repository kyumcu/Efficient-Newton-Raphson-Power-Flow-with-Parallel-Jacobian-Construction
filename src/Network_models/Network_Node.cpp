#include "Network_models/Network_Node.h"
#include "Device_models/Conductor.h"
#include "Network_models/Network_Model.h"
#include "Utils/DebugTools.h"
#include "Utils/Naming.h"

#include <algorithm> // For std::copy
#include <cmath>     // Include for sin, cos, and M_PI
#include <iostream>
#include <iterator> // For std::back_inserter

template<typename T, typename U>
void
NetworkNode<T, U>::fillNodeData(GridNetwork<T, U>& Network)
{

  std::vector<NodeData<T>> node_data;
  node_data.reserve(node_devices.size() + 1);
  std::vector<size_t> node_data_indexes(node_devices.size(), node_index);

  auto self_p = Network.getData(node_index, StateType::Phase_angle);

  node_data.emplace_back(node_index, Network.getData(node_index, StateType::Voltage_rms), (T)0);
  for (size_t i = 0; i < node_devices.size(); i++) {

    auto terminals =
      std::static_pointer_cast<Conductor<T, U>>(node_devices[i])->getTerminalIndexes();

    auto neighbor_index = terminals[1];
    if (terminals[0] != (size_t)node_index)
      neighbor_index = terminals[0];

    auto it = std::find(node_data_indexes.begin(),
                        node_data_indexes.end(),
                        neighbor_index); // is entry alredy exist

    if (it == node_data_indexes.end()) { // if not, add
      node_data.emplace_back(neighbor_index,
                             Network.getData(neighbor_index, StateType::Voltage_rms),
                             self_p - Network.getData(neighbor_index, StateType::Phase_angle));
      node_data_indexes[i] = neighbor_index;
    }
  }
  _node_data = std::move(node_data);
}

template<typename T, typename U>
void
NetworkNode<T, U>::CreateJacLineSignature_PQ(GridNetwork<T, U>& Network, VT_<T>& J)
{
  _Jac_triplets_ph.reserve(_node_data.size() * 2);
  _Jac_triplets_V.reserve(_node_data.size() * 2);

  size_t slack_offset = Network._NetworkNodes_S.size();
  size_t all_offset = Network._NetworkNodes.size();
  size_t PV_offset = Network._NetworkNodes_PV.size();
  size_t Non_PQ_offset = slack_offset + PV_offset;

  size_t size_of_vec = _node_data.size();

  // 11
  T temp = 0.0;
  _Jac_triplets_ph.emplace_back(
    _node_data[0]._index - slack_offset, _node_data[0]._index - slack_offset, temp);
  // 12
  _Jac_triplets_ph.emplace_back(_node_data[0]._index - slack_offset,
                                _node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                                temp);
  // 21
  _Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                               _node_data[0]._index - slack_offset,
                               temp);
  // 22
  _Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                               _node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                               temp);

  for (size_t i = 1; i < size_of_vec; i++) {

    if (_node_data[i]._index < slack_offset)
      continue; // neighbor is slack

    // 11
    _Jac_triplets_ph.emplace_back(
      _node_data[0]._index - slack_offset, _node_data[i]._index - slack_offset, temp);

    // 21
    _Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                                 _node_data[i]._index - slack_offset,
                                 temp);

    if (_node_data[i]._index < Non_PQ_offset)
      continue; // neighbor is slack or PV

    // 12
    _Jac_triplets_ph.emplace_back(_node_data[0]._index - slack_offset,
                                  _node_data[i]._index - PV_offset - 2 * slack_offset + all_offset,
                                  temp);
    // 22
    _Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                                 _node_data[i]._index - PV_offset - 2 * slack_offset + all_offset,
                                 temp);
  }
  std::copy(_Jac_triplets_ph.begin(), _Jac_triplets_ph.end(), std::back_inserter(J));
  std::copy(_Jac_triplets_V.begin(), _Jac_triplets_V.end(), std::back_inserter(J));
}

template<typename T, typename U>
void
NetworkNode<T, U>::CreateJacLineSignature_PV(GridNetwork<T, U>& Network, VT_<T>& J)
{
  _Jac_triplets_ph.reserve(_node_data.size() * 2);

  size_t slack_offset = Network._NetworkNodes_S.size();
  size_t all_offset = Network._NetworkNodes.size();
  size_t PV_offset = Network._NetworkNodes_PV.size();
  size_t Non_PQ_offset = slack_offset + PV_offset;

  size_t size_of_vec = _node_data.size();

  T temp = 0.0;
  _Jac_triplets_ph.emplace_back(
    _node_data[0]._index - slack_offset, _node_data[0]._index - slack_offset, temp);

  for (size_t i = 1; i < size_of_vec; i++) {
    if (_node_data[i]._index < slack_offset)
      continue; // neighbor is slack

    _Jac_triplets_ph.emplace_back(
      _node_data[0]._index - slack_offset, _node_data[i]._index - slack_offset, temp);

    if (_node_data[i]._index < Non_PQ_offset)
      continue; // neighbor is slack or PV

    _Jac_triplets_ph.emplace_back(_node_data[0]._index - slack_offset,
                                  _node_data[i]._index - PV_offset - 2 * slack_offset + all_offset,
                                  temp);
  }
  std::copy(_Jac_triplets_ph.begin(), _Jac_triplets_ph.end(), std::back_inserter(J));
}

template<typename T, typename U>
void
NetworkNode<T, U>::computePnQ(GridNetwork<T, U>& Network)
{
  P = 0.0;
  Q = 0.0;
  for (size_t i = 0; i < _node_data.size(); i++) {
    T V_square = _node_data[0]._voltage * _node_data[i]._voltage;
    std::complex<T> ybus = Network.getY_BusAt(_node_data[0]._index, _node_data[i]._index);
    T temp_P = ybus.real() * _node_data[i]._cos_phase + ybus.imag() * _node_data[i]._sin_phase;
    T temp_Q = ybus.real() * _node_data[i]._sin_phase - ybus.imag() * _node_data[i]._cos_phase;
    P += V_square * (temp_P);
    Q += V_square * (temp_Q);
  }
}

template<typename T, typename U>
void
NetworkNode<T, U>::computeJ_PQ(GridNetwork<T, U>& Network)
{
  VT_<T> Jac_triplets_ph;
  VT_<T> Jac_triplets_V;
  Jac_triplets_ph.reserve(_node_data.size() * 2);
  Jac_triplets_V.reserve(_node_data.size() * 2);

  size_t slack_offset = Network._NetworkNodes_S.size();
  size_t all_offset = Network._NetworkNodes.size();
  size_t PV_offset = Network._NetworkNodes_PV.size();
  size_t Non_PQ_offset = slack_offset + PV_offset;

  size_t size_of_vec = _node_data.size();

  T V_square = _node_data[0]._voltage * _node_data[0]._voltage;
  std::complex<T> ybus = Network.getY_BusAt(_node_data[0]._index, _node_data[0]._index);

  // 11
  T temp = /*ybus.real() * node_data[0]._sin_phase*/ -ybus.imag() /*_node_data[0]._cos_phase*/;
  temp = V_square * temp - Q;
  Jac_triplets_ph.emplace_back(
    _node_data[0]._index - slack_offset, _node_data[0]._index - slack_offset, temp);
  // 12
  temp = ybus.real() /* _node_data[0]._cos_phase + ybus.imag() * node_data[0]._sin_phase*/;
  temp = _node_data[0]._voltage * temp + P / _node_data[0]._voltage;
  Jac_triplets_ph.emplace_back(_node_data[0]._index - slack_offset,
                               _node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                               temp);
  // 21
  temp = ybus.real() /** _node_data[0]._cos_phase + ybus.imag() * node_data[0]._sin_phase*/;
  temp = -V_square * temp + P;
  Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                              _node_data[0]._index - slack_offset,
                              temp);
  // 22
  temp = /*ybus.real() * _node_data[0]._sin_phase*/ -ybus.imag() /** _node_data[0]._cos_phase*/;
  temp = _node_data[0]._voltage * temp + Q / _node_data[0]._voltage;
  Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                              _node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                              temp);

  for (size_t i = 1; i < size_of_vec; i++) {

    if (_node_data[i]._index < slack_offset)
      continue; // neighbor is slack

    V_square = _node_data[0]._voltage * _node_data[i]._voltage;
    ybus = Network.getY_BusAt(_node_data[0]._index, _node_data[i]._index);
    std::complex<T> ybus_2 = Network.getY_BusAt(_node_data[i]._index, _node_data[0]._index);

    // 11
    temp = ybus.real() * _node_data[i]._sin_phase - ybus.imag() * _node_data[i]._cos_phase;
    temp = V_square * temp;
    Jac_triplets_ph.emplace_back(
      _node_data[0]._index - slack_offset, _node_data[i]._index - slack_offset, temp);

    // 21
    temp = ybus.real() * _node_data[i]._cos_phase + ybus.imag() * _node_data[i]._sin_phase;
    temp = -V_square * temp;
    Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                                _node_data[i]._index - slack_offset,
                                temp);

    if (_node_data[i]._index < Non_PQ_offset)
      continue; // neighbor is slack or PV

    // 12
    temp = ybus.real() * _node_data[i]._cos_phase +
           ybus.imag() * _node_data[i]._sin_phase; // -sin(x) = sin(-x)
    temp = _node_data[0]._voltage * temp;
    Jac_triplets_ph.emplace_back(_node_data[0]._index - slack_offset,
                                 _node_data[i]._index - PV_offset - 2 * slack_offset + all_offset,
                                 temp);
    // 22
    temp = ybus.real() * _node_data[i]._sin_phase - ybus.imag() * _node_data[i]._cos_phase;
    temp = _node_data[0]._voltage * temp;

    Jac_triplets_V.emplace_back(_node_data[0]._index - PV_offset - 2 * slack_offset + all_offset,
                                _node_data[i]._index - PV_offset - 2 * slack_offset + all_offset,
                                temp);
  }
  _Jac_triplets_ph = std::move(Jac_triplets_ph);
  _Jac_triplets_V = std::move(Jac_triplets_V);
}

template<typename T, typename U>
void
NetworkNode<T, U>::computeJ_PV(GridNetwork<T, U>& Network)
{
  VT_<T> Jac_triplets_ph;
  Jac_triplets_ph.reserve(_node_data.size() * 2);

  size_t slack_offset = Network._NetworkNodes_S.size();
  size_t all_offset = Network._NetworkNodes.size();
  size_t PV_offset = Network._NetworkNodes_PV.size();
  size_t Non_PQ_offset = slack_offset + PV_offset;

  size_t size_of_vec = _node_data.size();

  T V_square = _node_data[0]._voltage * _node_data[0]._voltage;
  std::complex<T> ybus = Network.getY_BusAt(_node_data[0]._index, _node_data[0]._index);

  // 11
  T temp = /*ybus.real() * node_data[0]._sin_phase*/ -ybus.imag() /*_node_data[0]._cos_phase*/;
  temp = V_square * temp - Q;
  Jac_triplets_ph.emplace_back(
    _node_data[0]._index - slack_offset, _node_data[0]._index - slack_offset, temp);

  for (size_t i = 1; i < size_of_vec; i++) {
    if (_node_data[i]._index < slack_offset)
      continue; // neighbor is slack

    V_square = _node_data[0]._voltage * _node_data[i]._voltage;
    ybus = Network.getY_BusAt(_node_data[0]._index, _node_data[i]._index);

    temp = ybus.real() * _node_data[i]._sin_phase - ybus.imag() * _node_data[i]._cos_phase;
    temp = V_square * temp;
    Jac_triplets_ph.emplace_back(
      _node_data[0]._index - slack_offset, _node_data[i]._index - slack_offset, temp);

    if (_node_data[i]._index < Non_PQ_offset)
      continue; // neighbor is slack or PV

    temp = ybus.real() * _node_data[i]._cos_phase + ybus.imag() * _node_data[i]._sin_phase;
    temp = _node_data[0]._voltage * temp;
    Jac_triplets_ph.emplace_back(_node_data[0]._index - slack_offset,
                                 _node_data[i]._index - PV_offset - 2 * slack_offset + all_offset,
                                 temp);
  }
  _Jac_triplets_ph = std::move(Jac_triplets_ph);
}

template<typename T, typename U>
void
NetworkNode<T, U>::updateJ_PQ(GridNetwork<T, U>& Network)
{
  for (auto& tri : _Jac_triplets_ph)
    Network._Jacobian.coeffRef(tri.row(), tri.col()) = tri.value();
  for (auto& tri : _Jac_triplets_V)
    Network._Jacobian.coeffRef(tri.row(), tri.col()) = tri.value();
}

template<typename T, typename U>
void
NetworkNode<T, U>::updateJ_PV(GridNetwork<T, U>& Network)
{
  for (auto& tri : _Jac_triplets_ph)
    Network._Jacobian.coeffRef(tri.row(), tri.col()) = tri.value();
}

// Template Instantiation
template class NetworkNode<double, double>;

///!!! there are "if (n_index == 0): continue" statements in the pyth
/// code?
/// variables for slack is constant which does not conrtibute to Jac.