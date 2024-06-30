#pragma once

#include "../Network_models/Network_Node.h"
#include "Device_onePort.h"

template<typename T, typename U>
class Conductor : public OnePortDevice<T, U>
{

  using sp_TU_NetworkNode = std::shared_ptr<NetworkNode<T, U>>;

public:
  sp_TU_NetworkNode _terminals[2];

  size_t _terminal_indexes[2] = {};

  T _conductance = 0.0;
  T _susceptance = 0.0;
  T _line_charging_susceptance = 0.0;

  bool _status = false;

  Conductor(sp_TU_NetworkNode terminal_1,
            sp_TU_NetworkNode terminal_2,
            T conductance,
            T susceptance,
            bool status)
    : OnePortDevice<T, U>()
    , _conductance(conductance)
    , _susceptance(susceptance)
    , _status(status)
  {
    _terminals[0] = terminal_1;
    _terminals[1] = terminal_2;

    updateTerminalIndexes();
  }

  Conductor(sp_TU_NetworkNode terminal_1,
            sp_TU_NetworkNode terminal_2,
            T conductance,
            T susceptance,
            bool status,
            T line_charging_susceptance)
    : Conductor(terminal_1, terminal_2, conductance, susceptance, status)
  {
    _line_charging_susceptance = line_charging_susceptance;
  }

  // void setTerminals(T conductance) { _conductance = conductance; }
  const size_t* getTerminalIndexes() const { return _terminal_indexes; }

  void updateTerminalIndexes()
  {
    _terminal_indexes[0] = _terminals[0]->getNodeIndex();
    _terminal_indexes[1] = _terminals[1]->getNodeIndex();
  }

  void setTerminalConductance(T conductance) { _conductance = conductance; }
  const T getTerminalConductance() const { return _conductance; }

  void setTerminalSusceptance(T susceptance) { _susceptance = susceptance; }
  const T getTerminalSusceptance() const { return _susceptance; }

  const bool isActive() const { return _status; }
};
