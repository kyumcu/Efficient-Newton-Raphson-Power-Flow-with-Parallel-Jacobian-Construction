#pragma once

#include "../Device_models/Conductor.h"

template<typename T, typename U>
class Transformer : public Conductor<T, U>
{

  using sp_TU_NetworkNode = std::shared_ptr<NetworkNode<T, U>>;
  using TU_Conductor = Conductor<T, U>;

public:
  std::complex<T> _ratio = 0;

  Transformer(sp_TU_NetworkNode terminal_1,
              sp_TU_NetworkNode terminal_2,
              T conductance,
              T susceptance,
              bool status,
              T _line_charging_susceptance,
              std::complex<T> ratio)
    : TU_Conductor(terminal_1, terminal_2, conductance, susceptance, status)
    , _ratio(ratio)
  {
  }

  const std::complex<T> getRatio() const { return _ratio; }
};