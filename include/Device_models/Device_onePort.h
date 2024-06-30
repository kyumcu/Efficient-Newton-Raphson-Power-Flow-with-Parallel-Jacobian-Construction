#pragma once

#include "Device_base.h"

template<typename T, typename U>
class OnePortDevice : public Device<T, U>
{

public:
  OnePortDevice()
    : Device<T, U>()
  {
  }
  virtual ~OnePortDevice() {}

  // Implement virtual methods from Device
  void update(U /* timestep */) override {}
  void initialize() override { this->_isInitialized = true; }
  T getPower() const override { return 0.0; }

  // One-port specific methods
  void setTerminalVoltage(T voltage) { _terminalVoltage = voltage; }
  T getTerminalVoltage() const { return _terminalVoltage; }

  void setTerminalCurrent(T current) { _terminalCurrent = current; }
  T getTerminalCurrent() const { return _terminalCurrent; }

protected:
  T _terminalVoltage = 0.0; // Voltage at the terminal of the device
  T _terminalCurrent = 0.0;
};