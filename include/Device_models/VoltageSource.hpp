#pragma once


#include "Network_models/Network_device_onePort.hpp"


template <typename T>
class VoltageSource_base : public OnePortDevice {

    VoltageSource() {}
    explicit VoltageSource(T volt) : OnePortDevice(), _voltage(volt) {}
   
    virtual ~VoltageSource() = default;

    public:
    void setTerminalResistance(T resistance) { _resistance = resistance; }
    T getTerminalResistance() const { return _resistance; }

    protected:
    T _voltage = 0.0; 
};



template <typename T>
class VoltageSource_constant_DC : public VoltageSource_base<T> {

    VoltageSource_constant_DC() {}
    explicit VoltageSource_constant_DC(T volt) : VoltageSource_base<T>(volt) {}

   
    virtual ~VoltageSource() = default;


};