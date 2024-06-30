#pragma once

#include <unordered_map>
#include <utility> // For std::pair
#include <vector>

#include "Utils/MatpowerParser.h"
#include "Utils/Naming.h"

#include "Network_models/Network_Model.h"

namespace MPP = MatpowerParser;

// populate node type vectors
// this creates index maping
// loop generators to set voltage and powerFlows at nodes
// assign nodes to network inthe order S PV PQ
// loop buses  to accumulate Bs,Gs using index_map
// loop branches to populate transformers and lines
// also acumulate line charging sus
// add accumulated sus as to connectionto ground vector
// record reverse map

template<typename T, typename U>
void
constructNodes(const MPP::PowerFlowData& data,
               GridNetwork<T, U>& Network,
               v_sp_<NetworkNode<T, U>>& node_list)
{

  for (size_t i = 0; i < data.buses.size(); ++i) {
    node_list.emplace_back(
      std::make_shared<NetworkNode<T, U>>(data.buses[i].bus_i,
                                          i,
                                          static_cast<T>(data.buses[i].Vm),
                                          degToRadians<T>(data.buses[i].Va),
                                          static_cast<T>(-data.buses[i].Pd / data.baseMVA),
                                          static_cast<T>(-data.buses[i].Qd / data.baseMVA),
                                          intToBusType(data.buses[i].type)));
  }
}

template<typename T, typename U>
void
getPFandICfromGen(const MPP::PowerFlowData& data,
                  GridNetwork<T, U>& Network,
                  v_sp_<NetworkNode<T, U>>& node_list,
                  std::unordered_map<int, size_t>& tag2legacy)
{

  for (size_t i = 0; i < data.generators.size(); ++i) {
    if (data.generators[i].status) {
      size_t bus_index = tag2legacy[data.generators[i].bus];

      node_list[bus_index]->setGenConnected(true);

      node_list[bus_index]->setVoltageRms(static_cast<T>(data.generators[i].Vg));

      auto temp = node_list[bus_index]->getPowerFlowActive();
      node_list[bus_index]->setPowerFlowActive(
        static_cast<T>(data.generators[i].Pg / data.baseMVA) + temp);

      temp = node_list[bus_index]->getPowerFlowReactive();
      node_list[bus_index]->setPowerFlowReactive(
        static_cast<T>(data.generators[i].Qg / data.baseMVA) + temp);
    }
  }
}

template<typename T, typename U>
void
constructLines(const MPP::PowerFlowData& data,
               GridNetwork<T, U>& Network,
               v_sp_<NetworkNode<T, U>>& node_list,
               std::unordered_map<int, size_t>& tag2legacy,
               std::vector<T>& shunt_susceptance)
{
  for (size_t i = 0; i < data.branches.size(); ++i) {
    if (!data.branches[i].status)
      continue; // todo: extend for status change

    std::complex<T> temp_admittance =
      calculateAdmittance(std::complex<T>(data.branches[i].r, data.branches[i].x));
    std::complex<T> temp_transform = 1.0;

    size_t temp_term_1 = tag2legacy[data.branches[i].fbus];
    size_t temp_term_2 = tag2legacy[data.branches[i].tbus];

    if (((data.branches[i].ratio != 0) && (data.branches[i].ratio != 1)) ||
        (data.branches[i].angle != 0)) { // we have a transformer

      if (data.branches[i].ratio == 0) // there are transformers with 0 ratio but shift phase
        temp_transform = polarDegToCart(1.0, data.branches[i].angle);
      else
        temp_transform = polarDegToCart(data.branches[i].ratio, data.branches[i].angle);

      Network.addTransformer(std::make_shared<Transformer<T, U>>(node_list[temp_term_1],
                                                                 node_list[temp_term_2],
                                                                 temp_admittance.real(),
                                                                 temp_admittance.imag(),
                                                                 (bool)data.branches[i].status,
                                                                 data.branches[i].b,
                                                                 temp_transform));
      // p.u. sus. sacale with the square of voltage
      shunt_susceptance[temp_term_1] +=
        (data.branches[i].b) / (2.0 * std::abs(temp_transform) * (std::abs(temp_transform)));

    } else {
      Network.addConnection(std::make_shared<Conductor<T, U>>(node_list[temp_term_1],
                                                              node_list[temp_term_2],
                                                              temp_admittance.real(),
                                                              temp_admittance.imag(),
                                                              (bool)data.branches[i].status,
                                                              data.branches[i].b));
      shunt_susceptance[temp_term_1] += data.branches[i].b / 2.0;
    }
    shunt_susceptance[temp_term_2] += data.branches[i].b / 2.0;
  }
}

template<typename T, typename U>
void
constructGroundConnection(const MPP::PowerFlowData& data,
                          GridNetwork<T, U>& Network,
                          v_sp_<NetworkNode<T, U>>& node_list,
                          std::vector<T>& shunt_conductance,
                          std::vector<T>& shunt_susceptance)
{
  for (size_t i = 0; i < node_list.size(); ++i) {
    if (shunt_conductance[i] || shunt_susceptance[i])
      Network.addGroundConnection(std::make_shared<Conductor<T, U>>(
        node_list[i], Network.getGNode(), shunt_conductance[i], shunt_susceptance[i], true));
  }
}

template<typename T, typename U>
void
digest(const MPP::PowerFlowData& data, GridNetwork<T, U>& Network)
{

  Network._caseName = data.caseName;

  v_sp_<NetworkNode<T, U>> node_list;

  std::unordered_map<int, size_t> tag2legacy;
  tag2legacy.reserve(data.buses.size());

  // All buses have a (bus index) which is not linear,
  // tag2legacy provides a linearly increasing index
  for (size_t i = 0; i < data.buses.size(); ++i)
    tag2legacy[data.buses[i].bus_i] = i;

  constructNodes(data, Network, node_list);

  getPFandICfromGen(data, Network, node_list, tag2legacy);

  // branches have (Shunt Y) which is distributed to each connecting
  // node zero init all values for all nodes
  std::vector<T> shunt_susceptance(data.buses.size());
  std::vector<T> shunt_conductance(data.buses.size());

  for (size_t i = 0; i < data.buses.size(); ++i) { // Gs, Bs
    shunt_susceptance[i] += data.buses[i].Bs / data.baseMVA;
    shunt_conductance[i] += data.buses[i].Gs / data.baseMVA;
  }

  constructLines(data, Network, node_list, tag2legacy, shunt_susceptance);

  constructGroundConnection(data, Network, node_list, shunt_conductance, shunt_susceptance);

  Network.setTag2legacy(std::move(tag2legacy));
  Network.setNetworkNodes(std::move(node_list));
}