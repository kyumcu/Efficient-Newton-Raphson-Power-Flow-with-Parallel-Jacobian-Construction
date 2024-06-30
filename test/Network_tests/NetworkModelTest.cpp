#include <gtest/gtest.h>

#include "Utils/BasicMethods.h"

#include "Network_models/Network_Model.h"

TEST(NetworkModelTest_, VectorInitializeTest)
{

  using TU_NetworkNode = NetworkNode<double, double>;

  auto Network = GridNetwork<double, double>();

  std::shared_ptr<TU_NetworkNode> sp_node_1, sp_node_2, sp_node_3, sp_node_4, sp_node_5, sp_node_g;

  sp_node_1 = std::make_shared<TU_NetworkNode>(0, 1.06, 0.0, 0.0, 0.0, BusType::slack);
  sp_node_2 = std::make_shared<TU_NetworkNode>(1, 0, 0.0, 0.2, 0.2, BusType::PQ);
  sp_node_3 = std::make_shared<TU_NetworkNode>(2, 0, 0.0, -0.45, -0.15, BusType::PQ);
  sp_node_4 = std::make_shared<TU_NetworkNode>(3, 0, 0.0, -0.4, -0.05, BusType::PQ);
  sp_node_5 = std::make_shared<TU_NetworkNode>(4, 0, 0.0, -0.6, -0.1, BusType::PQ);
  sp_node_g = std::make_shared<TU_NetworkNode>(-1, 0, 0.0, 0.001, 0.0, BusType::PV);

  // push back of an L value copies and icresses the counter
  Network.addNode(sp_node_1);
  Network.addNode(sp_node_2);
  Network.addNode(sp_node_3);
  Network.addNode(sp_node_4);
  Network.addNode(sp_node_5);

  EXPECT_EQ(sp_node_1->getVoltageRms(), 1.06);

  std::vector<double> new_voltage(5, 2.0);

  Network.pullExtern(new_voltage, StateType::Voltage_rms);
  Network.pushToNodes_V();

  EXPECT_EQ(sp_node_1->getVoltageRms(), 2.0);
  EXPECT_EQ(sp_node_2->getVoltageRms(), 2.0);
  EXPECT_EQ(sp_node_3->getVoltageRms(), 2.0);
}

TEST(NetworkModelTest_, ConnectionAddingTest)
{

  using TU_NetworkNode = NetworkNode<double, double>;
  using TU_Conductor = Conductor<double, double>;

  auto Network = GridNetwork<double, double>();

  std::shared_ptr<TU_NetworkNode> sp_node_1, sp_node_2, sp_node_3, sp_node_4, sp_node_5, sp_node_g;

  sp_node_1 = std::make_shared<TU_NetworkNode>(0, 1.06, 0.0, 0.0, 0.0, BusType::slack);
  sp_node_2 = std::make_shared<TU_NetworkNode>(1, 0, 0.0, 0.2, 0.2, BusType::PQ);
  sp_node_3 = std::make_shared<TU_NetworkNode>(2, 0, 0.0, -0.45, -0.15, BusType::PQ);
  sp_node_4 = std::make_shared<TU_NetworkNode>(3, 0, 0.0, -0.4, -0.05, BusType::PQ);
  sp_node_5 = std::make_shared<TU_NetworkNode>(4, 0, 0.0, -0.6, -0.1, BusType::PQ);
  sp_node_g = std::make_shared<TU_NetworkNode>(-1, 0, 0.0, 0.001, 0.0, BusType::PV);

  // push back of an L value copies and icresses the counter
  Network.addNode(sp_node_1);
  Network.addNode(sp_node_2);
  Network.addNode(sp_node_3);
  Network.addNode(sp_node_4);
  Network.addNode(sp_node_5);

  EXPECT_EQ(sp_node_1->getVoltageRms(), 1.06);

  std::vector<double> new_voltage(5, 2.0);

  Network.pullExtern(new_voltage, StateType::Voltage_rms);
  Network.pushToNodes_V();

  auto sp_line_1_2 = std::make_shared<TU_Conductor>(sp_node_1, sp_node_2, 1 / 0.02, 1 / 0.06, true);
  Network.addConnection(sp_line_1_2);

  EXPECT_EQ(sp_node_1->node_devices.size(), 0);
  Network.resize();
  Network.orderANDClasifyNodes();
  Network.localizeDeviceList();

  EXPECT_EQ(sp_node_1->node_devices.size(), 1);
}
