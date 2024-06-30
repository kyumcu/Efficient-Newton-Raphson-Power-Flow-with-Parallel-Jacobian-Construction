#include <gtest/gtest.h>

#include "Network_models/Network_Node.h"

// Test fixture for NetworkNode tests
template<typename T>
class NetworkNodeTest : public ::testing::Test
{
protected:
  NetworkNode<T, T> defaultNode; // Instance for testing the default constructor

  // Setup any necessary data for each test
  virtual void SetUp() {}

  // Clean up after each test
  virtual void TearDown() {}
};

// Define the types to test with NetworkNode
using MyTypes = ::testing::Types<float, double>;
TYPED_TEST_SUITE(NetworkNodeTest, MyTypes);

// Test the default constructor
TYPED_TEST(NetworkNodeTest, DefaultConstructor)
{
  EXPECT_EQ(this->defaultNode.node_index, -1);
  EXPECT_FLOAT_EQ(this->defaultNode.voltage_rms, 0.0f);
  EXPECT_FLOAT_EQ(this->defaultNode.phase_angle, 0.0f);
  EXPECT_FLOAT_EQ(this->defaultNode.power_flow_reactive, 0.0f);
  EXPECT_FLOAT_EQ(this->defaultNode.power_flow_reactive, 0.0f);
  EXPECT_EQ(this->defaultNode.type, BusType::PQ);
}

// Test the parameterized constructor(index only)
TYPED_TEST(NetworkNodeTest, ParameterIsIndexConstructor)
{
  NetworkNode<TypeParam, TypeParam> node(1);

  EXPECT_EQ(node.node_index, 1);
  EXPECT_FLOAT_EQ(this->defaultNode.voltage_rms, 0.0f);
  EXPECT_FLOAT_EQ(this->defaultNode.phase_angle, 0.0f);
  EXPECT_FLOAT_EQ(this->defaultNode.power_flow_reactive, 0.0f);
  EXPECT_FLOAT_EQ(this->defaultNode.power_flow_reactive, 0.0f);
  EXPECT_EQ(this->defaultNode.type, BusType::PQ);
}

// Test the parameterized constructor
TYPED_TEST(NetworkNodeTest, ParameterizedConstructor)
{
  NetworkNode<TypeParam, TypeParam> node(1, 120.0, 30.0, 50.0, 20.0, BusType::PV);

  EXPECT_EQ(node.node_index, 1);
  EXPECT_FLOAT_EQ(node.voltage_rms, 120.0);
  EXPECT_FLOAT_EQ(node.phase_angle, 30.0);
  EXPECT_FLOAT_EQ(node.power_flow_active, 50.0);
  EXPECT_FLOAT_EQ(node.power_flow_reactive, 20.0);
  EXPECT_EQ(node.type, BusType::PV);
}

TYPED_TEST(NetworkNodeTest, NodeInitTest)
{

  using TU_NetworkNode = NetworkNode<TypeParam, TypeParam>;

  std::shared_ptr<TU_NetworkNode> sp_node_1, sp_node_2, sp_node_3, sp_node_4, sp_node_5, sp_node_g;
  std::vector<std::shared_ptr<TU_NetworkNode>> bus_list;

  sp_node_1 = std::make_shared<TU_NetworkNode>(0, 1.06, 0.0, 0.0, 0.0, BusType::slack);
  sp_node_2 = std::make_shared<TU_NetworkNode>(1, 0, 0.0, 0.2, 0.2, BusType::PQ);
  sp_node_3 = std::make_shared<TU_NetworkNode>(2, 0, 0.0, -0.45, -0.15, BusType::PQ);
  sp_node_4 = std::make_shared<TU_NetworkNode>(3, 0, 0.0, -0.4, -0.05, BusType::PQ);
  sp_node_5 = std::make_shared<TU_NetworkNode>(4, 0, 0.0, -0.6, -0.1, BusType::PQ);
  sp_node_g = std::make_shared<TU_NetworkNode>(-1, 0, 0.0, 0.001, 0.0, BusType::PV);

  // push back of an L value copies and icresses the counter
  bus_list.push_back(sp_node_1);
  bus_list.push_back(sp_node_2);
  bus_list.push_back(sp_node_3);
  bus_list.push_back(sp_node_4);
  bus_list.push_back(sp_node_5);

  EXPECT_EQ(bus_list[0]->getVoltageRms(), (TypeParam)1.06);
}
