/*
#include <gtest/gtest.h>

#include <iostream>

#include "Utils/BasicMethods.h"
#include "Utils/MatrixOps.h"
#include "Utils/Y_BusGenerator.h"

#include "../Replication_tests/Stag5BusTest.h"

TEST(Y_BusGeneratorTest_, DummyCallTest) {
    v_sp_<Conductor<double, double>> connection_list;
    auto Ybus = Y_BusGenerate<double, double>(connection_list,
connection_list, 5);
}




TEST_F(Stag5BusTest, Y_BusCorrectness_5xStag) {

    // Print the matrix to the console
    Eigen::Matrix<std::complex<double>, 5, 5> Y_Bus =
Y_BusGenerate<double, double>(device_list, device_to_ground_list, 5);


    expectMatrixNear(ref_YBus, Y_Bus);
    //std::cout << "Matrix:\n" << Y_Bus << std::endl;
}
*/