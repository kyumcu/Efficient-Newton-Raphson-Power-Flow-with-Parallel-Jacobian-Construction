#include <iostream>
#include <omp.h>

#include "Network_models/Network_Model.h"

#include "Utils/DigestParsedData.h"
#include "Utils/MatpowerParser.h"
#include "Utils/TestHelpers.h"

namespace MMP = MatpowerParser;

int
main(int argc, char* argv[])
{
  omp_set_num_threads(std::stoi(argv[2]));

  Eigen::initParallel();
  Eigen::setNbThreads(1);

  MMP::PowerFlowData pfData;

  MMP::parsePowerFlowData(MPOWERDIR "data/" + std::string(argv[1]), pfData);

  for (auto i = 0; i < std::stoi(argv[3]); ++i) {
    auto Network = GridNetwork<double, double>();
    digest<double, double>(pfData, Network);

#ifdef TIME0
    {
      auto start = std::chrono::high_resolution_clock::now();
#endif

      Network.prepeare_PF();

      size_t iter_counter = 0;
      do {
        Network.do_PF();
        iter_counter++;
      } while (Network.getDeltaX().maxCoeff() > 1e-6);

#ifdef TIME0
      auto end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> elapsed = end - start;
      std::cout << "'converged': " << elapsed.count() << ",\n";
    }
#endif

#ifdef TIME
    std::cout << "converged in " << iter_counter << " iterations" << std::endl;
#endif
  }
  return 0;
}
