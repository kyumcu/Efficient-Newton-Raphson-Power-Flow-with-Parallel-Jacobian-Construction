#ifdef DEBUG
#include <iostream>
#define DEBUG_COUT(msg, x) (std::cout << (msg) << (x) << std::endl)
#else
#define DEBUG_COUT(msg, x)
#endif