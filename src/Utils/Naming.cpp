
#include <iostream>

#include "Utils/Naming.h"

// Overload the << operator to handle BusType
std::ostream&
operator<<(std::ostream& os, const BusType& busType)
{
  switch (busType) {
    case BusType::slack:
      os << "slack";
      break;
    case BusType::PV:
      os << "PV";
      break;
    case BusType::PQ:
      os << "PQ";
      break;
    default:
      os << "Unknown BusType";
  }
  return os;
}

// Function to cast integer to BusType
BusType
intToBusType(int input)
{

  int val = 2; // PQ default

  if (input == 2)      // if PV
    val = 1;           // PV
  else if (input == 3) // Slack
    val = 0;           // S

  return static_cast<BusType>(val);
}